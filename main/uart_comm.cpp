#include "uart_comm.h"
#include <string>
#include <vector>
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "esp_log.h"

static const char* TAG = "uart_comm";

static StreamBufferHandle_t s_raw_stream = nullptr;
static uart_port_t s_uart;
static SemaphoreHandle_t s_uart_mutex;     // serialize TX
static QueueHandle_t s_q_rtcm;
static QueueHandle_t s_q_nmea;
static QueueHandle_t s_q_sky_unsol = nullptr;

// ---- Waiter registry (tiny fixed table) ----
struct Waiter {
    bool             in_use;
    uint8_t          reply_id;
    QueueHandle_t    q;          // each waiter owns a 1-item queue for its reply
};
static constexpr int MAX_WAITERS = 8;
static Waiter s_waiters[MAX_WAITERS];
static SemaphoreHandle_t s_waiters_mutex;

// deliver a parsed SkyTraq frame to matching waiter or unsolicited queue
static void deliver_sky(const uint8_t* pay, uint16_t len) {
    if (len == 0) return;
    uint8_t id = pay[0];

    xSemaphoreTake(s_waiters_mutex, portMAX_DELAY);
    for (int i=0;i<MAX_WAITERS;i++){
        if (s_waiters[i].in_use && s_waiters[i].reply_id == id) {
            SkytraqFrame f{}; f.len = len; memcpy(f.data, pay, len);
            xQueueOverwrite(s_waiters[i].q, &f);  // wake waiter
            xSemaphoreGive(s_waiters_mutex);
            return;
        }
    }
    xSemaphoreGive(s_waiters_mutex);

    if (s_q_sky_unsol) {
        SkytraqFrame f{}; f.len = len; memcpy(f.data, pay, len);
        xQueueSend(s_q_sky_unsol, &f, 0);
    }
}

// ---- CRC24Q for RTCM ----
static uint32_t crc24q(const uint8_t* data, size_t len) {
    uint32_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint32_t)data[i]) << 16;
        for (int b = 0; b < 8; ++b) {
            crc <<= 1;
            if (crc & 0x1000000) crc ^= 0x1864CFB;
        }
        crc &= 0xFFFFFF;
    }
    return crc & 0xFFFFFF;
}

// ---- Reader task: demux SkyTraq / NMEA / RTCM ----
static void reader_task(void*) {
    std::vector<uint8_t> rx(512);
    // NMEA
    std::string line; line.reserve(128);
    // SkyTraq framing
    enum { S_A0, S_A1, S_LH, S_LL, S_PAY, S_CS, S_T1, S_T2 } sst = S_A0;
    uint16_t slen=0, shav=0; uint8_t scs=0; uint8_t spay[300];

    // RTCM
    enum { R_SYNC, R_HDR1, R_HDR2, R_PAY } rst = R_SYNC;
    uint8_t rh1=0, rh2=0; int rleft=0; RtcmFrame rcur{};

    for (;;) {
        // Only read; no parsing inside mutex
        int n = uart_read_bytes(s_uart, rx.data(), rx.size(), pdMS_TO_TICKS(50));
        if (n <= 0) continue;

        if (s_raw_stream) {
            size_t sent = xStreamBufferSend(s_raw_stream, rx.data(), n, 0);
            (void)sent; // optional: track drops if sent < n
        }

        for (int i=0;i<n;i++){
            uint8_t b = rx[i];

            // --- Try SkyTraq first (framed) ---
            switch (sst) {
                case S_A0:  if (b==0xA0){ sst=S_A1; } else goto try_nmea_rtcm; break;
                case S_A1:  sst = (b==0xA1) ? S_LH : S_A0; break;
                case S_LH:  slen = ((uint16_t)b)<<8; sst=S_LL; break;
                case S_LL:  slen |= b; shav=0; scs=0; if (slen>sizeof(spay)){ sst=S_A0; break; } sst=S_PAY; break;
                case S_PAY: spay[shav++] = b; scs ^= b; if (shav==slen) sst=S_CS; break;
                case S_CS:  if (scs!=b){ sst=S_A0; break; } sst=S_T1; break;
                case S_T1:  sst = (b==0x0D) ? S_T2 : S_A0; break;
                case S_T2:
                    if (b==0x0A) deliver_sky(spay, slen);
                    sst=S_A0; break;
            }
            continue;

        try_nmea_rtcm:
            // --- RTCM (0xD3) ---
            switch (rst){
                case R_SYNC: if (b==0xD3){ rcur.len=0; rcur.data[rcur.len++]=b; rst=R_HDR1; } break;
                case R_HDR1: rh1=b; rcur.data[rcur.len++]=b; rst=R_HDR2; break;
                case R_HDR2:{
                    rh2=b; rcur.data[rcur.len++]=b;
                    int len10 = ((int)(rh1 & 0x03)<<8) | rh2;
                    if (len10<0 || len10>1023){ rst=R_SYNC; break; }
                    rleft = len10 + 3; // payload+CRC
                    rst=R_PAY; break;
                }
                case R_PAY:
                    rcur.data[rcur.len++] = b;
                    if (--rleft==0){
                        if (rcur.len >= 6){
                            uint32_t calc = crc24q(rcur.data, rcur.len-3);
                            uint32_t gotCrc = ((uint32_t)rcur.data[rcur.len-3] << 16) |
                                              ((uint32_t)rcur.data[rcur.len-2] << 8)  |
                                               (uint32_t)rcur.data[rcur.len-1];
                            if (calc==gotCrc && s_q_rtcm){
                                RtcmFrame out{}; out.len = rcur.len; memcpy(out.data, rcur.data, rcur.len);
                                xQueueSend(s_q_rtcm, &out, 0);
                            }
                        }
                        rst=R_SYNC;
                    }
                    break;
            }

            // --- NMEA ($ … \n) ---
            char c = (char)b;
            if (c=='\n'){
                while (!line.empty() && (line.back()=='\r'||line.back()=='\n')) line.pop_back();
                if (s_q_nmea && !line.empty() && line.front()=='$'){
                    NmeaLine nl{}; nl.len = (line.size() > 255)? 255 : line.size();
                    memcpy(nl.txt, line.data(), nl.len); nl.txt[nl.len] = 0;
                    xQueueSend(s_q_nmea, &nl, 0);
                }
                line.clear();
            } else if ((b>=0x20 && b<=0x7E) || c=='\r' || c=='\t') {
                if (line.size()<255) line.push_back(c);
            } else {
                if (!line.empty()) line.clear();
            }
        }
    }
}

// ---- Public API ----
void uart_comm_start(uart_port_t uart, int baud, int tx_pin, int rx_pin, size_t rx_buf_bytes)
{
    s_uart = uart;
    s_uart_mutex    = xSemaphoreCreateMutex();
    s_waiters_mutex = xSemaphoreCreateMutex();
    memset(s_waiters, 0, sizeof(s_waiters));

    for (int i = 0; i < MAX_WAITERS; ++i) {
    s_waiters[i].in_use   = false;
    s_waiters[i].reply_id = 0;
    s_waiters[i].q        = xQueueCreate(1, sizeof(SkytraqFrame));
    configASSERT(s_waiters[i].q);  // make sure it really exists
    }

    uart_config_t cfg = {};
    cfg.baud_rate = baud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if ESP_IDF_VERSION_MAJOR >= 5
    cfg.source_clk = UART_SCLK_APB;
#endif
    uart_driver_install(uart, rx_buf_bytes, 0, 0, nullptr, 0);
    uart_param_config(uart, &cfg);
    uart_set_pin(uart, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    s_q_rtcm = xQueueCreate(32, sizeof(RtcmFrame));
    s_q_nmea = xQueueCreate(16, sizeof(NmeaLine));

    // 32 KB raw pipe; trigger on every byte (trigger level = 1)
    s_raw_stream = xStreamBufferCreate(32 * 1024, 1);

    xTaskCreatePinnedToCore(reader_task, "uart_reader", 4096, nullptr, 7, nullptr, tskNO_AFFINITY);
}

static QueueHandle_t waiter_register(uint8_t reply_id) {
    xSemaphoreTake(s_waiters_mutex, portMAX_DELAY);
    for (int i=0;i<MAX_WAITERS;i++){
        if (!s_waiters[i].in_use) {
            s_waiters[i].in_use   = true;
            s_waiters[i].reply_id = reply_id;
            xQueueReset(s_waiters[i].q);     // clear stale data
            QueueHandle_t q = s_waiters[i].q;
            xSemaphoreGive(s_waiters_mutex);
            return q;
        }
    }
    xSemaphoreGive(s_waiters_mutex);
    return nullptr;
}

static void waiter_unregister(QueueHandle_t q) {
    xSemaphoreTake(s_waiters_mutex, portMAX_DELAY);
    for (int i=0;i<MAX_WAITERS;i++){
        if (s_waiters[i].q == q) {
            s_waiters[i].in_use   = false;
            s_waiters[i].reply_id = 0;
            xQueueReset(s_waiters[i].q);     // leave queue allocated
            break;
        }
    }
    xSemaphoreGive(s_waiters_mutex);
}

StreamBufferHandle_t uart_comm_get_raw_stream() { return s_raw_stream; }
QueueHandle_t uart_comm_get_rtcm_queue() { return s_q_rtcm; }
QueueHandle_t uart_comm_get_nmea_queue() { return s_q_nmea; }
void uart_comm_set_unsolicited_sky_queue(QueueHandle_t q) { s_q_sky_unsol = q; }

bool uart_comm_sky_send_and_wait(const uint8_t* pay, uint16_t PL,
                                 uint8_t expect_reply_id,
                                 uint8_t* rx_out, uint16_t* rx_len,
                                 TickType_t timeout)
{
    if (PL==0 || pay==nullptr) return false;
    // register waiter
    QueueHandle_t myq = waiter_register(expect_reply_id);
    if (!myq) return false;

    // build frame
    uint8_t frame[2+2+256+1+2]; size_t k=0; uint8_t cs=0;
    frame[k++]=0xA0; frame[k++]=0xA1;
    frame[k++]= PL>>8; frame[k++]= PL & 0xFF;
    memcpy(&frame[k], pay, PL); for (int i=0;i<PL;i++) cs ^= pay[i]; k += PL;
    frame[k++]=cs; frame[k++]=0x0D; frame[k++]=0x0A;

    // send
    if (xSemaphoreTake(s_uart_mutex, timeout) != pdTRUE) { waiter_unregister(myq); return false; }
    uart_flush_input(s_uart);                          // flush BEFORE
    int w = uart_write_bytes(s_uart, (const char*)frame, k);
    xSemaphoreGive(s_uart_mutex);
    if (w != (int)k) { waiter_unregister(myq); return false; }

    // wait for reply
    SkytraqFrame f{};
    bool ok = (xQueueReceive(myq, &f, timeout) == pdTRUE);
    if (ok && rx_out && rx_len && *rx_len >= f.len) {
        memcpy(rx_out, f.data, f.len);
        *rx_len = f.len;
    }
    waiter_unregister(myq);
    return ok;
}
