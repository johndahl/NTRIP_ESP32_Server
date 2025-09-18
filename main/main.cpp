// ESP32-WROVER-E | Single-UART GPS + NTRIP Source + Bluetooth SPP Bridge (WiFi list + LED states)
// - One GPS UART (UART1): transparent SPP<->UART, extract RTCM3 (CRC checked) -> NTRIP SOURCE
// - WiFi fallback list: tries SSIDs in order until connected
// - LED: OFF (no WiFi), BLINK (WiFi up, NTRIP down), SOLID (NTRIP streaming)

#include <string>
#include <vector>
#include <algorithm>
#include <cstring>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

// Wi-Fi
#include "esp_wifi.h"

// UART
#include "driver/uart.h"
#include "driver/gpio.h"

// TCP (NTRIP)
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/tcp.h>   // keepalive opts

// Bluetooth Classic SPP
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "esp_gap_bt_api.h"

#include "wifi_credentials.h"  // your WiFi credentials
#include "ntrip_credentials.h" // your NTRIP credentials

static const char* TAG = "NTRIP_BT_SPP";

// GPS serial (single port for everything)
static constexpr int GPS_BAUD = 115200;   // << you raised baud; keep it here

// UART pin map (bidirectional)
static constexpr uart_port_t UART_GPS   = UART_NUM_1;
static constexpr int         GPS_TX_PIN = 26;  // ESP -> GPS RX
static constexpr int         GPS_RX_PIN = 27;  // GPS TX -> ESP

// GGA injection to caster (OFF for Source)
static constexpr bool        ENABLE_GGA_INJECT   = false;
static constexpr uint32_t    GGA_INJECT_EVERY_MS = 10000;

// Buffers & timing
static constexpr size_t UART_RX_BUF        = 16 * 1024;
static constexpr int    UART_READ_TO_MS    = 50;
static constexpr size_t NET_WRITE_CHUNK    = 1024;
static constexpr int    RECONNECT_DELAY_MS = 5000;

// Bluetooth SPP
static constexpr const char* SPP_SERVER_NAME = "ESP32_NMEA";

// LED status pin
static constexpr gpio_num_t LED_GPIO = GPIO_NUM_2;
static constexpr TickType_t BLINK_MS = 500;      // 1 Hz
static constexpr bool LED_ACTIVE_LOW = false;
// =======================================================

// ---- Event bits ----
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;  // got IP
static const int NTRIP_UP_BIT       = BIT1;  // streaming

static SemaphoreHandle_t gga_mutex;
static std::string last_gga;

// ---- WiFi multi-AP state ----
static int  s_wifi_index = 0;
static bool s_ever_got_ip = false;
static int  s_retries_on_current = 0;
static const int MAX_RETRIES_BEFORE_ROTATE = 3;

// ---------- Wi-Fi ----------
static void wifi_apply_index(int idx) {
    wifi_config_t wc = {};
    std::strncpy((char*)wc.sta.ssid, WIFI_LIST[idx].ssid, sizeof(wc.sta.ssid));
    std::strncpy((char*)wc.sta.password, WIFI_LIST[idx].pass, sizeof(wc.sta.password));
    wc.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // adjust if you have OPEN networks
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_LOGI(TAG, "WiFi: trying SSID \"%s\" (idx=%d)", WIFI_LIST[idx].ssid, idx);
}

static void wifi_connect_current() {
    wifi_apply_index(s_wifi_index);
    ESP_ERROR_CHECK(esp_wifi_connect());
}

static void wifi_try_next() {
    s_wifi_index = (s_wifi_index + 1) % WIFI_LIST_LEN;
    s_retries_on_current = 0;
    wifi_connect_current();
}

static void wifi_event_handler(void*, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        wifi_connect_current();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected (idx=%d), reason=%d", s_wifi_index,
                 data ? ((wifi_event_sta_disconnected_t*)data)->reason : -1);

        if (!s_ever_got_ip) {
            // Early stage: rotate through configured SSIDs until one connects
            wifi_try_next();
        } else {
            // We had a connection before; try a few quick retries on the same SSID
            if (++s_retries_on_current >= MAX_RETRIES_BEFORE_ROTATE) {
                ESP_LOGW(TAG, "Rotating to next SSID after %d retries", s_retries_on_current);
                wifi_try_next();
            } else {
                ESP_LOGI(TAG, "Reconnecting to same SSID (attempt %d)", s_retries_on_current);
                esp_wifi_connect();
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        auto* e = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&e->ip_info.ip));
        s_ever_got_ip = true;
        s_retries_on_current = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, nullptr));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Smoother power draw; helps LED stability
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
}

// ---------- UART ----------
static void uart_init_gps() {
    uart_config_t cfg = {};
    cfg.baud_rate = GPS_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
#if ESP_IDF_VERSION_MAJOR >= 5
    cfg.source_clk = UART_SCLK_APB;
#endif
    ESP_ERROR_CHECK(uart_driver_install(UART_GPS, UART_RX_BUF, 0, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_GPS, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_GPS, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// ---------- NTRIP (v1 SOURCE push) ----------
static int connect_ntrip_socket() {
    struct addrinfo hints = {};
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_family   = AF_UNSPEC;

    struct addrinfo* res = nullptr;
    if (getaddrinfo(CASTER_HOST, nullptr, &hints, &res) != 0 || !res) {
        ESP_LOGE(TAG, "DNS failed for %s", CASTER_HOST);
        return -1;
    }
    int sock = -1;
    for (auto p = res; p; p = p->ai_next) {
        if (p->ai_family == AF_INET)
            ((sockaddr_in*)p->ai_addr)->sin_port = htons(CASTER_PORT);
#if LWIP_IPV6
        else if (p->ai_family == AF_INET6)
            ((sockaddr_in6*)p->ai_addr)->sin6_port = htons(CASTER_PORT);
#endif
        sock = ::socket(p->ai_family, p->ai_socktype, 0);
        if (sock < 0) continue;

        int ka = 1;  setsockopt(sock, SOL_SOCKET,  SO_KEEPALIVE, &ka, sizeof(ka));
        int idle = 30; setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE,  &idle, sizeof(idle));
        int intv = 10; setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &intv, sizeof(intv));
        int cnt  = 3;  setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,  sizeof(cnt));

        if (::connect(sock, p->ai_addr, p->ai_addrlen) == 0) { freeaddrinfo(res); return sock; }
        ::close(sock); sock = -1;
    }
    freeaddrinfo(res);
    return -1;
}

static bool ntrip_v1_handshake(int sock) {
    std::string req = "SOURCE ";
    req += NTRIP_SOURCE_PW;
    req += " ";
    if (MOUNTPOINT_WITH_SLASH) req += "/";
    req += NTRIP_MOUNTPOINT;
    req += "\r\nSource-Agent: NTRIP ESP32-IDF-NTRIP/BTSPP\r\n\r\n";

    ssize_t w = send(sock, req.data(), req.size(), 0);
    if (w < 0 || (size_t)w != req.size()) return false;

    char buf[512];
    int n = recv(sock, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return false;
    buf[n] = '\0';
    std::string resp(buf, n);

    if (resp.find("ICY 200") != std::string::npos ||
        resp.find("HTTP/1.0 200") != std::string::npos ||
        resp.find("HTTP/1.1 200") != std::string::npos) {
        ESP_LOGI(TAG, "Caster accepted");
        return true;
    }
    if (resp.find("ERROR - Bad Password") != std::string::npos) {
        ESP_LOGE(TAG, "Bad SOURCE password");
    } else {
        ESP_LOGE(TAG, "Unexpected caster response: %.*s", n, buf);
    }
    return false;
}

// ---------- LED status task ----------
static void led_task(void*) {
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(LED_GPIO, GPIO_FLOATING);
    gpio_set_drive_capability(LED_GPIO, GPIO_DRIVE_CAP_0);

    bool blink_on = false;
    for (;;) {
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);

        if (bits & NTRIP_UP_BIT) {
            // Solid ON
            gpio_set_level(LED_GPIO, LED_ACTIVE_LOW ? 0 : 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else if (bits & WIFI_CONNECTED_BIT) {
            // Blink
            blink_on = !blink_on;
            int lvl = blink_on ? (LED_ACTIVE_LOW ? 0 : 1) : (LED_ACTIVE_LOW ? 1 : 0);
            gpio_set_level(LED_GPIO, lvl);
            vTaskDelay(pdMS_TO_TICKS(BLINK_MS));
        } else {
            // OFF
            gpio_set_level(LED_GPIO, LED_ACTIVE_LOW ? 1 : 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ---------- Bluetooth SPP ----------
static uint32_t g_spp_handle = 0;
static SemaphoreHandle_t spp_mutex;

static void spp_write_bytes(const uint8_t* data, size_t len) {
    if (!len) return;
    xSemaphoreTake(spp_mutex, portMAX_DELAY);
    uint32_t h = g_spp_handle;
    xSemaphoreGive(spp_mutex);
    if (!h) return;

    // modest chunks keep BT stack responsive
    while (len) {
        size_t n = len > 256 ? 256 : len;
        esp_spp_write(h, n, const_cast<uint8_t*>(data));
        data += n;
        len  -= n;
        vTaskDelay(1);
    }
}

static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
    switch (event) {
    case ESP_SPP_INIT_EVT:
        ESP_LOGI(TAG, "SPP init");
        esp_bt_dev_set_device_name(SPP_SERVER_NAME);
        esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(TAG, "SPP server started");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(TAG, "SPP client connected");
        xSemaphoreTake(spp_mutex, portMAX_DELAY);
        g_spp_handle = param->srv_open.handle;
        xSemaphoreGive(spp_mutex);
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(TAG, "SPP client disconnected");
        xSemaphoreTake(spp_mutex, portMAX_DELAY);
        g_spp_handle = 0;
        xSemaphoreGive(spp_mutex);
        break;
    case ESP_SPP_DATA_IND_EVT:
        // Bytes from phone/PC → forward to GPS UART
        if (param->data_ind.len > 0) {
            uart_write_bytes(UART_GPS, (const char*)param->data_ind.data, param->data_ind.len);
        }
        break;
    default: break;
    }
}

static void bt_spp_init() {
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE)); // Classic only
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));

    // (Optional) fixed PIN (Windows pairing friendliness)
    esp_bt_pin_code_t pin_code = {'1','2','3','4'};
    ESP_ERROR_CHECK(esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_FIXED, 4, pin_code));

    spp_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_cb));
    ESP_ERROR_CHECK(esp_spp_init(ESP_SPP_MODE_CB));
    ESP_ERROR_CHECK(esp_bt_dev_set_device_name(SPP_SERVER_NAME));
}

// ---------- RTCM queue ----------
struct RtcmMsg {
    uint16_t len;
    uint8_t  data[1200];  // RTCM3 max ~1029; padded
};
static QueueHandle_t q_rtcm = nullptr;

// CRC-24Q (poly 0x1864CFB, init 0)
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

// ---------- Tasks ----------
static SemaphoreHandle_t stats_mutex;
static uint32_t stat_rtcm_ok = 0, stat_rtcm_bytes = 0;

// GPS UART reader: mirror to SPP, capture GGA, extract RTCM3 (CRC-checked) → q_rtcm
static void gps_uart_reader_task(void*) {
    std::vector<uint8_t> rx(1024);

    // NMEA line capture
    std::string line; line.reserve(128);

    // RTCM3 extractor state
    enum class RState { SYNC, HDR1, HDR2, PAYLOAD };
    RState st = RState::SYNC;
    uint8_t hdr1 = 0, hdr2 = 0;
    int payload_left = 0; // payload + 3 CRC bytes
    RtcmMsg cur{};

    for (;;) {
        int got = uart_read_bytes(UART_GPS, rx.data(), rx.size(), pdMS_TO_TICKS(UART_READ_TO_MS));
        if (got <= 0) continue;

        // 1) Mirror raw bytes to SPP
        spp_write_bytes(rx.data(), got);

        // 2) NMEA GGA capture
        for (int i = 0; i < got; ++i) {
            unsigned char uc = rx[i];
            char c = (char)uc;
            if (c == '\n') {
                while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) line.pop_back();
                if (!line.empty() && line.front() == '$') {
                    if (line.size() >= 6 && line[3] == 'G' && line[4] == 'G' && line[5] == 'A') {
                        xSemaphoreTake(gga_mutex, portMAX_DELAY);
                        last_gga = line; // no CRLF
                        xSemaphoreGive(gga_mutex);
                    }
                }
                line.clear();
            } else if ((uc >= 0x20 && uc <= 0x7E) || c == '\r' || c == '\t') {
                if (line.size() < 255) line.push_back(c);
            } else {
                if (!line.empty()) line.clear();
            }
        }

        // 3) Extract RTCM3 and enqueue ONLY if CRC is OK
        for (int i = 0; i < got; ++i) {
            uint8_t b = rx[i];
            switch (st) {
            case RState::SYNC:
                if (b == 0xD3) { cur.len = 0; cur.data[cur.len++] = b; st = RState::HDR1; }
                break;
            case RState::HDR1:
                hdr1 = b; cur.data[cur.len++] = b; st = RState::HDR2; break;
            case RState::HDR2: {
                hdr2 = b; cur.data[cur.len++] = b;
                int len10 = ((int)(hdr1 & 0x03) << 8) | hdr2;
                if (len10 < 0 || len10 > 1023) { st = RState::SYNC; break; }
                payload_left = len10 + 3; // payload + CRC
                st = RState::PAYLOAD;
                break;
            }
            case RState::PAYLOAD: {
                cur.data[cur.len++] = b;
                if (--payload_left == 0) {
                    if (cur.len >= 6) {
                        uint32_t calc = crc24q(cur.data, cur.len - 3);
                        uint32_t gotCrc = ((uint32_t)cur.data[cur.len-3] << 16) |
                                          ((uint32_t)cur.data[cur.len-2] << 8)  |
                                           (uint32_t)cur.data[cur.len-1];
                        if (calc == gotCrc) {
                            RtcmMsg m{};
                            m.len = cur.len;
                            memcpy(m.data, cur.data, cur.len);
                            if (q_rtcm) xQueueSend(q_rtcm, &m, 0);
                            xSemaphoreTake(stats_mutex, portMAX_DELAY);
                            stat_rtcm_ok++;
                            stat_rtcm_bytes += cur.len;
                            xSemaphoreGive(stats_mutex);
                        }
                    }
                    st = RState::SYNC;
                }
                break;
            }
            } // switch
        }     // for bytes
    }         // forever
}

// NTRIP sender: connects, SOURCE handshake, sends RTCM frames from q_rtcm
static void ntrip_task(void*) {
    RtcmMsg m{};

    for (;;) {
        // Wait for Wi-Fi
        xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        int sock = connect_ntrip_socket();
        if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS)); continue; }
        if (!ntrip_v1_handshake(sock)) { shutdown(sock, SHUT_RDWR); close(sock); vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS)); continue; }

        xEventGroupSetBits(s_wifi_event_group, NTRIP_UP_BIT);
        ESP_LOGI(TAG, "NTRIP streaming…");

        for (;;) {
            // Send one RTCM message (wait up to 500 ms)
            if (xQueueReceive(q_rtcm, &m, pdMS_TO_TICKS(500)) == pdTRUE) {
                size_t off = 0;
                while (off < m.len) {
                    int chunk = std::min<int>(NET_WRITE_CHUNK, m.len - off);
                    int s = send(sock, m.data + off, chunk, 0);
                    if (s <= 0) goto reconnect;
                    off += s;
                }
            }
            // Detect remote close
            char tmp; int r = recv(sock, &tmp, 1, MSG_DONTWAIT);
            if (r == 0 || (r < 0 && errno != EWOULDBLOCK && errno != EAGAIN)) { goto reconnect; }
        }

    reconnect:
        xEventGroupClearBits(s_wifi_event_group, NTRIP_UP_BIT);
        shutdown(sock, SHUT_RDWR); close(sock);
        vTaskDelay(pdMS_TO_TICKS(RECONNECT_DELAY_MS));
    }
}

// Stats ticker (every 5 s)
static void stats_task(void*) {
    uint32_t prev_ok=0, prev_bytes=0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        xSemaphoreTake(stats_mutex, portMAX_DELAY);
        uint32_t ok = stat_rtcm_ok, bytes = stat_rtcm_bytes;
        xSemaphoreGive(stats_mutex);
        uint32_t d_ok = ok - prev_ok, d_b = bytes - prev_bytes;
        prev_ok = ok; prev_bytes = bytes;
        ESP_LOGI(TAG, "RTCM OK frames: %u in 5s (%.1f/s), bytes: %u (~%u bps)",
                 d_ok, d_ok / 5.0f, d_b, (unsigned)(d_b*8/5));
    }
}

// ---------- app_main ----------
extern "C" void app_main(void) {
    s_wifi_event_group = xEventGroupCreate();
    gga_mutex   = xSemaphoreCreateMutex();
    stats_mutex = xSemaphoreCreateMutex();

    wifi_init();
    uart_init_gps();
    bt_spp_init();

    // Queues
    q_rtcm = xQueueCreate(32, sizeof(RtcmMsg));

    // Tasks
    xTaskCreatePinnedToCore(&gps_uart_reader_task, "gps_uart_reader", 4096, nullptr, 7, nullptr, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&ntrip_task,          "ntrip_task",      4096, nullptr, 6, nullptr, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&stats_task,          "stats_task",      2048, nullptr, 2, nullptr, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&led_task,            "led_task",        2048, nullptr, 1, nullptr, tskNO_AFFINITY);
}