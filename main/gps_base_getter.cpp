#include "gps_base_getter.h"
#include "uart_comm.h"
#include "web_server.h"
#include "esp_log.h"

static const char* TAG = "gps_base_getter";
static TaskHandle_t s_task = nullptr;
static uart_port_t  s_uart = UART_NUM_1;
static uint32_t     s_period_ms = 1000;

// SkyTraq: request payload ID (see your earlier code/comments)
// You were sending 0x23 and expecting a reply beginning with 0xA5.
// Keep that unless your module uses a different pair.
static constexpr uint8_t REQ_ID         = 0x23;
static constexpr uint8_t REPLY_FIRST_ID = 0x8B;

static bool query_base_once(double* lat, double* lon, double* h) {
    // Send payload-only (uart_comm will frame it: A0 A1 ... CS 0D 0A)
    uint8_t req[1] = { REQ_ID };

    uint8_t  rx[64];
    uint16_t rxlen = sizeof(rx);
    // Modest timeout to avoid blocking the UART pipeline
    if (!uart_comm_sky_send_and_wait(req, sizeof(req), REPLY_FIRST_ID,
                                     rx, &rxlen, pdMS_TO_TICKS(700))) {
        return false;
    }
    // Expected payload layout (big-endian):
    // [0]=0x8B, [1]=mode, [10..17]=lat(double), [18..25]=lon(double), [26..29]=h(float)
    if (rxlen < 1 + 1 + 8 + 8 + 4) return false;

    union { uint64_t u; double d; } u64;
    union { uint32_t u; float  f; } u32;

    u64.u = 0;
    for (int j = 0; j < 8; ++j) u64.u = (u64.u << 8) | rx[10 + j];
    *lat = u64.d;

    u64.u = 0;
    for (int j = 0; j < 8; ++j) u64.u = (u64.u << 8) | rx[18 + j];
    *lon = u64.d;

    u32.u = 0;
    for (int j = 0; j < 4; ++j) u32.u = (u32.u << 8) | rx[26 + j];
    *h = u32.f;

    return true;
}

static void task_fn(void*) {
    ESP_LOGI(TAG, "started (period=%u ms, uart=%d)", (unsigned)s_period_ms, (int)s_uart);
    for (;;) {
        double lat = 0, lon = 0, h = 0;
        bool ok = query_base_once(&lat, &lon, &h);
        if (ok) {
            web_report_position(lat, lon, h, true);
        } else {
            // Keep last known values, but flag invalid so UI can show status
            web_report_position(0, 0, 0, false);
        }
        vTaskDelay(pdMS_TO_TICKS(s_period_ms));
    }
}

void gps_base_getter_start(uart_port_t cfg_uart, uint32_t period_ms) {
    if (s_task) return; // already running
    s_uart = cfg_uart;
    s_period_ms = period_ms ? period_ms : 1000;
    xTaskCreatePinnedToCore(task_fn, "gps_base_getter", 3072, nullptr, 5, &s_task, tskNO_AFFINITY);
}
