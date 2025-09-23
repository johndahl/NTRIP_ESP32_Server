#pragma once
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Start the periodic getter task that queries the GNSS base position
// and updates web_report_position(lat, lon, h, valid).
// - cfg_uart: the UART used for SkyTraq control (same you pass to web_start)
// - period_ms: how often to poll (e.g. 1000)
void gps_base_getter_start(uart_port_t cfg_uart, uint32_t period_ms);
