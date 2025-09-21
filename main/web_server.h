#pragma once
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start the web server (call once after Wi-Fi is up)
void web_start(uart_port_t gnss_cfg_uart);

// Report the *current* rover/base position (call from your NMEA handler)
void web_report_position(double lat_deg, double lon_deg, double h_m, bool valid);

// Optional: fetch what’s stored in NVS (base)
bool web_get_base(double* lat, double* lon, double* h);

#ifdef __cplusplus
}
#endif