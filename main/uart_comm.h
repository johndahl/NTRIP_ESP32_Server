#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/queue.h"
#include "driver/uart.h"

// ---------- Protocol tags ----------
enum class UartProto : uint8_t {
    SKYTRAQ = 0,  // A0 A1 ... CS 0D 0A (ID = first payload byte)
    RTCM3   = 1,  // 0xD3 LEN(10-bit) payload CRC24Q
    NMEA    = 2   // $...*CS\r\n
};

// ---------- Frames ----------
struct SkytraqFrame {             // payload includes ID as first byte
    uint16_t len;
    uint8_t  data[300];
};
struct RtcmFrame {
    uint16_t len;
    uint8_t  data[1200];          // ~1kB max
};
struct NmeaLine {
    uint16_t len;
    char     txt[256];            // 255 + NUL
};

// ---------- API ----------
void uart_comm_start(uart_port_t uart, int baud,
                     int tx_pin, int rx_pin,
                     size_t rx_buf_bytes = 8*1024);

// queues you can read from (created inside start)
QueueHandle_t uart_comm_get_rtcm_queue();  // continuous RTCM stream
QueueHandle_t uart_comm_get_nmea_queue();  // optional NMEA lines

StreamBufferHandle_t uart_comm_get_raw_stream(); // raw byte stream (all data as-is)

// Send a SkyTraq command and wait for a reply with a specific ID.
// - tx_payload: bytes to put in payload (first byte MUST be command ID);
//               CS is computed for you (XOR of payload bytes).
// - expect_reply_id: first byte of reply payload you expect (e.g., ACK=0x83 or specific 0xA5 etc.)
// - rx_out: where reply payload will be copied (incl. ID at [0])
// Returns true on matched reply before timeout.
bool uart_comm_sky_send_and_wait(const uint8_t* tx_payload, uint16_t tx_len,
                                 uint8_t expect_reply_id,
                                 uint8_t* rx_out, uint16_t* rx_len,
                                 TickType_t timeout);

// Optional: broadcast all future SkyTraq frames that don’t match waiters
// to a generic unsolicited queue (NULL to disable). Default disabled.
void uart_comm_set_unsolicited_sky_queue(QueueHandle_t q);
