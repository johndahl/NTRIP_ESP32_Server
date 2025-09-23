#include "web_server.h"
#include <string>
#include <atomic>
#include <cmath>
#include <cstring>
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "driver/uart.h"
#include "esp_check.h"
#include "uart_comm.h"

// Embedded page from CMake target_add_binary_data
//extern const char _binary_index_html_start[] asm("_binary_index_html_start");
//extern const char _binary_index_html_end[]   asm("_binary_index_html_end");

extern const unsigned char _binary_index_html_start[];
extern const unsigned char _binary_index_html_end[];

static const char* TAG = "web";
static uart_port_t s_cfg_uart = UART_NUM_1; // default; overwritten by web_start
static std::atomic<double> s_lat{NAN}, s_lon{NAN}, s_h{NAN};
static std::atomic<bool>   s_valid{false};

// ---------- NVS helpers ----------
static esp_err_t nvs_save_base(double lat, double lon, double h) {
    nvs_handle_t hnd;
    ESP_RETURN_ON_ERROR(nvs_open("cfg", NVS_READWRITE, &hnd), TAG, "nvs_open");
    esp_err_t err1 = nvs_set_blob(hnd, "base", &lat, sizeof(lat));
    esp_err_t err2 = nvs_set_blob(hnd, "base2", &lon, sizeof(lon));
    esp_err_t err3 = nvs_set_blob(hnd, "base3", &h,   sizeof(h));
    esp_err_t err4 = nvs_commit(hnd);
    nvs_close(hnd);
    return (err1||err2||err3||err4) ? ESP_FAIL : ESP_OK;
}
static bool nvs_load_base(double* lat, double* lon, double* h) {
    nvs_handle_t hnd;
    if (nvs_open("cfg", NVS_READONLY, &hnd) != ESP_OK) return false;
    size_t sz = sizeof(double);
    if (nvs_get_blob(hnd, "base", lat, &sz) != ESP_OK) { nvs_close(hnd); return false; }
    sz = sizeof(double);
    if (nvs_get_blob(hnd, "base2", lon, &sz) != ESP_OK){ nvs_close(hnd); return false; }
    sz = sizeof(double);
    if (nvs_get_blob(hnd, "base3", h, &sz) != ESP_OK)  { nvs_close(hnd); return false; }
    nvs_close(hnd);
    return true;
}

// ---------- SkyTraq binary pack/send (Message 0x22 = Set Base Position) ----------
// Protocol: <A0 A1> <PL_hi PL_lo> <ID> <payload> <CS> <0D 0A>
// CS = XOR over all payload bytes (ID+payload), big-endian multi-byte fields.
// For 0x22: mode(1), survey_len(4), stddev_m(4), lat(double), lon(double), h(float), attr(1)
static inline void be32(uint8_t* p, uint32_t v){ p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v; }
static inline void be64(uint8_t* p, uint64_t v){ for(int i=0;i<8;i++) p[i] = (v>>(56-8*i)) & 0xFF; }

static bool skytraq_set_base_static(uart_port_t uart, double lat_deg, double lon_deg, float h_m, bool save_flash){
    uint8_t pay[1 + 1 + 4 + 4 + 8 + 8 + 4 + 1] = {0};
    size_t off = 0;
    pay[off++] = 0x22;                  // ID
    pay[off++] = 0x02;                  // Static
    be32(&pay[off], 0x000007D0); off+=4;// Survey length placeholder
    be32(&pay[off], 0x0000001E); off+=4;// Std dev placeholder
    union { double d; uint64_t u; } u64;
    u64.d = lat_deg; be64(&pay[off], u64.u); off+=8;
    u64.d = lon_deg; be64(&pay[off], u64.u); off+=8;
    union { float f; uint32_t u; } u32;
    u32.f = h_m; be32(&pay[off], u32.u); off+=4;
    pay[off++] = save_flash ? 0x01 : 0x00;

    uint8_t rx[32]; uint16_t rxlen = sizeof(rx);
    bool ok = uart_comm_sky_send_and_wait(pay, off,
                                          /*expect_reply_id=*/0x83, // ACK
                                          rx, &rxlen,
                                          pdMS_TO_TICKS(700));
    if (ok) {
        ESP_LOGI(TAG, "SkyTraq Set Base Position OK");
        return true;
    }
    ESP_LOGW(TAG, "No ACK from SkyTraq after 0x22");
    return false;
}

// ---------- Switch SkyTraq to binary protocol ----------
bool skytraq_switch_to_binary(uart_port_t) {
    uint8_t pay[3] = { 0x09, 0x01, 0x00 };      // ID + args
    uint8_t rx[8]; uint16_t rxlen = sizeof(rx);
    bool ok = uart_comm_sky_send_and_wait(pay, sizeof(pay), 0x83, rx, &rxlen, pdMS_TO_TICKS(500));
    if (!ok) ESP_LOGW(TAG, "No ACK to Switch-to-Binary");
    return ok;
}

// ---------- Query SkyTraq base position (Message 0x25)
// Returns true on success, and fills lat, lon, h (height in meters)
static bool skytraq_query_base_position(uart_port_t, double* lat, double* lon, double* h) {
    uint8_t pay[1] = { 0x23 };                 // request ID
    uint8_t rx[64]; uint16_t rxlen = sizeof(rx);

    // Expect reply payload starting with 0xA5
    if (!uart_comm_sky_send_and_wait(pay, sizeof(pay), 0xA5, rx, &rxlen, pdMS_TO_TICKS(700)))
        return false;

    // [0]=0xA5, [1]=mode, [2..9]=lat, [10..17]=lon, [18..21]=h(float)
    if (rxlen < 1 + 1 + 8 + 8 + 4) return false;

    union { uint64_t u; double d; } u64;
    union { uint32_t u; float  f; } u32;

    u64.u = 0;
    for (int j = 0; j < 8; ++j) {
        u64.u = (u64.u << 8) | rx[2 + j];
    }
    *lat = u64.d;

    u64.u = 0;
    for (int j = 0; j < 8; ++j) {
        u64.u = (u64.u << 8) | rx[10 + j];
    }
    *lon = u64.d;

    u32.u = 0;
    for (int j = 0; j < 4; ++j) {
        u32.u = (u32.u << 8) | rx[18 + j];
    }
    *h = u32.f;

    return true;
}



// ---------- HTTP handlers ----------
static esp_err_t handle_root(httpd_req_t* req){
    httpd_resp_set_type(req, "text/html");

    const unsigned char* start = _binary_index_html_start;
    const unsigned char* end   = _binary_index_html_end;
    size_t len = (size_t)(end - start);

    ESP_LOGI("web", "index.html len=%u", (unsigned)len);

    if (len == 0) {
        return httpd_resp_sendstr(req,
            "<!doctype html><meta charset=utf-8>"
            "<h3>index.html not embedded (len=0)</h3>"
            "<p>Check EMBED_TXTFILES and symbol names.</p>");
    }
    return httpd_resp_send(req, (const char*)start, len);
}

static esp_err_t handle_get_gnss(httpd_req_t* req){
    httpd_resp_set_type(req, "application/json");

    cJSON* j = cJSON_CreateObject();
    if (s_valid.load()){
        cJSON_AddBoolToObject(j, "valid", true);
        cJSON_AddNumberToObject(j, "lat", s_lat.load());
        cJSON_AddNumberToObject(j, "lon", s_lon.load());
        cJSON_AddNumberToObject(j, "alt", s_h.load());
    } else {
        cJSON_AddBoolToObject(j, "valid", false);
    }

    char* out = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);
    esp_err_t r = httpd_resp_sendstr(req, out);
    free(out);
    return r;
}

static esp_err_t handle_get_base(httpd_req_t* req){
    httpd_resp_set_type(req, "application/json");
    double lat, lon, h;
    cJSON* j = cJSON_CreateObject();
    if (nvs_load_base(&lat, &lon, &h)){
        cJSON_AddBoolToObject(j, "present", true);
        cJSON_AddNumberToObject(j, "lat", lat);
        cJSON_AddNumberToObject(j, "lon", lon);
        cJSON_AddNumberToObject(j, "alt", h);
    } else {
        cJSON_AddBoolToObject(j, "present", false);
    }
    char* out = cJSON_PrintUnformatted(j);
    cJSON_Delete(j);
    esp_err_t r = httpd_resp_sendstr(req, out);
    free(out);
    return r;
}

static esp_err_t handle_post_base(httpd_req_t* req){
    // Read body
    char buf[256]; int len = httpd_req_recv(req, buf, sizeof(buf)-1);
    if (len <= 0) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no body");
    buf[len] = 0;

    // Parse JSON {lat, lon, alt, saveToFlash: true/false}
    cJSON* j = cJSON_Parse(buf);
    if (!j) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
    cJSON* jl  = cJSON_GetObjectItem(j, "lat");
    cJSON* jlo = cJSON_GetObjectItem(j, "lon");
    cJSON* ja  = cJSON_GetObjectItem(j, "alt");
    cJSON* js  = cJSON_GetObjectItem(j, "saveToFlash");
    bool save  = js && cJSON_IsBool(js) ? cJSON_IsTrue(js) : true;
    if (!cJSON_IsNumber(jl) || !cJSON_IsNumber(jlo) || !cJSON_IsNumber(ja)){
        cJSON_Delete(j);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing lat/lon/alt");
    }
    double lat = jl->valuedouble, lon = jlo->valuedouble, alt = ja->valuedouble;
    cJSON_Delete(j);

    // Save in NVS
    if (nvs_save_base(lat, lon, alt) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "nvs fail");

    // Push to receiver via SkyTraq 0x22 (Static)
    bool ok = skytraq_set_base_static(s_cfg_uart, lat, lon, (float)alt, save);
    if (!ok) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "uart write fail or no ack");

    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t handle_ping(httpd_req_t* req) {
    return httpd_resp_sendstr(req, "ok");
}

// ---------- Public API ----------
void web_report_position(double lat, double lon, double h, bool valid){
    s_lat.store(lat); s_lon.store(lon); s_h.store(h); s_valid.store(valid);
}

void web_start(uart_port_t cfg_uart){
    s_cfg_uart = cfg_uart;

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;                  // change if needed
    cfg.stack_size = 8192;          // bump for cJSON/NVS
    cfg.lru_purge_enable = true;    // drop oldest session if out of sockets
    cfg.max_open_sockets = 4;       // be explicit
    cfg.recv_wait_timeout = 5;
    cfg.send_wait_timeout = 5;
    cfg.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t srv = nullptr;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));

    httpd_uri_t root = { .uri="/", .method=HTTP_GET, .handler=handle_root, .user_ctx=nullptr };
    httpd_uri_t gnss = { .uri="/api/gnss", .method=HTTP_GET, .handler=handle_get_gnss, .user_ctx=nullptr };
    httpd_uri_t getb = { .uri="/api/base", .method=HTTP_GET, .handler=handle_get_base, .user_ctx=nullptr };
    httpd_uri_t postb= { .uri="/api/base", .method=HTTP_POST,.handler=handle_post_base,.user_ctx=nullptr };
    httpd_uri_t ping = { .uri="/ping", .method=HTTP_GET, .handler=handle_ping, .user_ctx=nullptr };
    httpd_register_uri_handler(srv, &root);
    httpd_register_uri_handler(srv, &gnss);
    httpd_register_uri_handler(srv, &getb);
    httpd_register_uri_handler(srv, &postb);
    httpd_register_uri_handler(srv, &ping);

    ESP_LOGI(TAG, "Web UI on http://<device-ip>/");
}
