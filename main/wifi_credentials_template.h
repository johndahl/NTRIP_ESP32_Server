
#pragma once
// ===================== USER CONFIG =====================
// Wi-Fi fallback list (edit to taste; order = priority)
struct WifiCred { const char* ssid; const char* pass; };
static constexpr WifiCred WIFI_LIST[] = {
    { "YourSSID1", "YourPassword1" },
    { "YourSSID2", "YourPassword2" }
};
static constexpr int WIFI_LIST_LEN = sizeof(WIFI_LIST)/sizeof(WIFI_LIST[0]);