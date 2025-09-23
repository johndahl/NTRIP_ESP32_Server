idf_component_register(
  SRCS
  main.cpp
  web_server.cpp
  uart_comm.cpp
  gps_base_getter.cpp
  INCLUDE_DIRS "."
  REQUIRES
    esp_event
    esp_netif
    nvs_flash
    esp_wifi
    driver
    lwip
    bt
    esp_http_server
    json
    esp_common
    mdns
  EMBED_TXTFILES "index.html"
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
set_source_files_properties(ntrip_source.cpp PROPERTIES
  COMPILE_FLAGS "-O0 -g3 -fno-inline -fno-ipa-sra -fno-ipa-cp -fno-ipa-cp-clone -fvar-tracking -fvar-tracking-assignments"
)
