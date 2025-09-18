idf_component_register(
  SRCS
  main.cpp
  INCLUDE_DIRS "."
  REQUIRES
    esp_event
    esp_netif
    nvs_flash
    esp_wifi
    driver
    lwip
    bt
)
set_property(TARGET ${COMPONENT_LIB} PROPERTY CXX_STANDARD 17)
set_source_files_properties(ntrip_source.cpp PROPERTIES
  COMPILE_FLAGS "-O0 -g3 -fno-inline -fno-ipa-sra -fno-ipa-cp -fno-ipa-cp-clone -fvar-tracking -fvar-tracking-assignments"
)
