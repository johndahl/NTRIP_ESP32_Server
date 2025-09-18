| Supported Targets | ESP32 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- |

# ESP32 based NTRIP server
The ESP is connecting to a known WiFi at start up and wait for RTCM data on the serial port. If valid data is received, it connect to a mount point at the NTRIP caster and start to transmit. 


## How to Use Example

You need to rename `wifi_credentials_template.h` and `ntrip_credentials_template.h` to `wifi_credentials.h` and `ntrip_credentials.h`, respectively, and add your credentials.

Use `idf.py build` to compile the project. 

### Hardware Required
This project has been tested on a ESP32 wrover board. You need a GPS unit that can recieve raw GPS data in RTCM format transmit on a 3.3 v uart port. Additionally, you need a mount point at a NTRIP caster. This project is tested on a self-hosted caster on a Synology NAS, but any hosted service online should work.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

