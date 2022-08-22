# esp32-homekit-dht22
ESP32 Homekit DHT22 sensor based on Espressif's ESP-IDF and ESP-Homekit-SDK

![Hardware image](https://github.com/ckshum88/esp32-homekit-dht22/blob/main/image/esp32_dht22.jpg?raw=true)

## Compiling

### Prerequisites

- [ESP-IDF](https://github.com/espressif/esp-idf) (Tested on ESP-IDF v4.3)
- [ESP-Homekit-SDK](https://github.com/espressif/esp-homekit-sdk)

### Building

Install Espressif's ESP-IDF and ESP-Homekit-SDK as described in the official website.

Set the environment variable ```HOMEKIT_PATH``` to the base path of the Homekit SDK.

Then, run ```idf.py flash monitor``` in the root folder.

## Running

Provision the ESP32 module onto your WiFi network according to the [instructions here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/provisioning/provisioning.html#provisioning-tools). After that, add the accessory to the Home app. The default setup code is ```111 22 333```.

## Author

Alfred Shum - [ckshum88](https://github.com/ckshum88)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
