# HAIVE OS Client Library for Arduino

This library allows connecting the `ESP8266` and `ESP32` microcontrollers to HAIVE OS, thus bringing HAIVE OS to embedded platforms. The core communication is handled by an implementation of the rosbridge protocol [specified here](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md).

## Installation

Install the library by copying the [hos_client](https://github.com/Molcure/HAIVE-OS/tree/main/hos_clients/Arduino/libraries/hos_client) folder to your Arduino libraries folder. See the [Arduino documentation](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries#manual-installation) for further details.

### Dependencies

The `hos_client` library also has some other dependencies that need to be installed depending on the development platform:

- `ESP8266`:
  - [links2004/WebSockets@^2.3.7](https://github.com/Links2004/arduinoWebSockets)
  - [bblanchon/ArduinoJson@^6.19.4](https://github.com/bblanchon/ArduinoJson)
- `ESP32`:
  - [bblanchon/ArduinoJson@^6.19.4](https://github.com/bblanchon/ArduinoJson)

These dependencies can be installed using the [Arduino IDE library manager](https://docs.arduino.cc/software/ide-v1/tutorials/installing-libraries).

## Starter Code

```C++
#include <hos_client.h>

#define DEBUG_PRINT true

const char* wlan_ssid = "WLAN_SSID";
const char* wlan_password = "WLAN_PW";
const char* hos_address = "192.168.2.240";
const int hos_port = 9091;

DeviceClient hos_client(wlan_ssid, wlan_password, hos_address, hos_port);

void setup(){
  hos_client.connect();
}

void loop(){
  hos_client.loop();

  if (!hos_client.is_connected())
    return;
}

```
