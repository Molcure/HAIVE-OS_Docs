# Streaming Device Data

This section goes shows an example of how to stream data from a `M5StickCPlus` to HAIVE OS.

## Firmware

This example assumes that you have another MCU connected to the M5 via UART that is sending data values.

```C++
// ESP32Example.pde
//
// Starter code for connecting ESP32 to HAIVE OS
//
// hos_client requires the following dependencies to be installed:
//  - bblanchon/ArduinoJson@^6.19.4

#include <M5StickCPlus.h>
#include <hos_client.h>


#define DEBUG_PRINT true


/*
 * HAIVE OS Client setup
 */
const char* wlan_ssid = "Molcure_Local";
const char* wlan_password = "makesense";
const char* hos_address = "192.168.2.240";
const int hos_port = 9091;

DeviceClient hos_client(wlan_ssid, wlan_password, hos_address, hos_port);


void send_data(float value) {
  String msg = "S0V" + String(value);
  hos_client.send_stream_value(msg.c_str());
}

void setup() {
  // Initialize M5StickC Plus
  M5.begin();
  // Initialisetup_streamze Serial Monitor
  Serial.begin(115200);
  // Initialize Serial2
  Serial2.begin(115200, SERIAL_8N1, 32, 33);

  hos_client.setup_stream("my_stream");  // Setup a stream by name
  hos_client.connect();

  // Initialize LCD
  M5.Lcd.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.println("Waiting for data...");
}

void loop() {
  // Update HAIVE OS client connection
  hos_client.loop();
  if (!hos_client.is_connected())
    return;

  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    float value = data.toFloat();

    send_data(value);

    // Display the value on the LCD
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.print("Value: ");
    M5.Lcd.println(value);

    // Send the value to the Serial Monitor
    Serial.print("Received value: ");
    Serial.println(value);
  }
}
```

## Checking The Stream Data

Follow these steps in order.

1. Turn off `M5StickCPlus`
2. Start HAIVE OS
```shell
ros2 launch hos_run hos_run.launch.py 
```
3. Turn on `M5StickCPlus`
4. Open a new shell instance on the HAIVE OS machine and check your topics
```shell
ros2 topic list
```

You should see a list similar to this:

```
/client_count
/connected_clients
/haive_os/device/api_call_result
/haive_os/device/command_result
/haive_os/device/connected
/haive_os/device/disconnected
/haive_os/device/ping
/haive_os/device/reconnected
/haive_os/device/timeout
/haive_os/device/uid11283384/stream/my_stream
/haive_os/load_protocol
/haive_os/stop_protocol
/parameter_events
/rosout
```

We want to be listening to the topic `/haive_os/device/uid11283384/stream/my_stream` which can be achieved by running:

```shell
ros2 topic echo /haive_os/device/uid11283384/stream/my_stream
```

 Your console should show outputs similar to this:

```
uid: 11283384
data: S0V0.00
---
```
