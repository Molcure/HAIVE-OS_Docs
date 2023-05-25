# Implementing A New Device

This section takes you through the steps it would require to implement a new device in the HAIVE sytem using HAIVE OS from scratch.

## Developing The Client

For now, HAIVE OS has a client library for the ESP embedded platform called [hos_client](https://github.com/Molcure/HAIVE-OS/tree/main/hos_clients/Arduino/libraries/hos_client).

TODO: Developing a similar library for Python support based on [roslibpy](https://roslibpy.readthedocs.io/en/latest/) should be no big task.

You can follow the install instructions and start developing the firmware code. A good start is the examples which you can find under `File/Examples/hos_client` in your Arduino IDE. Let's take a look at the `ESP32` example code:

```C++
// ESP32Example.pde
//
// Starter code for connecting ESP32 to HAIVE OS
//
// hos_client requires the following dependencies to be installed:
//  - bblanchon/ArduinoJson@^6.19.4
#include <hos_client.h>


#define DEBUG_PRINT true

//**************************************** DATA DEFINITIONS ****************************************//
/*
 * Serial baud rates
 */ 
#define SERIAL_BAUD  74880
#define SLAVE_BAUD     9600

/*
 * HAIVE OS Client setup
 */
const char* wlan_ssid = "Molcure_Local";
const char* wlan_password = "makesense";
const char* hos_address = "192.168.2.240";
const int hos_port = 9091;

DeviceClient hos_client(wlan_ssid, wlan_password, hos_address, hos_port);

// HAIVE OS command state variables
bool executing_command = false;
uint32_t command_id;

/*
 * L1 responses
 */
#define L1_ACK_FROM_ASTAR 0x32  // WARNING: ASCII symbol 'SP' (Space); This was transmitted from CTNv4.1_Astar
#define L1_DONE           0x30  // WARNING: ASCII symbol '0'
#define L1_PROCESSING     0x31  // WARNING: ASCII symbol '1'
#define L1_ERROR          0x41  // WARNING: ASCII symbol 'A'

char l1_status;
int error_code;

bool new_status_available = false;

/*
 * Serial data
 */
#define DATA_LENGTH  14

char CMD_KEY[DATA_LENGTH/2] = { 'N', };
float CMD_VAL[DATA_LENGTH/2] = { 0.0, };

bool new_data_available = false;

//**************************************** FUNCTION DEFINITIONS ****************************************//

void print_cmd_data() {
  for (unsigned int i = 0; i < DATA_LENGTH; ++i) {
    if (i % 2 == 0){
      Serial.print(CMD_KEY[i/2]);
    } else {
      Serial.print(CMD_VAL[(i-1)/2]);
    }
  }
  Serial.print("\n");
}

void process_l1_status(uint32_t command_id)
{
  new_status_available = false;

  if (l1_status == L1_DONE) {
    hos_client.report_cmd_result(command_id, true);
    hos_client.set_device_state(IDLE);

    executing_command = false;
  }

  else if (l1_status == L1_ERROR) {
    hos_client.report_cmd_result(command_id, false);
    hos_client.set_device_state(ERROR);

    executing_command = false;
  }
}

void serial_getter(Stream &serial_port)
{
  if (!serial_port.available())
    return;

  char c = serial_port.peek();

  // Handle L1 status responses
  if (c == L1_DONE || c == L1_PROCESSING || c == L1_ERROR || c == L1_ACK_FROM_ASTAR)
  {
    l1_status = serial_port.read();
    new_status_available = true;

    switch (l1_status)
    {
      case L1_DONE:
#if DEBUG_PRINT
        Serial.println("Recveived L1 status: DONE");
#endif
        break;
      case L1_PROCESSING:
#if DEBUG_PRINT
        Serial.println("Recveived L1 status: PROCESSING");
#endif
        break;
      case L1_ERROR:
        error_code = serial_port.parseInt();
#if DEBUG_PRINT
        Serial.println("Recveived L1 status: ERROR");
#endif
        break;
      case L1_ACK_FROM_ASTAR:
#if DEBUG_PRINT
        Serial.println("Recveived L1 status: ACK_FROM_ASTAR");
#endif
        break;
    }

    return;
  }

  // Otherwise, we assume that we are receiving serialized data from the serial port

  // Reset key value store
  for (unsigned int i = 0; i < DATA_LENGTH; ++i) {
    if (i % 2 == 0) {
      CMD_KEY[i/2] = 'N';
    } else {
      CMD_VAL[(i-1)/2] = 0.0f;
    }
  }

#if DEBUG_PRINT
  Serial.println("SERIAL_GETTER::INFO: Reading serial buffer...");
#endif

  // Read data
  char CR = '\r';
  char LF = '\n';
  for (unsigned int i = 0; i < DATA_LENGTH; ++i) {

    // Exit when end_marker is detected
    if (serial_port.peek() == CR || serial_port.peek() == LF) {
#if DEBUG_PRINT
      Serial.println("SERIAL_GETTER::INFO: Exit due to end_marker.");
#endif
      while (serial_port.peek() == CR || serial_port.peek() == LF) serial_port.read();
      new_data_available = true;
      return;
    }

    // Store the key in CMD_KEY
    if (i % 2 == 0) {

      // Break if no data is available
      if (serial_port.peek() == -1) {
#if DEBUG_PRINT
        Serial.println("SERIAL_GETTER::WARNING: Exit due to empty buffer.");
#endif
        new_data_available = true;
        return;
      }

      CMD_KEY[i/2] = serial_port.read();

      // Return if character is not alphabet
      if (!isAlpha(CMD_KEY[i/2])) {
#if DEBUG_PRINT
        Serial.print("SERIAL_GETTER::WARNING: Exit due to non-alphabet ASCII value ");
        Serial.print((int)CMD_KEY[i/2]);
        Serial.println(".");
#endif
        CMD_KEY[i/2] = 'N';
        new_data_available = true;
        return;
      }
    }

      // Store the value in CMD_VALUE
    else {
      CMD_VAL[(i-1)/2] = serial_port.parseFloat();
    }
  }

#if DEBUG_PRINT
  Serial.println("SERIAL_GETTER::ERROR: Maximum data length exeeded without reading end_marker.");
#endif
}

void process_serial(Stream &serial_port)
{
  if (!serial_port.available())
    return;

  delay(10);

#if DEBUG_PRINT
  Serial.print("Bytes available: ");
  Serial.println(serial_port.available());
#endif

  serial_getter(serial_port);
}

void process_messages()
{
  // Process commands received from HAIVE OS
  if (hos_client.command_available()) {
#if DEBUG_PRINT
    Serial.println("Sending HAIVE OS command to slave.");
#endif

    command_id = hos_client.send_command(Serial1);
    executing_command = true;
  }

  // Process messages from MCU
  process_serial(Serial1);

  // Process execution status from slave MCU
  if (executing_command && new_status_available)
    process_l1_status(command_id);

  // Process serialized data from slave MCU
  if (new_data_available) {
    new_data_available = false;

    print_cmd_data();

    // Handle data here
  }
}

//**************************************** MAIN PROGRAM ****************************************//

void setup(){
  Serial.begin(SERIAL_BAUD);
  Serial1.begin(SLAVE_BAUD);

//  hos_client.setup_command_api();  // Setup command api client
//  hos_client.setup_stream("insert_stream_name");  // Setup a stream by name

  hos_client.connect();
  
  Serial.println("Ready");
}


void loop(){
  // Timing variables
  static unsigned long previous_time_ms = 0;
  long current_time_ms = millis();

  // Update HAIVE OS client connection
  hos_client.loop();
  if (!hos_client.is_connected())
    return;

  // Process HAIVE OS and slave messages
  process_messages();

  previous_time_ms = current_time_ms;
}
```

First we need to uncomment line 246:

```C++

  // ...

  hos_client.setup_command_api();

  // ...

```

This will allow the client to receive [DeviceCommand](https://molcure.github.io/HAIVE-OS_Pages/#/hos_interfaces/?id=devicecommandsrv) requests and will trigger a condition in the `process_messages` function. The current implementation automatically sends the received command to a slave MCU. Let's change that code a little and print the received message to our Serial monitor instead:

```C++

// ...

void process_messages()
{
  // Process commands received from HAIVE OS
  if (hos_client.command_available()) {
#if DEBUG_PRINT
    Serial.println("Sending HAIVE OS command to slave.");
#endif

    // command_id = hos_client.send_command(Serial1);
    // executing_command = true;
    command_id = hos_client.send_command(Serial);

    hos_client.report_cmd_result(command_id, true);
    hos_client.set_device_state(IDLE);
  }

  // Process messages from MCU
  process_serial(Serial1);

  // Process execution status from slave MCU
  if (executing_command && new_status_available)
    process_l1_status(command_id);

  // Process serialized data from slave MCU
  if (new_data_available) {
    new_data_available = false;

    print_cmd_data();

    // Handle data here
  }
}

// ...

```

We also added two function calls to `hos_client` methods `report_cmd_result` and `set_device_state`. `report_cmd_result` signals to HAIVE OS that the command has finished execution and that it was successful. Furthermore we set the device state mack to its initial `IDLE` state so that the client can receive further requests. 

The next step will be that we define our devices functionality in our HAIVE OS device database.

## Updating The HAIVE OS Database

Let's open the [HAIVE OS device database](https://airtable.com/appR9FYsP809nu76u?ao=cmVjZW50).
We first want to add a new device in the `devices` table called `TestDevice`:



We set our `ros_target` to be `ESP32`. We also marked the device api to be serialized, so that we receive an encoded string of our function on our embedded system. Next, let's update the `shinkawasaki-fleet` table and give our new device a device identifier:



Next to our device identifier we also assigned a `uid` of `999` which is a placeholder, and will be updated once we established a first connection to HAIVE OS. Furthermore we selected our `device_type` `TestDevice` which we just created. Let's now define some functionality in the `apis` table:


The function we added is called `my_test_function`. We added two function arguments and a result. For possible argument types, please check the [HAIVE OS device database documentation](https://molcure.github.io/HAIVE-OS_Pages/#/hos_device_layer/?id=haive-os-device-database). We also added a function description and selected our `device_type` `TestDevice`.

Lastly, since we wish to serialize our function call into a string, we have to define a serializer in the `api_serializers` table:



Don't forget to also select our function `my_test_function` for the `function_name`. The way that the serializer works is that once HAIVE OS receives a request for a device API call, it serializes the argument data by substituting each `$` with an argument from the `arguments` list described in `apis`. This implies that `len(arguments) == #$` must hold.

Now we are ready to make a request to our new functionality, but first we need to startup HAIVE OS.

## Starting HAIVE OS and making a `DeviceAPI` request

...
