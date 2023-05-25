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
//#if DEBUG_PRINT
//    Serial.println("Sending HAIVE OS command to slave.");
//#endif

    // command_id = hos_client.send_command(Serial1);
    // executing_command = true;

    String cmd = hos_client.get_command();
    Serial.println(cmd);

    hos_client.report_cmd_result(command_id, true);
    hos_client.set_device_state(IDLE);
  }

  // ...
}

// ...

```

We also added two function calls to `hos_client` methods `report_cmd_result` and `set_device_state`. `report_cmd_result` signals to HAIVE OS that the command has finished execution and that it was successful. Furthermore we set the device state mack to its initial `IDLE` state so that the client can receive further requests. 

Now all we have to do is flash our firmware to an actual `ESP32`.

The next step will be that we define our devices functionality in our HAIVE OS device database.

## Updating The HAIVE OS Database

Let's open the [HAIVE OS device database](https://airtable.com/appR9FYsP809nu76u?ao=cmVjZW50).
We first want to add a new device in the `devices` table called `TestDevice`:

![Adding New Device](img/new_device1.png 'Adding New Device')

We set our `ros_target` to be `ESP32`, added a custom `device_role` called `MyRole` and a description of our device. We also marked the device api to be serialized, so that we receive an encoded string of our function on our embedded system. Next, let's update the `shinkawasaki-fleet` table and give our new device a device identifier:

![Adding To Fleet](img/new_device2.png 'Adding To Fleet')

Next to our device identifier we also assigned a `uid` of `999` which is a placeholder, and will be updated once we established a first connection to HAIVE OS. Furthermore we selected our `device_type` `TestDevice` which we just created. Let's now define some functionality in the `apis` table:


![Define API](img/new_device3.png 'Define API')


The function we added is called `my_test_function`. We added two function arguments and a result. For possible argument types, please check the [HAIVE OS device database documentation](https://molcure.github.io/HAIVE-OS_Pages/#/hos_device_layer/?id=haive-os-device-database). We also added a function description and selected our `device_type` `TestDevice`.

Lastly, since we wish to serialize our function call into a string, we have to define a serializer in the `api_serializers` table:

![Define API Serializer](img/new_device4.png 'Define API Serializer')

Don't forget to also select our function `my_test_function` for the `function_name`. The way that the serializer works is that once HAIVE OS receives a request for a device API call, it serializes the argument data by substituting each `$` with an argument from the `arguments` list described in `apis`. This implies that `len(arguments) == #$` must hold. We used the characters `A` and `B` in our serializer, which allows us to parse for specific arguments on the ESP.

Now we are ready to make a request to our new functionality, but first we need to startup HAIVE OS.

## Starting HAIVE OS and making a `DeviceAPI` request

First make sure that the `ESP32` we programmed in the first stepped is turned off. Then let's startup HAIVE OS:

```shell
cd HAIVE-OS/
. install/setup.bash
ros2 launch hos_run hos_run.launch.py
```

Your console output should look similar to this:

```
[INFO] [launch]: All log files can be found below /home/ubuntu/.ros/log/2023-05-25-04-08-14-023253-ubuntu-1902
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [rosbridge_websocket-1]: process started with pid [1903]
[INFO] [rosapi_node-2]: process started with pid [1905]
[rosbridge_websocket-1] [INFO] [1684987697.437130647] [rosbridge_websocket]: Rosbridge WebSocket server started on port 9091
[INFO] [launch.user]: 
'##::::'##::::'###::::'####:'##::::'##:'########:::::'#######:::'######::
 ##:::: ##:::'## ##:::. ##:: ##:::: ##: ##.....:::::'##.... ##:'##... ##:
 ##:::: ##::'##:. ##::: ##:: ##:::: ##: ##:::::::::: ##:::: ##: ##:::..::
 #########:'##:::. ##:: ##:: ##:::: ##: ######:::::: ##:::: ##:. ######::
 ##.... ##: #########:: ##::. ##:: ##:: ##...::::::: ##:::: ##::..... ##:
 ##:::: ##: ##.... ##:: ##:::. ## ##::: ##:::::::::: ##:::: ##:'##::: ##:
 ##:::: ##: ##:::: ##:'####:::. ###:::: ########::::. #######::. ######::
..:::::..::..:::::..::....:::::...:::::........::::::.......::::......:::

[INFO] [device_manager-3]: process started with pid [1927]
[device_manager-3] SAVED DB INFO IN: /home/ubuntu/HAIVE-OS/install/hos_device_layer/share/hos_device_layer/db/devices.json
[device_manager-3] SAVED DB INFO IN: /home/ubuntu/HAIVE-OS/install/hos_device_layer/share/hos_device_layer/db/apis.json
[device_manager-3] SAVED DB INFO IN: /home/ubuntu/HAIVE-OS/install/hos_device_layer/share/hos_device_layer/db/api_serializers.json
[device_manager-3] SAVED DB INFO IN: /home/ubuntu/HAIVE-OS/install/hos_device_layer/share/hos_device_layer/db/streams.json
[device_manager-3] SAVED DB INFO IN: /home/ubuntu/HAIVE-OS/install/hos_device_layer/share/hos_device_layer/db/fleet-shinkawasaki.json
[device_manager-3] [INFO] [1684987704.223878952] [device_manager]: Running device_manager
[INFO] [l2_proxy-4]: process started with pid [1939]
[l2_proxy-4] [INFO] [1684987707.654673109] [l2_proxy]: Running l2_proxy
```

Now we should be ready to connect our `ESP32` to HAIVE OS. Connect it to your PC and open a serial monitor set to baud rate `74880`. Press the `RST` button on the `ESP32`and the serial monitor output should look similar to this:

```
Ready
HOS_CLIENT::INFO: Connected to HAIVE OS.
HOS_CLIENT::INFO: Ping HAIVE OS with device_id >> 10758656
HOS_CLIENT::INFO: Publishing message >> {"op":"publish","topic":"haive_os/device/ping","msg":{"data":10758656}}
HOS_CLIENT::INFO: Received message >> {"op": "service_response", "service": "haive_os/device/connect", "values": {"success": true, "error": ""}, "result": true}
HOS_CLIENT::INFO: Service response received.
-------------
service_response
haive_os/device/connect
1

1
-------------
HOS_CLIENT::INFO: Ping HAIVE OS with device_id >> 10758656
HOS_CLIENT::INFO: Publishing message >> {"op":"publish","topic":"haive_os/device/ping","msg":{"data":10758656}}

```

On the HAIVE OS side we should see an error that reads similar to this:

```shell
[device_manager-3] [ERROR] [1684831838.095123738] [device_manager]: Unable to get device id for connected device (uid=10758656). Please check integrity of database!
```

This is good, since it means the connection request from our `ESP32` went through. We also get a hint for the last step of what we need to do. If we remember the step of updating the HAIVE OS device database, we filled in a placeholder device uid `999`. We have to go back and update the UID with the actual value `10758656` in the `fleet-shinkawasaki` table:

![Update Device UID](img/new_device5.png 'Update Device UID')

Please also make sure that this UID is not already assigned to another device!

We need to reboot our system, since we need to reload the device database into memory. Please follow now these steps:

1. Turn off `ESP32`
2. Quit HAIVE OS (ctrl+c)
3. Restart HAIVE OS
```shell
ros2 launch hos_run hos_run.launch.py
```
4. Start the `ESP32`

Your shell console of HAIVE OS should now read something like this:

```
[rosbridge_websocket-1] [INFO] [1684990811.068259883] [rosbridge_websocket]: Client connected. 1 clients total.
[rosbridge_websocket-1] [INFO] [1684990811.137724580] [rosbridge_websocket]: [Client 405bd135-ec8f-4090-a8d7-b9f0c7d4492d] Advertised service haive_os/device/uid10758656/serialized_command.
[device_manager-3] [INFO] [1684990811.152298369] [device_manager]: device-10758656: device api service registered.
[device_manager-3] [INFO] [1684990811.155815187] [device_manager]: device-10758656 #streams: 0.
[device_manager-3] [INFO] [1684990811.159051268] [device_manager]: New device connected: uid=10758656
```

Now we should be ready to send our first `DeviceAPICall` requests. So let's open a new shell intance on our HAIVE OS machine and send the service call:

```shell
ros2 service call haive_os/device/api_call hos_interfaces/srv/DeviceAPICall '{device_id: 'T1234', function_name: 'my_test_function', args: [{'name': 'num1', 'type': 'int32', 'data': '2'}, {'name': 'num2', 'type': 'int32', 'data': '3'}]}'
```

If we check the serial monitor of our `ESP32` we should see some output similar to this:

```
HOS_CLIENT::INFO: Received message >> {"op": "call_service", "id": "service_request:haive_os/device/uid10758656/serialized_command:1", "service": "haive_os/device/uid10758656/serialized_command", "args": {"cmd_id": 0, "serialized_command": "A2B3", "function_name": "my_test_function"}}
HOS_CLIENT::INFO: Service call received.
-------------
call_service
service_request:haive_os/device/uid10758656/serialized_command:1
haive_os/device/uid10758656/serialized_command
0
A2B3
-------------
HOS_CLIENT::INFO: Received new L1 command >> A2B3
HOS_CLIENT::INFO: Sending service response >> {"op":"service_response","id":"service_request:haive_os/device/uid10758656/serialized_command:1","service":"haive_os/device/uid10758656/serialized_command","values":{"is_valid":true,"error":""},"result":true}
A2B3
```

As we see, we received our encoded string holding our function arguments `A2B3`. Congratulations!

Checking our shell output of HAIVE OS, we can also see that the API call was received.

```
[device_manager-3] [INFO] [1684992207.247540049] [device_manager]: Received api call: task_id=0 | device_id=T1234 | function_name=my_test_function | args=[hos_interfaces.msg.TypedField(name='num1', type='int32', data='2'), hos_interfaces.msg.TypedField(name='num2', type='int32', data='3')]
[device_manager-3] [INFO] [1684992207.250480870] [device_manager]: Serialized command >> A2B3
[device_manager-3] [INFO] [1684992207.254103714] [device_manager]: api call success
[device_manager-3] [INFO] [1684992207.372172068] [device_manager]: Received device command result: device_uid=10758656 | cmd_id=0 | success=True | error= | result_str=
```

We also see that we received a device command result. However, its `result_str` is empty. This is because we did not prepare an appropriate result on the `ESP32`. So lets update the firmware code.

First we should add a function that can take a command String object and parse its contents into alphabet and int value arrays:

```C++
// ...

char alphabets[DATA_LENGTH/2];
int ints[DATA_LENGTH/2];

void parseString(const String& inputString, char* lettersArray, int* numbersArray) {
  int letterIndex = 0;
  int numberIndex = 0;
  int inputLength = inputString.length();
  
  for (int i = 0; i < inputLength; i++) {
    char currentChar = inputString.charAt(i);
    
    if (isAlpha(currentChar)) {
      if (letterIndex < DATA_LENGTH/2) {
        lettersArray[letterIndex++] = currentChar;
      }
    } else if (isDigit(currentChar)) {
      if (numberIndex < DATA_LENGTH/2) {
        int number = 0;
        while (i < inputLength && isDigit(inputString.charAt(i))) {
          number = (number * 10) + (inputString.charAt(i) - '0');
          i++;
        }
        i--; // Move back one position to account for the loop increment
        
        numbersArray[numberIndex++] = number;
      }
    }
  }
}

// ...
```

In the `process_messages` function we need to add our parsing function, calculate the result and then send it back to HAIVE OS:

```C++
// ...

void process_messages()
{
  // Process commands received from HAIVE OS
  if (hos_client.command_available()) {
//#if DEBUG_PRINT
//    Serial.println("Sending HAIVE OS command to slave.");
//#endif

    // command_id = hos_client.send_command(Serial1);
    // executing_command = true;

    String cmd = hos_client.get_command();
    Serial.println(cmd);

    // Parse the command string
    parseString(cmd, alphabets, ints);

    // Get values from our value array and calculate the sum
    int sum = ints[0] + ints[1];

    // Create results string of format V$
    String result = "V" + String(sum);

    // Send the result
    hos_client.report_cmd_result(command_id, true, result);
    hos_client.set_device_state(IDLE);
  }

  // ...

}

// ...
```

The current protocol of preparing the result string is appending serialized values with a `V` prefix. This is how the HAIVE OS device manager (currently) reasons about distinct result values.

After uploading the new firmware to the ESP, we can try resending our request. But first, we also want to receive the final result of our request so we will also want to subscribe to the `DeviceAPICallResult` topic. So let's open another new shell instance on our HAIVE OS machine and subscribe to the result topic:

```shell
ros2 topic echo /haive_os/device/api_call_result
```

After powering on the `ESP32`, make sure you see it is connected to HAIVE OS. Then we can send again our API request:

```shell
ros2 service call haive_os/device/api_call hos_interfaces/srv/DeviceAPICall '{device_id: 'T1234', function_name: 'my_test_function', args: [{'name': 'num1', 'type': 'int32', 'data': '2'}, {'name': 'num2', 'type': 'int32', 'data': '3'}]}'
```

When checking the shell instance listening to the `api_call_result` you should see the following output:

```
task_id: 0
success: true
error: ''
result_jsons: '{"sum": 5}'
request_time_s: 1684996411.1909344
response_time_s: 1684996411.3194978
---
```

This concludes this section and should give you a start for developing new devices for the HAIVE platform.
