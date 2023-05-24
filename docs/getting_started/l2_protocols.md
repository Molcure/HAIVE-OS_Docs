# Running L2 Protocols

In this section we will go through the process of installing the HAIVE OS client library onto an ESP8266 of the HAIVE4 hardware, and then launching a test protocol using HAIVE OS.

## Install HAIVE OS On ESP8266 Boards

1. Launch HAIVE OS 
```
ros2 launch hos_run hos_run.launch.py
```
2. Download the [HAIVE4_Firmware repo](https://github.com/Molcure/HAIVE4_Firmware)
3. Switch to branch `haive_os`
4. Upload `ESP8266` firmware to ESP

## Update HAIVE OS Device Database

We first need to make sure that HAIVE OS can identify the `ESP8266` that we want to connect. For that we first have to find out the `device_uid`:

1. Start `ESP8266`
2. Serial monitor of ESP should read
```
HOS_CLIENT::INFO: Connected to HAIVE OS.
HOS_CLIENT::INFO: Ping HAIVE OS with device_id >> 10758656
```
3. Terminal output of HAIVE OS shows an error if the device uid can not be identified:
```
[device_manager-3] [ERROR] [1684831838.095123738] [device_manager]: Unable to get device id for connected device (uid=10758656). Please check integrity of database!
```
4. Open HAIVE OS device database: https://airtable.com/appR9FYsP809nu76u/tbl7KDBSSYoh6h6Tz/viw7VRDMwasaRVVZX?blocks=hide
5. Open table `fleet-shinkawasaki` and find `device_id` that the ESP8266 belongs to
6. Paste the uid (e.g. `10758656`) into the `device_uid` field.
7. Restart HAIVE OS, which will load the updated device database
8. Start the `ESP8266` and the terminal output should read something similar to this
```
[rosbridge_websocket-1] [INFO] [1684832130.179877493] [rosbridge_websocket]: Client connected. 1 clients total.
[rosbridge_websocket-1] [INFO] [1684832130.248676247] [rosbridge_websocket]: [Client d82b7f96-2ecd-4c18-8a14-1e2b439568bb] Advertised service haive_os/device/uid10758656/serialized_command.
[device_manager-3] [INFO] [1684832130.261701403] [device_manager]: device-10758656: device api service registered.
[device_manager-3] [INFO] [1684832130.265217048] [device_manager]: device-10758656 #streams: 0.
[device_manager-3] [INFO] [1684832130.268682063] [device_manager]: New device connected: uid=10758656
```

For this tutorial we will only need one HAIVE with device ID `H4001`, so updating one ESP8266 and the entry for `H4001` in the device databse should be enough for now.

## Sending The Protocol

1. Install the updated `ESP8266` into HAIVE `H4001` and start the power
2. Confirm in the HAIVE OS terminal that the `ESP8266` connected to the system. If it didn't, try restarting the HAIVE.
3. Open another console on the HAIVE OS machine and navigate to the `HAIVE-OS` folder
4. Make sure to execute `setup.bash`:
```shell
. install/setup.bash 
```
5. Use the following command to load and start execution of a protocol:
```shell
ros2 topic pub -1 /haive_os/load_protocol std_msgs/String 'data: TT_TEST_H4001.json'
```
- WARNING: Not docking the opener attachment to `H4001` before starting the protocol will result in failure! The reason for that is the emittance of a useless error byte by the Arduino Due firmware. Be aware!
- Once the protocol has finished, ther terminal should show a message similar to this:
```shell
[l2_proxy-4] -------- FINISHED PROTOCOL EXECUTION --------
[l2_proxy-4] 
[l2_proxy-4] Name: TT_TEST_H4001
[l2_proxy-4] Executed commands: 20
[l2_proxy-4] Time: 0:00:15
[l2_proxy-4] 
```
- To abort protocol execution you can run:
```shell
ros2 topic pub -1 /haive_os/stop_protocol std_msgs/Empty
```
