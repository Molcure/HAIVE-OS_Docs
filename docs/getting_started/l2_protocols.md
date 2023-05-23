# Running L2 Protocols

Under construction ...

- Launch HAIVE OS `ros2 launch hos_run hos_run.launch.py`

## Install HAIVE OS On ESP8266 Boards

- Download HAIVE4_Firmware repo
- Switch to branch `haive_os`
- Upload `ESP8266` firmware to ESP

## Update HAIVE OS Device Database

- Start `ESP8266`
- Serial monitor of ESP should read
```
HOS_CLIENT::INFO: Connected to HAIVE OS.
HOS_CLIENT::INFO: Ping HAIVE OS with device_id >> 10758656
```
- Terminal output of HAIVE OS should show error
```
[device_manager-3] [ERROR] [1684831838.095123738] [device_manager]: Unable to get device id for connected device (uid=10758656). Please check integrity of database!
```
- Open HAIVE OS device database: https://airtable.com/appR9FYsP809nu76u/tbl7KDBSSYoh6h6Tz/viw7VRDMwasaRVVZX?blocks=hide
- Open table `fleet-shinkawasaki` and find `device_id` that the ESP8266 belongs to
- Paste the uid (e.g. `10758656`) into the `device_uid` field.
- Restart HAIVE OS to load the updated device database
- Start the `ESP8266` and the terminal output should read something similar to this
```
[rosbridge_websocket-1] [INFO] [1684832130.179877493] [rosbridge_websocket]: Client connected. 1 clients total.
[rosbridge_websocket-1] [INFO] [1684832130.248676247] [rosbridge_websocket]: [Client d82b7f96-2ecd-4c18-8a14-1e2b439568bb] Advertised service haive_os/device/uid10758656/serialized_command.
[device_manager-3] [INFO] [1684832130.261701403] [device_manager]: device-10758656: device api service registered.
[device_manager-3] [INFO] [1684832130.265217048] [device_manager]: device-10758656 #streams: 0.
[device_manager-3] [INFO] [1684832130.268682063] [device_manager]: New device connected: uid=10758656
```

For this tutorial we will only need one HAIVE with device ID `H4001`, so updating one ESP8266 and the entry for `H4001` in the device databse should be enough for now.

## Sending The Protocol

- Install the updated ESP8266 into HAIVE `H4001` and start the power
- Confirm in the HAIVE OS terminal that the ESP8266 reconnected to the system
- Open another console on the HAIVE OS machine and navigate to the HAIVE-OS folder
- Make sure to execute `setup.bash`:
```shell
. install/setup.bash 
```
- Use the following command to load and start execution of a protocol:
```shell
ros2 topic pub -1 /haive_os/load_protocol std_msgs/String 'data: TT_TEST_H4001.json'
```
- To abort protocol execution you can run:
```shell
ros2 topic pub -1 /haive_os/stop_protocol std_msgs/Empty
```
