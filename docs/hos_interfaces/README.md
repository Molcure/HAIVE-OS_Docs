# hos_interfaces

This package contains all ROS interfaces currently used by HAIVE OS.

## Messages

### `ConnectionInfo.msg`

This message is emitted by the `DeviceManager` whenever the connection status of a device changes.

```
string device_id
uint32 device_uid
string device_type
float64 event_time_s
```

- `device_id`: A device identifier as defined in the device database (e.g. `H4001`)
- `device_uid`: A unique numerical identifier (determined by an ESP chip id)
- `device_type`: A device type as defined in the device database (e.g. `DeltaHaive`)
- `event_time_s`: Time in seconds since the epoch as a floating point number

### `DeviceCommandResult.msg`

This message is emitted by a device whenever it has finished processing a `DeviceCommand`. Usually only the `DeviceManager` should care for this message.

```
uint32 uid
uint32 cmd_id
bool success
string error
string result_str
```

- `uid`: Unique numerical identifier (determined by an ESP chip id) of the device that processed the `DeviceCommand`
- `cmd_id`: A unique numerical identifier (determined by the device manager at the time of creating a device command request)
- `success`: Flag that indicates if device command exectuion was successful or not
- `error`: Contains an error message, if a device command was not successful
- `result_str`: Contains a serialized JSON object with result data as defined in the device database

### `DeviceAPICallResult.msg`

This message is emitted by the `DeviceManager` whenever a `DeviceCommand` has finished processing.

```
uint32 task_id
bool success
string error
string result_jsons
float64 request_time_s
float64 response_time_s
```

- `task_id`: A unique numerical identifier (determined by the device manager at the time of making the `DeviceAPICall`)
- `success`: Flag that indicates if a device API call was successful or not
- `error`: Contains an error message, if a device API call was not successful
- `result_jsons`: Contains a serialized JSON object with result data as defined in the device database
- `request_time_s`: Time in seconds since the epoch as a floating point number when the request was made by the device manager to the device
- `response_time_s`: Time in seconds since the epoch as a floating point number when the response was received by the device manager from the device

### `DeviceStream.msg`

This message contains serialized data that is streamed by devices.

```
uint32 uid
string data
```

- `uid`: Unique numerical identifier (determined by an ESP chip id) of the device that is streaming the data
- `data`: Serialized data that needs to be casted using information contained in the device database

### `TypedField.msg`

This is a datatype used by certain services to encode meta information for a serialized data field.

```
string name
string type
string data
```

- `name`: Name of the field
- `type`: Type of the field (possible values: `[uint8~uint64, int8~int64, float32~float64, bool, string]`)
- `data`: Serialized data


## Services

### ConnectDevice.srv

This is a core service served by the `DeviceManager` and is used by devices to connect devices to HAIVE OS.

```
uint32 uid
---
bool success
string error
```

- `uid`: Unique numerical identifier (determined by an ESP chip id) of the device that is requesting to connect to HAIVE OS
- `success`: Flag that indicates if connection to HAIVE OS was successful or not
- `error`: Contains an error message, if device connection was not successfully handled by the device manager

### DeviceCommand.srv

This is a core service served by devices to the `DeviceManager`.

```
uint32 cmd_id
string function_name
uint8 num_args
TypedField[] args
---
bool is_valid
string error
```

- `cmd_id`: A unique numerical identifier (determined by the device manager)
- `function_name`: Name of the function to be called on the client side
- `num_args`: Number of arguments of the function
- `args`: Function arguments passed as `TypedFields`
- `is_valid`: Flag that indicates if `DeviceCommand` request was successful or not
- `error`: Contains an error message, if `DeviceCommand` request was not successful

### SerializedDeviceCommand.srv

This is a core service served by devices to the `DeviceManager`. This is usally used by embedded devices (like found in HAIVE4) due to memory limitations of the serial buffer. The serialized command is created by the `DeviceManager` using information from the device database.

```
uint32 cmd_id
string serialized_command
string function_name
---
bool is_valid
string error
```

- `cmd_id`: A unique numerical identifier (determined by the device manager)
- `serialized_command`: A serialized form of a function exposed by a device (see L1 specification for an example)
- `function_name`: Name of the function that serialzed command originated from
- `is_valid`: Flag that indicates if `DeviceCommand` request was successful or not
- `error`: Contains an error message, if `DeviceCommand` request was not successful

### DeviceAPICall.srv

This is a core service served by the `DeviceManager`. It serves as a centralized interface to execute functionality of devices in HAIVE OS.

```
string device_id
string function_name
TypedField[] args
---
bool is_valid
string error
uint32 task_id
```

- `device_id`: A device identifier as defined in the device database (e.g. `H4001`) that is the target of the `DeviceAPICall` (i.e., it will trigger a `DeviceCommand`)
- `function_name`: Name of the function to to be executed by the device
- `args`: Function arguments passed as `TypedFields`
- `is_valid`: Flag that indicates if `DeviceAPICall` request was successful or not
- `error`: Contains an error message, if `DeviceAPICall` request was not successful
- `task_id`: A unique numerical identifier (determined by the device manager) to make it possible for the client to relate a `DeviceAPICallResult` to the `DeviceAPICall`

### DeviceConnectionInfos.srv

This is a core service served by the `DeviceManager`. It allows to query information about the currently connected devices.

```
---
ConnectionInfo[] device_infos
```

- `device_infos`: List of device connection infos (see `ConnectionInfo`)


### BehaviorRequestMoveContainer.srv

This is a service served by the `PathFindingBehaviorNode`. It allows to send requests to move a container from its current position to a new position in the HAIVE system.

```
string container_id
string haive_id
uint8 slot
---
bool success
string error
```

- `container_id`: Device identifier of the container to be moved (e.g. `C4001`)
- `haive_id`: Device identifier of the goal HAIVE to be moved to (e.g. `H4001`)
- `slot`: Slot number to be moved to (e.g. `1`)
- `success`: Flag that indicates if pathfinding request was successful of not
- `error`: Contains an error message, if pathfinding request was not successful

## Adding New Interfaces

To add a new interface to HAIVE OS, you have to follow these steps:

1. Create a new `.msg` or `.srv` file inside the `hos_interfaces` package
2. Update the `CMakeLists.txt` file inside the `hos_interfaces` package

```
# ...

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TypedField.msg"
  "msg/DeviceAPICallResult.msg"
  "msg/DeviceCommandResult.msg"
  "msg/DeviceStream.msg"
  "msg/ConnectionInfo.msg"
  # Add new msg file here <--
  "srv/ConnectDevice.srv"
  "srv/DeviceAPICall.srv"
  "srv/DeviceCommand.srv"
  "srv/SerializedDeviceCommand.srv"
  "srv/BehaviorRequestMoveContainer.srv"
  "srv/DeviceConnectionInfos.srv"
  # Add new srv file here <--
)

# ...
```

3. Recompile the HAIVE OS workspace

```
colcon build
. install/setup.bash
```
