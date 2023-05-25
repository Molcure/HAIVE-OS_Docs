# hos_device_layer

The device layer is the lowest layer in the HAIVE OS. It is responsible for handling device connections and exposing functionality of connected devices to upper layers.

## Device Manager

The `DeviceManager` is the node that contains all logic of the device layer. The public interface of the node consists of the following ROS topics and services:

### Subscribed
- `DEVICE_PING_TOPIC = 'haive_os/device/ping'`: Pings HAIVE OS from device
- `DEVICE_CMD_RESULT_TOPIC = 'haive_os/device/command_result'`: Sends result of command call from device to HAIVE OS
- `DEVICE_STREAM_TOPIC(uid, stream_name) = 'haive_os/device/uid{uid}/stream/{stream_name}'`: Streaming data from device to HAIVE OS

### Published

- `DEVICE_API_CALL_RESULT_TOPIC = 'haive_os/device/api_call_result'`: Sends result of device api call from HAIVE OS upwards
- `DEVICE_CONNECTED = 'haive_os/device/connected'`: Sends connection info when a new device connects to HAIVE OS
- `DEVICE_TIMEOUT = 'haive_os/device/timeout'`: Sends connection info when a device timed out
- `DEVICE_RECONNECTED = 'haive_os/device/reconnected'`: Sends connection info when a known device reconnects to HAIVE OS
- `DEVICE_DISCONNECTED = 'haive_os/device/disconnected'`: Sends connection info when a known device disconnects from HAIVE OS

### Served

- `CONNECT_DEVICE_SERVICE = 'haive_os/device/connect'`: Requests device connection to HAIVE OS
- `DEVICE_COMMAND_SERVICE(uid: int) = 'haive_os/device/uid{uid}/command'`: Serves device command interface
- `SERIALIZED_DEVICE_COMMAND_SERVICE(uid: int) = 'haive_os/device/uid{uid}/serialized_command'`: Serves device command interface using serialized data (e.g. used for L1)
- `DEVICE_API_CALL_SERVICE = 'haive_os/device/api_call'`: Serves device api from the device layer upwards
- `DEVICE_CONNECTION_INFOS_SERVICE = 'haive_os/connected_devices'`: Serves device connection infos from the device layer upwards

The `DeviceManager` also loads device specification information about the HAIVE system to have knowledge about device types, device functionality and fleet constellations. This information is stored in an online database on Airtable, which serves as the human-interface to edit the specification. Once data is loaded from the online database, a copy of that information is also stored locally and can be used to start the node in an offline environemnt (see `device_db` launch argument in `hos_run.launch.py`).

## HAIVE OS Device Database

The HAIVE OS device database can be viewed using [Airtable](https://airtable.com/appR9FYsP809nu76u?ao=cmVjZW50). The base currently consists of the following core tables:

- `devices`: Contains all devices currently contained in the HAIVE system (status of HAIVE4.2)
  - `device_type`: A device type is equivalent to a distinct hardware device
  - `device_role`: A device role describes a class of several device types (possible values: `[Haive, Container, Bridge, Equipment, Sensor]`)
  - `serialized_api`: Flag which indicates of the device API functionality needs to be serialized
  - `ros_target`: Describes the type of a devices client
  - `description`: Description of the device
- `apis`: Describes the API of a device
  - `function_name`: Unique name of the function
  - `arguments`: List of function arguments that are encoded as strings of the format `{arg_name}:{arg_type}`. Possible `arg_type` values are `[uint8~uint64, int8~int64, float32~float64, bool, string]`.
  - `results`: List of result values that are encoded as strings of the format `{arg_name}:{arg_type}`. Possible `arg_type` values are `[uint8~uint64, int8~int64, float32~float64, bool, string]`.
  - `description`: Description of the function (will also be added by `hos_generator` to the `hos_device_api.py` file)
- `serializers`: Holds rules for serializing device APIs
  - `serializer`: A string that describes the rule serializing a device API by substituting each `$` with an argument from the `arguments` list described in `apis`. This implies that `len(arguments) == #$` must hold.
- `streams`: Describes data streams that are exposed by devices
  - `stream`: Unique name of the stream
  - `values`: List of values that are streamed

Additionally the databse describes different fleets, that speficy HAIVE system configurations. A fleet table creates a relation between a device identifier and a numerical identifier:

- `fleet-*`:
  - `device_id`: User level identifier which can be a duplicate across different fleets (e.g. `H4001`)
  - `device_uid`: A globally unique numerical identifier which is usually defined by a chip ID (e.g. chip id of `ESP32`)

## Device API Calls

The way to interact with devices using HAIVE OS is to use `DeviceAPICalls` that are requested asynchronously to the `DeviceManager`. They are in fact ROS service calls. In `hos_device_layer/gen/hos_device_api.py` you will find convenient functions that handle the process of creating a request and sending it to the device manager for you. All of these functions will return an object with associated information regarding you request:

```python
@dataclass
class DeviceAPICallInfo:
  call_time: float
  future: Future
  client: Client
```

The future can be checked while spinning your node to receive the response of your request. As described in the `hos_interfaces` section, a `DeviceAPICall` response will tell you if your service call was successful and will also give you a unique `task_id` for identifying your `DeviceAPICallResult`. For that, your node needs to be subscribed to the `DEVICE_API_CALL_RESULT_TOPIC` published by the `DeviceManager`. This level of indirection is necessary since device functions may take several seconds to finish executing.

Check [TODO](TODO) for a basic example of how to make a `DeviceAPICall` from a node and process the result.
