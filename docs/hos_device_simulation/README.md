# hos_device_simulation

The `hos_device_simulation` package contains a software simulation of the HAIVE4 platform. See the [TODO](TODO) for an example of how to use the `hos_device_simulation` package.

## Haive4Simulation Node

The [Haive4Simulation](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_device_simulation/hos_device_simulation/simulation.py#L19) node is a native ROS node that launches the simulation of a HAIVE system. The HAIVE system is described by a layout file in `hos_device_simulation/layouts` and the node sets up device interfaces using the [roslibpy library](https://roslibpy.readthedocs.io/en/latest/) depending on that description.

## Layout Files

The layout file describes the HAIVE system the `Haive4Simulation` node is supposed to simulate. Let's take a look at the `test_layout.json` file:

```json
{
  "fleet": "shinkawasaki",
  "layout": [
    {
      "haive_id": "H4001",
      "position": [
        1, 
        1
      ],
      "containers": {
        "C4003": 1,
        "C4002": 9,
        "C4001": 11
      }
    },
    {
      "haive_id": "H4002",
      "position": [
        2,
        2
      ],
      "containers": {
        "C4007": 3,
        "C4008": 9
      }
    },
    {
      "haive_id": "H4003",
      "position": [
        2,
        1
      ],
      "containers": {
        "C4005": 1,
        "C4006": 3,
        "C4010": 5,
        "C4011": 11
      }
    }
  ]
}
```

- `fleet`: Contains a string describing the fleet name. This fleet name needs to be a name that is available in the [HAIVE OS device database](https://github.com/Molcure/HAIVE-OS/blob/master/docs/hos_device_layer/?id=haive-os-device-database).
- `layout`: List of dictionaries that describe HAIVE configurations.
  - `haive_id`: A device identifier string of the fleet (e.g. `H4001`)
  - `position`: Position of the HAIVE in a hexagonal grid. The HAIVE system follows an [“odd-r” horizontal layout](https://www.redblobgames.com/grids/hexagons/#coordinates).
  - `containers`: A list of key-value pairs that describe containers of the HAIVE workstation and their positions. The key is `container_id` (e.g. `C4001`) according to the device database. The position refers to the HAIVE workstation slot that the container is initially placed on (possible values are `[0, 1, 3, 5, 7, 9, 11]`).

For every HAIVE and Container description in the layout file, the simulation node creates an instance of a [DeviceInterface](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_device_simulation/hos_device_simulation/device_interface.py#L12).

## `DeviceInterface` Class

The [DeviceInterface](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_device_simulation/hos_device_simulation/device_interface.py) class is an abstract class that is inherited by [HaiveInterfaces](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_device_simulation/hos_device_simulation/interfaces_haive.py) and [ContainerInterfaces](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_device_simulation/hos_device_simulation/interfaces_container.py). While the `Haive4Simulation` node is a native ROS node, the `DeviceInterface` instances are `roslibpy clients`. This gives a more accurate representation of HAIVE OS clients usually being non-ROS systems that use [rosbridge](https://github.com/RobotWebTools/rosbridge_suite) to interface with HAIVE OS.

The `DeviceInterface` holds the client connection in a private `_client` field which is of type [roslibpy.Ros](https://roslibpy.readthedocs.io/en/latest/reference/index.html). All basic HAIVE OS communication is implemented in the `DeviceInterface` class.

## `HaiveInterface` and `ContainerInterface` Classes

The `HaiveInterface` and `ContainerInterface` classes do not implement any low level communication with HAIVE OS and usually only have contain callback functions that model some robot behavior. These callbacks can be registered using the `_register_device_command` method. You will find the registration of the callback functions in the implementation of the `power_on` method, which gets called once the HAIVE OS has started up.

Let's take a look at the [BaseHaiveInterface](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_device_simulation/hos_device_simulation/interfaces_haive.py#L8) as an example:

```python
class BaseHaiveInterface(DeviceInterface):
  def __init__(self, uid: int, simulation_node: Node, position_x: int, position_y: int) -> None:
    super().__init__(uid, simulation_node)
    self._position = (position_x, position_y)

  def power_on(self):
    super().power_on()

    self._register_device_command('haive_led_all_off', self.cmd_led_all_off)
    self._register_device_command('haive_led_all_on', self.cmd_led_all_on)
    self._register_device_command('haive_led_slot_on', self.cmd_led_slot_on)
    self._register_device_command('turntable_move', self.cmd_turntable_move)
    self._register_device_command('set_slot_power', self.cmd_slot_power)
    self._register_device_command('haive_get_position', self.cmd_get_position)

  # ...
```

To register a device command callback function you have to pass in a string identifier and a reference to the method to call. The string identifier needs to be identical with a `function_name` defined in the [HAIVE OS device database](https://github.com/Molcure/HAIVE-OS/blob/master/docs/hos_device_layer/?id=haive-os-device-database).

For now  most of the functions modeled in the `DeviceInterface` classes do nothing except logging an info and then reporting the result of the function:

```python
  # ...

  def cmd_led_slot_on(self, cmd_id: int, serialized_cmd: str):
    self._logger.info(f"uid-{self._uid}, cmd-{cmd_id}: cmd_led_slot_on >> {serialized_cmd}")
    self._report_cmd_result(cmd_id, True)  # Every device command callback needs to send a result when the command has finished

  # ...

```

One example of a more complex implementation that makes the [TODO](TODO) example for a digital twin implementation possible, we modeled the streaming of motor positions, which makes the 3D simulation of the HAIVE move:

```python
  # ...

  def cmd_turntable_move(self, cmd_id: int, serialized_cmd: str):
    self._logger.info(f"uid-{self._uid}, cmd-{cmd_id}: cmd_turntable_move >> {serialized_cmd}")

    cmd = self._deserialize_cmd(serialized_cmd)

    from_slot = self._haive_graph.get_turntable_position(self._device_id)
    to_slot = int(cmd[3])

    self._logger.info(f"cmd_turntable_move >> from_slot:{from_slot}, to_slot:{to_slot}")

    self._haive_graph.move_turntable(self._device_id, to_slot)

    turntable_values = {
      1:  300, # 1918,
      # delta: 143
      3:  0, # 1775,
      # delta: 141
      5:  60, # 1634,
      # delta: 137
      # Flips side
      7:  120, # 1497,
      # delta: 145
      9:  180, #1352,
      # delta: 138
      11: 240, # 1214
      # delta: 704
    }
    from_value = turntable_values[from_slot]
    to_value = turntable_values[to_slot]

    diff = abs(from_value - to_value)
    distance = min(diff, 360 - diff)

    factor = -1.0 if ((from_value - to_value) + 360) % 360 < 180 else 1.0

    cmd_data = {
      'cmd_id': cmd_id,
      'from_value': from_value,
      'to_value': to_value,
      'distance': distance,
      'factor': factor,
      'elapsed': 0.0,
      'duration': 1.0,
    }
    self._logger.info(f"##### cmd_turntable_move >> cmd_data:{cmd_data}")

    simulate_streams = self._node.get_parameter('simulate_streams').get_parameter_value().bool_value
    if not simulate_streams:
      self._report_cmd_result(cmd_data['cmd_id'], True)
    else:
      self._client.call_later(0.033, lambda: self._simulate_turntable(cmd_data))

  def _simulate_turntable(self, cmd_data: Dict):
    if cmd_data['elapsed'] < cmd_data['duration']:
      def ease_out_expo(x: float) -> float:
        if x == 1.0:
          return 1.0
        return 1.0 - pow(2.0, -10.0 * x)

      def lerp(from_value: float, to_value: float, t: float) -> float:
        return from_value * (1.0 - t) + (to_value * t)

      value = lerp(0.0, cmd_data['distance'], ease_out_expo(cmd_data['elapsed']/cmd_data['duration']))
      self._stream_data('turntable_motor', str((cmd_data['from_value'] + value * cmd_data['factor']) % 360))

      cmd_data['elapsed'] += 0.033
      self._client.call_later(0.033, lambda: self._simulate_turntable(cmd_data))
    else:
      self._stream_data('turntable_motor', str((cmd_data['from_value'] + cmd_data['distance'] * cmd_data['factor']) % 360))
      self._report_cmd_result(cmd_data['cmd_id'], True)

  # ...
```
