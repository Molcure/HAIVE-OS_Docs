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

### `HaiveInterface` and `ContainerInterface` Classes

TODO
