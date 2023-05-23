# hos_l2_proxy

The `hos_l2_proxy` package allows HAIVE OS to handle protocol requests that follow the [L2 specification](https://airtable.com/app56FFN9rPq29WLM/tbl8q73cJDZ7iWL6e/viwGo8UIqPMPVYSmR?blocks=hide). See the [TODO](TODO) for an example of how to use the `hos_l2_proxy` package.

## `L2Proxy` Node

This is proxy node that can receive L2 protocols in JSON format. It translates L2 commands into HAIVE OS device command requests and executes these commands according to the L2 specification. It exposes two topics to handle protocol execution:

- `haive_os/load_protocol`
- `haive_os/stop_protocol`

The `L2Proxy` node is part of the default configuration when running the `hos_run.launch.py` file:

```shell
ros2 launch hos_run hos_run.launch.py
```

We can also simulate the execution of L2 commands using the [hos_device_simulation](https://github.com/Molcure/HAIVE-OS/blob/master/docs/hos_device_simulation/) package:

```shell
ros2 launch hos_run hos_run.launch.py haive4_simulation:='True' l2_proxy_protocol_file:='TT_TEST_H4001.json' l2_proxy_ignore_wait_commands:='--ignore-wait'
```

To get more information about possible launch file configuraitons please check the [hos_run documentation](https://github.com/Molcure/HAIVE-OS/blob/master/docs/hos_run/).

## `L2Simulation` Class

To be able to execute L2 commands, the `hos_l2_proxy` package implements a simple [virtual machine](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_l2_proxy/hos_l2_proxy/l2_simulation.py#L100) for executing a kind of L2 bytecode. This is necessary to derive certain parameters when translating from L2 commands to HAIVE OS device commands.

The initilization of the `L2Simulation` happens in the [_parse_l2_protocol](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_l2_proxy/hos_l2_proxy/l2_proxy.py#L137) method of the `L2Proxy` class:

```python
  # ...

  def _parse_l2_protocol(self, protocol_data: ProtocolData):
    simulation = L2Simulation(self._device_id_to_type, protocol_data)
    simulation.run()

    if simulation.critical_error:
      self.get_logger().error(f"L2 simulation failed: {simulation.critical_error}")
      return

    self.get_logger().info("L2 simulation completed.")

  # ...
```

The usage of the simulator for command translation can be seen in the [command.py module](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_l2_proxy/hos_l2_proxy/command.py), which holds the generator functions.

## Calibration Data

Another requirement for executing L2 commands is the availablility of HAIVE4 calibration data of the delta arm. This data can be generated using the [HAIVE4 UI](https://github.com/Molcure/HAIVE4_User-Interface) to [calibrate the delta HAIVE](https://www.notion.so/molcure/HAIVE-4-1-Calibration-f48fdd0fc2ff49de90289193a43296c0?pvs=4). You can find an example file in `hos_l2_proxy/calibrations`.

To select a calibration file when launching HAIVE OS you can use the launch argument `l2_proxy_calibration_path`:

```shell
ros2 launch hos_run hos_run.launch.py l2_proxy_calibration_path:='calibrations/calib_data-220713-tip_dispo.json'
```
