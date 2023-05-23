# hos_run

The `hos_run` package contains all launch files for HAIVE OS:

## hos_run.launch.py

This is the main launch file to launch the HAIVE OS. In its custom configuration it launches the following core nodes in order:

- `rosbridge_server`: A WebSocket server implementation that exposes the rosbridge_library. rosbridge provides a JSON interface to ROS, allowing any client to send JSON to publish or subscribe to ROS topics, call ROS services, and more. rosbridge supports a variety of transport layers, including WebSockets and TCP. For more details see [here](https://github.com/RobotWebTools/rosbridge_suite). This server is used by HAIVE OS clients on the hardware and software side.
- `device_manager`: Lowest layer in HAIVE OS. Handles connection of hardware devices and manages device command requests from upper layers.
- `l2_proxy`: A proxy node that can receive L2 protocols in JSON format. It translates L2 commands into HAIVE OS device command requests and executes these commands according to the [L2 specification](https://airtable.com/app56FFN9rPq29WLM/tbl8q73cJDZ7iWL6e/viwGo8UIqPMPVYSmR?blocks=hide).

There are several launch arguments you can use to customize the configuration of the launched HAIVE OS. Typing `ros2 launch hos_run hos_run.launch.py -s` will give you a list of all HAIVE OS arguments:

- `wait_rosbridge_s`: Time to wait for rosbridge server node to start. (default: `'2.0'`)
- `wait_for_device_layer_s`: Time to wait for device manager node to start. (default: `'10.0'`)
- `log_level`: Logging level (default: `'debug'`)
- `fleetname`: Identifies the data to be loaded for a specfic fleet. (default: `'shinkawasaki'`)
- `device_db`: Flag to use local copy of db data. (Allows reducing `wait_for_device_layer_s` thus increasing bootup time). Valid choices are: `['--local-db', '--no-local-db']` (default: `'--no-local-db'`)
- `haive4_simulation'`: Starts a software simulation of HAIVE4 hardware clients. (default: 'False')
- `haive4_sim_simulate_streams`: Simulates the streaming of real-time data from the hardware. Valid choices are: `['--simulate-streams', '--no-simulate-streams']` (default: `'--simulate-streams'`)
- `haive4_sim_layout_path`: Path to the data that contains the configuration of HAIVE workstations and containers. (default: `'layouts/test_layout.json'`)
- `use_l2_proxy`: Starts proxy node to control HAIVE4 hardware with L2 commands. (default: `'True'`)
- `l2_proxy_calibration_path`: Path to the calibration data needed for L2 execution using the L2 proxy node. (default: `'calibrations/calib_data-220713-tip_dispo.json'`)
- `l2_proxy_ignore_wait_commands`: Ignores execution of L2 wait commands, thus speeding up the execution. (Use for testing). Valid choices are: `['--ignore-wait', '--no-ignore-wait']` (default: `'--no-ignore-wait'`)
- `l2_proxy_protocol_file`: Name of protocol file for L2 execution using the L2 proxy node. (default: `None`)

This allows for some useful custom configurations when launching the HAIVE OS:

```shell
ros2 launch hos_run hos_run.launch.py haive4_simulation:='True' l2_proxy_protocol_file:='TT_TEST_H4001.json' l2_proxy_ignore_wait_commands:='--ignore-wait'
```

 Additionally to the core nodes this launches a software simulation node of the HAIVE4 hardware and starts executing a L2 protocol described in `TT_TEST_H4001.json` on startup. While executing the protocol, it ignores any L2 wait commands to speed up execution.

## hos_generator.launch.py

This will launch a script that will update the device API file `hos_device_layer/gen/hos_device_api.py`. You need to launch it whenever you update the device API database.
