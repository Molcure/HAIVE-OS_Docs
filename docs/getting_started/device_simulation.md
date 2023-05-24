# Using The `hos_device_simulation` Package

The `hos_device_simulation` package makes it possible to simulatae clients that are defined in the HAIVE OS device database. To start HAIVE OS with the device simulation enabled, we need to run the `hos_run` launch file with the `haive4_simulation` flag enabled:

```shell
ros2 launch hos_run hos_run.launch.py haive4_simulation:='True'
```

This will simulate a HAIVE4 configuration that is described in the layout file found in `hos_device_simulation/layouts/test_layout.json`. This file defines the HAIVE4 as it is found in the office.

To execute a protocol, we can send a request in a separate shell instance:

```shell
ros2 topic pub -1 /haive_os/load_protocol std_msgs/String 'data: test_protocol.json'
```

This simulates the execution of the `Kusakabe-Challenge` protocol. You can launch the execution of the protocol with wait times disabled in a single command when launching the system:

```shell
ros2 launch hos_run hos_run.launch.py haive4_simulation:='True' l2_proxy_protocol_file:='test_protocol.json' l2_proxy_ignore_wait_commands:='--ignore-wait'
```
