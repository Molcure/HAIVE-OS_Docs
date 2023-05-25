# Making Device API Calls

This section goes shows an example node that makes use of the generated device API module found in `hos_device_layer/gen/hos_device_api.py` to make device API requests to the device manager.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

from hos_interfaces.msg import DeviceAPICallResult
import hos_device_layer.gen.hos_device_api as DeviceAPI


node_name = 'my_node'

class MyNode(Node):
  def __init__(self, layout_path: str):
    super().__init__(node_name)

    self._api_call_infos = []

    self.create_subscription(Int8, 'haive_os/my_node/move_container', lambda msg: self.move_container(msg.data), 10)
    self.create_subscription(DeviceAPICallResult, DEVICE_API_CALL_RESULT_TOPIC, self._on_device_api_call_result, 10)

  def move_container(self, distance: int):
    api_call_info = DeviceAPI.container_move(self, 'C4005', distance, 75)
    self._api_call_infos.append(api_call_info)

  def _on_device_api_call_result(self, msg: DeviceAPICallResult):
    self.get_logger().info(f"_on_device_api_call_result: task_id={msg.task_id} | success={msg.success} | error={msg.error} | result_jsons={msg.result_jsons} | request_time_s={msg.request_time_s} | response_time_s={msg.response_time_s}")

  def spin(self):
    while rclpy.ok():
      rclpy.spin_once(self)

      api_call_infos = []

      for api_call_info, device_id in self._api_call_infos:
        future = api_call_info.future

        if future.done():
          res = future.result()

          if res.is_valid:
            self.get_logger().info(f"DeviceAPICall sucess: task_id={res.task_id}")
            self._pending_tasks[res.task_id] = device_id
          else:
            self.get_logger().error(f"DeviceAPICall failed: {res.error}")

        else:
          api_call_infos.append((api_call_info, device_id))

      self._api_call_infos = api_call_infos


def main(args=None):
  rclpy.init(args=args)

  node = MyNode()

  node.get_logger().info(f"Running {node_name}")
  node.spin()

  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()

```

You can find the implementation of this node in [hos_behavior_layer/my_node.py](https://github.com/Molcure/HAIVE-OS/blob/main/src/hos_behavior_layer/hos_behavior_layer/my_node.py).

There is also a launch file available to test the node. It also starts the `Haive4Simulation` node to test the consumption of our device API request:

```shell
ros2 launch hos_run hos_my_node.launch.py
```

We can test our `move_container` functionality by publishing to the topic we setup:

```shell
ros2 topic pub -1 haive_os/my_node/move_container std_msgs/Int8 'data: 2'
```

The console output of HAIVE OS should show messages from our test node:

```
[my_node-5] [INFO] [1684999519.803129892] [my_node]: DeviceAPICall sucess: task_id=0
[my_node-5] [INFO] [1684999519.843806080] [my_node]: _on_device_api_call_result: task_id=0 | success=True | error= | result_jsons={} | request_time_s=1684999519.787117 | response_time_s=1684999519.8389845
```
