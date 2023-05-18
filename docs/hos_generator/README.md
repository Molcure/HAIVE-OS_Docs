# hos_generator

The `hos_generator` package contains scripts for automated code generation.

## hos_device_layer

The `hos_device_layer.py` module contains functions that take take information described by the HAIVE OS device database and automatically generates functions that can be used by upper layer nodes to execute device functionality.

See the [hos_run](https://github.com/Molcure/HAIVE-OS/blob/master/docs/hos_run/) documentation to see how to execute the generation script.

Let's take a look at the generated file in `hos_device_layer/gen/hos_device_api.py` and see what it does for the example of the `delta_arm_move` function:

```python
# ...
from hos_device_layer.device_manager import (create_device_api_call, send_device_api_call, DeviceAPIArg, DeviceAPICallInfo)

# ...

#
# Moves endeffector of Delta HAIVE to 3D coordinate (x, y, z) in mm
#
def delta_arm_move(node: Node, device_id: str, x: float, y: float, z: float) -> DeviceAPICallInfo:
  function_name = inspect.stack()[0][3]
  args = [
    DeviceAPIArg(
      'x',
      'float32',
      str(x),
    ),
    DeviceAPIArg(
      'y',
      'float32',
      str(y),
    ),
    DeviceAPIArg(
      'z',
      'float32',
      str(z),
    ),
  ]
  request = create_device_api_call(device_id, function_name, args)
  return send_device_api_call(node, request)

# ...
```

In every device api function we first have to pass in a reference to the calling `node` and a `device_id` (e.g. `H4001`). Next follow the actual device function arguments, in this case `x`, `y` and `z` for the cartesian coordinates of the delta arm.

The function then first looks up its name on the stack and generates a list of `DeviceAPIArg` that hold the device function arguments information. A `DeviceAPICall` request is then generated using the `create_device_api_call` function of the `hos_device_layer.device_manager` module. Finally the `DeviceAPICall` is sent by using the `send_device_api_call` function (also exposed by the `hos_device_layer.device_manager` module). To be able to track the result of the `DeviceAPICall` the function then returns a `DeviceAPICallInfo` object to the caller of the function.

See the [TODO](TODO) for an example of how to use the `hos_device_api.py` module in a node.
