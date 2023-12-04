
# Robot State (State Space representation)
## Introduction
Robot space (can also be called State space representation) is a mandatory tool for any robotics system to make protocol dynamics but please make a difference between create a dynamic behavior and actually knowing the status of the system.

another exemple : if we add devices during runtime, we can’t change the protocol while it is running, while having robot state and sensor allow to do so, behave according to the devices status that might change for any reasons (users, errors).

![Robot State model](img/state_space_representation.png 'Robot State model')

There are two mains steps : 

- Creating an estimator:
after a firmware API call has a been done, we should update the theoretical state which can be deduced by the command ask to robot (so here we assume that the robot does exactly what he has been asked). but obviously this will sometimes not happend then we need an observer !

- Creating an observer:
this observer is sensors related, from the information get on the real world, we translate this as robot state information

(e.g. a simple rotation encoder can give more informations than just an angle such as a boolean is_open,  … or combined with an other sensor it might give something else)



if **!H4001.contain(T4001)**:

DeviceAPI.path_planning_move_container (Container in container_list where deviceID == (T4001))

DeviceAPI.pipetting(H4001, 5ml, TubeT4001.slot)



## How to update robot state
### Create new State/Update attributes
![Robot State model](img/Airtable_robot_state.png 'Robot State model')
new device state or attributes can be updated/added in the robot_state table.
- ros_state_msg : the name of the state message return by the GetStateAPI and will give acces to all its attributes
- attributes : list of attributes that will define the state of the device.
- inheritance : allow to create a inheritance pattern (for instance, most of the device will inherit from DeviceState, which means they all inherit from DeviceState attributes such as is_connected, device_id, ...)

### StateAPI and estimator
![Robot State model](img/ROS_robot_state.png 'Robot State model')
To have a proper estimator, each time a firmware API is called (API from Airtable) we need to update target devices state. Upper level Software APi will also create behaviour that will change the robot state, but most of these software API will be breaked down to several firmware APi call that why if every DeviceAPi are called along with StateAPI call, the quality of the StateAPI doesn't depend on the developper which moight forgot some attributes to updarte if they do iot manually.

Unfortunatly unlike DeviceAPI, StateAPI need to be updated by human.
Indeed after generating robot_state msg and the StateAPIGen, SetStateAPI is automatically called with DeviceAPI args

by default StateAPI calls look like this :

```python
  def state_delta_arm_move(self, node: Node, device_id: str, x: float, y: float, z: float) -> StateAPICallInfo:
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
    raise Exception("This state function has not been updated, please override this function in StateAPI Class")

```

By default if the function is not overwritten, it will throw an Execption, forcing the developper to update properly the estimator function according the args.

### How to generate StateAPI

Then we need to generate first the StateAPI library :

$ros2 launch hos_run hos_generator.launch.py

This will generate all the State ros_msg and update DeviceAPI and StateAPI files (hos_device_api.py and hos_state_api_raw). then we need to build again.

$colcon build

before build we should also check that every StateAPI are well overrided in hos_state_api.py.

### Sensors callback

Whenever a sensor send some information data directly to the HAIVE-OS, it need to trigger callback methods for some device.
Automatically, every devices referred as child device in the Airtable database, his state_updater will automatically subscribe to this sensors

```python
  def _on_device_stream(self, stream_name: str, message: object) -> None:
    device_uid = message.uid
    data = message.data
    self.get_logger().warning(f"THIS IS A DEBUG MSG: stream event from {self.node_name}>> {stream_name}:{device_uid}:{data}")

    #TODO:Ricky:Here we have to identify the stream type from UID using self.db  
    sensor_device_id = self._db.get_device_id(device_uid)
    stream = self._db.get_device_streams(sensor_device_id)
    for s in stream:
      self.get_logger().info(s.stream)
    #TODO:so the sorting/switch case should be done here ? indeed each sensor should behind trigger different call backs
    match s.stream:
      case "hall_sensor":
        pass
      case "rfid_info":
        rfid_info_callback(self, data)
      case _:
        self.get_logger().warning(f"No callback found for this stream type : {s.stream}")
    
```

In the switch case, according to the sensor type we call a certain callback. Of course you can override the _on_device_stream method so that according to the device you can change the callback function.