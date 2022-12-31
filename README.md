# State-Machine-for-Acfly-Mavros
a light weighted state machine for acfly mavros based on boost::sml  
**note:** you should always just read the `state-machine.hpp` file for further detailed usage. 
### USAGE  
This is a brief explanation of how to use this state machine.  
* include the `state-machine.hpp` in your `*.cpp` file  
* declare the class `drone::dependencies d`, where you can pass parameters to the state machine  
* pass a ROS handler to the state machine using `d.n = nh` 
* initialize your state machine by `sml::sm<drone::icarus> sm{d, rate}`. Here `rate` is for pausing the state machine if you wanna control the time flow there.
* trigger events in the state machine, for example: `sm.process_event(drone::release{})` will make your drone on standby.

### TO DO
add trajectory planning

### 用法
下面是有关如何使用该状态机的内容
* 在你的`*.cpp`文件里include `state-machine.hpp`
* 声明一个 `drone::dependencies d`, 用来传递参数给状态机
* 传一个ROS句柄给状态机，使用`d.n = nh`
* 初始化一个状态机，使用 `sml::sm<drone::icarus> sm{d, rate}`。这里 `rate` 是用来控制状态机内的ROS时间的。
* 触发事件。例如, `sm.process_event(drone::release{})` 会使你的无人机进入准备状态。

### TO DO
添加轨迹规划
