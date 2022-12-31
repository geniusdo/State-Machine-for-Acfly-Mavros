# State-Machine-for-Acfly-Mavros
a light weighted state machine fro acfly mavros based on boost::sml
**note:** you should always just read the `state-machine.hpp` file for further detailed usage. 
### USAGE  
This is a brief explanation of how to use this state machine.  
* include the `state-machine.hpp` in your `*.cpp` file  
* declare the class `drone::dependencies d`, where you can pass parameters to the state machine  
* pass a ROS handler to the state machine using `d.n = nh;` 
* initialize your state machine by `sml::sm<drone::icarus> sm{d, rate}`. Here rate is for pausing the state machine if you wanna control the time flow there.
* trigger events in the state machine, for example: `sm.process_event(drone::release{})` will make your drone on standby.

### TO DO
add trajectory planning
