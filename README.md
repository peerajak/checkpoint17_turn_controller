# checkpoint17_turn_controller

- To do parameter testing
ros2 run turn_controller turn_controller 1.0 0.1 0.0005  2>&1 >/dev/null | grep Summary

## Simulation
source install/setup.bash; ros2 run turn_controller turn_controller

## Real robot 
source install/setup.bash; ros2 run turn_controller turn_controller 2


