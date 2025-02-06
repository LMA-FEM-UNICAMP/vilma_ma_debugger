# vilma_ma_debugger

## Simulating topics during development

```shell
ros2 topic pub /vilma_ma_ros/sensors_ma std_msgs/msg/Float64MultiArray "{data: $(python3 -c 'import random; print([random.randint(0, 1000) for _ in range(31)])')}"

ros2 topic pub /vilma_ma_ros/state_ma std_msgs/msg/Float64MultiArray "{data: $(python3 -c 'import random; print([random.randint(0, 100) for _ in range(15)])')}"

ros2 topic pub /vilma_ma_ros/joystick_ma std_msgs/msg/Float64MultiArray "{data: $(python3 -c 'import random; print([random.randint(0, 10) for _ in range(10)])')}"
```