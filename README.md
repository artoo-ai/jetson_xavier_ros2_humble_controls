

# Docker Build
```bash
docker build -t artoo_controls .
```

```bash
docker run -it --rm --privileged --network host artoo_controls
```

# Start Docker running Entrypoint File
```bash
docker run -it --rm --privileged --network host -v /dev:/dev --runtime nvidia --device=/dev/i2c-8 --group-add dialout  artoo_controls
```

# Start Docker In Bash
```bash
docker run -it --rm --privileged --network host -v /dev:/dev --runtime nvidia --device=/dev/i2c-8 --group-add dialout --entrypoint /bin/bash artoo_controls
```


Inside the container
Keyboard
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Joy
```bash
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
```
Configs: https://github.com/ros-teleop/teleop_twist_joy/blob/indigo-devel/config/ps3.config.yaml
xbox is working with Nintendo Switch Remote

PWM PCA9685
```bash
ros2 launch pwm_pca9685 esc_diff_drive.launch.py
```

```bash
ros2 topic echo /joy
```

```bash
ros2 pkg list
```

```bash
ros2 pkg prefix pwm_pca9685
```