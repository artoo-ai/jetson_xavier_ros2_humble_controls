

# Docker Build
```bash
docker build -t artoo_joy .
```

```bash
docker run -it --rm --privileged --network host artoo_joy
```

# Start Docker
```bash
docker run -it --rm --privileged --network host -v /dev:/dev --runtime nvidia --device=/dev/i2c-8 --group-add dialout --entrypoint /bin/bash ros2_humble_joy
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

```bash
ros2 topic echo /joy
```
