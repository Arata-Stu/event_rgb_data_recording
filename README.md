# Event RGB Data Recording

## build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Arata-Stu/event_rgb_data_recording.git

cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## run recorder
```bash
ros2 launch recording_launch record.launch.xml
```

## recording trigger using ros2 service
```shell
## recording start
ros2 service call /set_recording std_srvs/srv/SetBool "{data: true}"

## recording end
ros2 service call /set_recording std_srvs/srv/SetBool "{data: false}"

```