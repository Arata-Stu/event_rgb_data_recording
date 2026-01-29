# Event RGB Data Recording

## build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone 

cd ~/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## recording trigger using ros2 service
```shell
## recording start
ros2 service call /set_recording std_srvs/srv/SetBool "{data: true}"

## recording end
ros2 service call /set_recording std_srvs/srv/SetBool "{data: false}"

```