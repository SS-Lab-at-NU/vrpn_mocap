# vrpn_mocap

![rolling](https://github.com/alvinsunyixiao/vrpn_mocap/actions/workflows/rolling.yml/badge.svg)
![jazzy](https://github.com/alvinsunyixiao/vrpn_mocap/actions/workflows/jazzy.yml/badge.svg)
![iron](https://github.com/alvinsunyixiao/vrpn_mocap/actions/workflows/iron.yml/badge.svg)
![humble](https://github.com/alvinsunyixiao/vrpn_mocap/actions/workflows/humble.yml/badge.svg)
![foxy](https://github.com/alvinsunyixiao/vrpn_mocap/actions/workflows/foxy.yml/badge.svg)

ROS2 [VRPN](https://github.com/vrpn/vrpn) client built pirmarily to interface
with motion capture devices such as VICON and OptiTrack. A detailed list of
supported hardware can be found on
[vrpn wiki](https://github.com/vrpn/vrpn/wiki/Available-hardware-devices).

### Dependencies

`sudo apt-get install netbase`

If using inside Docker container, run with `--network=host`

### Build From Source

1. Clone this repo into your ROS2 workspace
2. Run `rosdep install --from-paths src -y --ignore-src` to install dependencies
3. Run `colcon build`

## Usage

### Launch Default Configuration from Command Line
Run the following command,
```bash
ros2 launch vrpn_mocap client.launch.py server:=<server_ip> port:=<port>
```
replacing `<server_ip>` and `<port>` with your VRPN server ip and port, e.g.
```bash
ros2 launch vrpn_mocap client.launch.py server:=192.168.0.4 port:=3883
```
Then with `ros2 topic list`, you should be able to see the following topics
```bash
/vrpn_mocap/<tracker_name>/pose
```
where `<tracker_name>` is usually the name of your tracked objects.

### Customized Launch
Check out the default [parameter file](config/client.yaml) and
[launch file](launch/client.launch.py). You can then write your own launch
file with custom configurations.

### Parameters

#### `client.launch.py`
- `server (string)` -- server name, either ip address or domain name (default: `"<optitrack_ip>"`)
- `port (int)` -- VRPN server port (default: `3883`)

#### `client.yaml`
- `frame_id (string)` -- frame name of the fixed world frame (default: `"world"`)
- `update_freq (double)` -- frequency of the motion capture data publisher (default: `100.`)
- `refresh_freq (double)` -- frequency of dynamic adding new tracked objects (default: `1.`)
- `sensor_data_qos` -- use best effort QoS for VRPN data stream, set to false to use
  system default QoS which is reliable (default: `true`)
- `multi_sensor (bool)` -- set to true if there are more than one sensor (frame) reporting on
  the same object (default: `false`)
- `use_vrpn_timestamps (bool)` -- use timestamps coming from VRPN. This ensures that the interval between frames timestamps is not modified (default: `false`)

### Acknowledgement
Some ideas are borrowed from the well known
[vrpn\_client\_ros](https://github.com/ros-drivers/vrpn_client_ros) -- a ROS1
package for interfacing with VRPN devices. I thank the authors and developers
for delivering and maintaining the original implementation.
