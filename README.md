# testbot: files for the Fusion 360 + ROS 2 workshop

You export the robot from Fusion 360 with fusion2URDF. That gives you a ROS 2
package called `testbot_description`. These files turn that export into a
robot you can simulate and drive.

Two ways to use this repository:

- **Follow the steps** with your own export: `simulation_files/` on Day 4,
  `controller_files/` on Day 5 (sections 1 and 2 below).
- **Use the complete package** `testbot_description/`: the finished, tested
  robot with simulation and controllers already in it (section 3).

## Get the files

```bash
cd ~/Downloads
git clone https://github.com/zorx01/testbot-ros2-workshop.git
```

## 1. Simulation files (Day 4)

Copy everything in `simulation_files/` into your exported package. Files with
the same name are replaced; that is intended.

```bash
cp -r ~/Downloads/testbot-ros2-workshop/simulation_files/. \
      ~/Documents/test_ws/src/testbot_description/
cd ~/Documents/test_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch testbot_description display.launch.py   # RViz, wheel sliders
ros2 launch testbot_description sim.launch.py       # Gazebo
```

What lands where:

| File | Goes to | What it does |
|---|---|---|
| `launch/rsp.launch.py` | `launch/` | publishes the robot description and TF |
| `launch/display.launch.py` | `launch/` (replaces the exported one) | RViz with joint sliders |
| `launch/sim.launch.py` | `launch/` | Gazebo + spawn the robot |
| `urdf/testbot.urdf.xacro` | `urdf/` (replaces) | entry point; adds the sim parts |
| `urdf/assemblies/testbot.urdf.xacro` | `urdf/assemblies/` (replaces) | the exported robot, corrected for simulation |
| `urdf/testbot.gazebo.xacro` | `urdf/` | wheel friction, Gazebo plugin |
| `urdf/testbot.sensors.xacro` | `urdf/` | LiDAR and IMU |
| `worlds/obstacles.world` | `worlds/` | the test world |
| `rviz/*.rviz` | `rviz/` | RViz layouts |
| `scripts/*.py` | `scripts/` | drive and scan demos |
| `CMakeLists.txt`, `package.xml` | package root (replace) | install the new folders, list dependencies |

## 2. Controller files (Day 5)

Install the control packages once:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control
```

Then add the two controller files and rebuild:

```bash
cp -r ~/Downloads/testbot-ros2-workshop/controller_files/. \
      ~/Documents/test_ws/src/testbot_description/
cd ~/Documents/test_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch testbot_description sim.launch.py
```

| File | Goes to | What it does |
|---|---|---|
| `urdf/testbot.ros2_control.xacro` | `urdf/` | tells ros2_control which joints it drives |
| `config/ros2_controllers.yaml` | `config/` (replaces) | `joint_state_broadcaster` + `diff_drive_controller` |

The launch files notice the new files by themselves: with them, `sim.launch.py`
starts the controllers; without them, it does not.

Drive it:

```bash
ros2 topic pub --rate 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
```

## 3. The complete package

Everything from sections 1 and 2, already put together. Use it to run the
experiments, or to compare against your own package.

```bash
cd ~/Downloads
git clone https://github.com/zorx01/testbot-ros2-workshop.git
cp -r ~/Downloads/testbot-ros2-workshop/testbot_description ~/Documents/test_ws/src/
cd ~/Documents/test_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch testbot_description sim.launch.py rviz:=true
```

Only one `testbot_description` may be in `src/` at a time. If your own
export is there, move it out first (for example to `~/Documents/my_export`).
