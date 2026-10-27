# Fusion 360 + ROS 2 workshop: from CAD export to a driving robot

You export a two-wheeled robot from Fusion 360 with fusion2URDF. That gives a
ROS 2 package (`<robot>_description`) that RViz can show but Gazebo cannot
simulate yet. This repository has two things:

| Folder | What it is |
|---|---|
| `fusion2sim/` | a script that turns **your** export into a package you can simulate and drive |
| `stateqbot_description/` | the workshop robot, already converted and tested: use it for the experiments |

## Get it

```bash
cd ~/Downloads
git clone https://github.com/zorx01/testbot-ros2-workshop.git
```

## 1. Look at your export in RViz (Day 4)

```bash
cp -r ~/Downloads/stateqbot_description ~/Documents/test_ws/src/   # your export
python3 ~/Downloads/testbot-ros2-workshop/fusion2sim/fusion2sim.py \
        ~/Documents/test_ws/src/stateqbot_description --no-controller
cd ~/Documents/test_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch stateqbot_description display.launch.py     # move the joints with the sliders
ros2 launch stateqbot_description sim.launch.py rviz:=true   # Gazebo
```

The script prints what it measured (wheel radius, wheel separation, caster
position) and what it changed. Check those numbers against your CAD.

What it fixes, all measured from your meshes, so any size of robot works:

- the two wheel joints are found (child link named like `wheel`, or
  `--left NAME --right NAME`); wheel radius, width and separation come from
  the wheel meshes
- a mirrored wheel axis is flipped, so a forward command drives forward
- joints with the same name as a link get `_joint` added (Gazebo refuses
  the duplicate)
- collisions: cylinders for the wheels, boxes for the other parts, and a
  frictionless sphere where the body touches the ground (the caster), at
  exactly wheel-bottom height so the robot stands level
- wheel friction, contact stiffness scaled to the robot's weight
- a 2D LiDAR (on the link named like `lidar`, or a new one on top), an IMU,
  a world with obstacles, and launch files for RViz and Gazebo

Your original export is kept in `fusion_export_backup/`. Running the script
again always starts from that backup.

## 2. Add the controller (Day 5)

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control
python3 ~/Downloads/testbot-ros2-workshop/fusion2sim/fusion2sim.py \
        ~/Documents/test_ws/src/stateqbot_description
cd ~/Documents/test_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch stateqbot_description sim.launch.py rviz:=true
ros2 topic pub --rate 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist "{linear: {x: 0.15}}"
```

Without `--no-controller` the script also writes
`urdf/<robot>.ros2_control.xacro` and `config/ros2_controllers.yaml`. The
launch files see them and start `joint_state_broadcaster` and
`diff_drive_controller`.

## 3. The complete package

`stateqbot_description/` is the result of step 2, tested in Gazebo. Use it
for the experiments so everyone has the same robot. Only one package with
that name may be in `src/`, so move your own out first:

```bash
mv ~/Documents/test_ws/src/stateqbot_description ~/Documents/my_export
cp -r ~/Downloads/testbot-ros2-workshop/stateqbot_description ~/Documents/test_ws/src/
cd ~/Documents/test_ws && colcon build --symlink-install && source install/setup.bash
```

## Limits of the script

Two driven wheels plus casters only (no skid-steer, tracks or mecanum).
The CAD must be x forward, z up, with the wheel axles along y, and the meshes
in metres; the script stops with a message if the axles are not along y.
