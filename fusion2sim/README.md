# fusion2sim

```bash
python3 fusion2sim.py PATH/TO/<robot>_description [--no-controller] [--left J --right J] [--lidar LINK]
```

Converts a fusion2URDF export of a two-wheeled robot, in place, into a package
that runs in Gazebo Classic with diff_drive_controller. See the main README
for what it changes. Needs Python 3 and numpy (both come with ROS 2 Humble);
the self-check at the end needs `xacro` and `check_urdf` (source ROS 2 first).

`template/` holds the launch files, world, RViz layouts and demo scripts that
are copied into the package with the names filled in.

Tested in Gazebo on four robots: stateqbot, testbot, stateqbot scaled 2.5x
(93 kg), and stateqbot turned round (drive wheels at the front). Each one:
stands level, both wheels reach the commanded speed, drives 0.6 m with
odometry equal to Gazebo's ground truth.
