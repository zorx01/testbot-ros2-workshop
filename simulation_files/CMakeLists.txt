cmake_minimum_required(VERSION 3.8)
project(testbot_description)

find_package(ament_cmake REQUIRED)

# Install package data.
# MANUAL EDIT (not from fusion2URDF): added `worlds` to this list.
# sim.launch.py resolves worlds/obstacles.world through the SHARE
# directory, so without this the launch file works from a source tree and
# fails from an installed one.
install(DIRECTORY urdf meshes launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

# Docs and images, so the README renders from the install tree too.
install(DIRECTORY docs images
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(FILES robot_data.yaml README.md
  DESTINATION share/${PROJECT_NAME}
)

# MANUAL EDIT (not from fusion2URDF): executable demo scripts.
# These go to lib/${PROJECT_NAME}, NOT share/, because that is the only
# place `ros2 run <pkg> <script>` looks. install(PROGRAMS ...) rather than
# install(FILES ...) so the executable bit survives.
install(PROGRAMS
  scripts/drive_demo.py
  scripts/scan_reader.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
