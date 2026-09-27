cmake_minimum_required(VERSION 3.0.2)
project(environments)

find_package(catkin REQUIRED)
catkin_package()

# Add this package's models/ to GAZEBO_MODEL_PATH when the workspace is sourced,
# so worlds that reference model://... resolve against custom models kept here.
catkin_add_env_hooks(60_environments SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
