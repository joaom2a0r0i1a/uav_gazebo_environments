cmake_minimum_required(VERSION 3.0.2)
project(environments)

find_package(catkin REQUIRED)
catkin_package()

# Puts this package on the gazebo search paths for shells that run gzserver without roslaunch.
catkin_add_env_hooks(gazebo_paths SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

install(DIRECTORY worlds models scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
