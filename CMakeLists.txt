cmake_minimum_required(VERSION 3.0.2)
project(swarm_ros_bridge)

add_compile_options(-std=c++14)

# 所有设备共用源码；RACER 路径按当前用户解析，也可通过 -DRACER_WORKSPACE 覆盖。
set(RACER_WORKSPACE "$ENV{HOME}/racer" CACHE PATH "RACER catkin workspace")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")
list(APPEND CMAKE_PREFIX_PATH "${RACER_WORKSPACE}/devel")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs

  message_generation
)

catkin_package(
   INCLUDE_DIRS include
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 手动添加racer的消息头文件路径（不链接库）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "${RACER_WORKSPACE}/devel/include"
)

add_executable(bridge_node src/bridge_node.cpp)

target_link_libraries(bridge_node
   ${catkin_LIBRARIES}
   zmq
   zmqpp
)


install(TARGETS bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# scripts
install(PROGRAMS
	scripts/listener.py
	scripts/talker.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
