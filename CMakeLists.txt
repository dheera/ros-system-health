cmake_minimum_required(VERSION 2.8.3)
project(system_health)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  topic_tools
)

catkin_package(
      INCLUDE_DIRS include
    #  LIBRARIES wheel_odometry
    #  CATKIN_DEPENDS roscpp rospy std_msgs
    #  DEPENDS system_lib
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_library(pslib STATIC include/cpslib/pslib.h include/cpslib/pslib_linux.c include/cpslib/common.h include/cpslib/common.c)

add_executable(node_watcher src/node_watcher.cpp)
add_dependencies(node_watcher ${catkin_EXPORTED_TARGETS})
add_executable(system_watcher src/system_watcher.cpp)
add_dependencies(system_watcher ${catkin_EXPORTED_TARGETS})
add_executable(topic_watcher src/topic_watcher.cpp)
add_dependencies(topic_watcher ${catkin_EXPORTED_TARGETS})

target_link_libraries(node_watcher ${catkin_LIBRARIES})
target_link_libraries(system_watcher pslib ${catkin_LIBRARIES})
target_link_libraries(topic_watcher ${catkin_LIBRARIES})

install(TARGETS
    node_watcher
    system_watcher
    topic_watcher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

