cmake_minimum_required(VERSION 3.0.2)
project(ins_pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  message_filters
  visualization_msgs
  sensor_msgs
)

add_message_files(
  FILES
  ins622.msg
)

generate_messages(
               DEPENDENCIES
               std_msgs
)


catkin_package(
               INCLUDE_DIRS include
               CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime
)

#——————————————————————————————————该指令的作用主要是指定要链接的动/静态库,库文件的路径————————————————————————————————————
link_directories(
					  /usr/local/lib 
					  ./library
)


#——————————————————————————————————相关头文件路径————————————————————————————————————
include_directories(
                   ${catkin_INCLUDE_DIRS}
                   ./include
                   usr/local/include     
)

#——————————————————————————————————可执行程序————————————————————————————————————
add_executable(ins_data
               ./src/ins_data.cpp

)

target_link_libraries(ins_data
					      ${catkin_LIBRARIES}
					      libserial.so
)

add_dependencies(ins_data data_transform
 					 ${ins_pkg_EXPORTED_TARGETS}
	             ${catkin_EXPORTED_TARGETS}
)
add_executable(data_transform src/data_transform.cpp)
target_link_libraries(data_transform ${catkin_LIBRARIES})

add_executable(gps_path src/gps_path.cpp) 
target_link_libraries(gps_path ${catkin_LIBRARIES})






















