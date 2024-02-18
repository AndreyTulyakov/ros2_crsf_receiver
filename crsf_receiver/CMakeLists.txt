cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(crsf_receiver)

set_target_properties(${TARGET} PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crsf_receiver_msg REQUIRED)
find_package(CppLinuxSerial REQUIRED)


add_executable(crsf_receiver_node 
  src/crsf_receiver_node.cpp
  src/crsf_receiver.cpp
  src/crsf_parser.cpp
  src/crc8.cpp
  src/utils.cpp
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(crsf_receiver_node CppLinuxSerial)
ament_target_dependencies(crsf_receiver_node crsf_receiver_msg rclcpp rclcpp_components std_msgs)

install(TARGETS crsf_receiver_node 
  DESTINATION lib/${PROJECT_NAME} 
)


ament_package()
