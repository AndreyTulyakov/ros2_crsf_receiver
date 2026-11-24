cmake_minimum_required(VERSION 3.20)

project(crsf_receiver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(io_context REQUIRED)
find_package(crsf_receiver_msg REQUIRED)
find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)


add_executable(crsf_receiver_node
  src/baudrate_helper.cpp
  src/crsf_receiver_node.cpp
  src/crsf_receiver.cpp
  src/crsf_parser.cpp
  src/crc8.cpp
  src/utils.cpp
)


target_include_directories(crsf_receiver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${serial_driver_INCLUDE_DIRS}
  ${io_context_INCLUDE_DIRS}
)

target_include_directories(crsf_receiver_node SYSTEM PRIVATE ${ASIO_INCLUDE_DIRS})
target_compile_definitions(crsf_receiver_node PRIVATE ${ASIO_DEFINITIONS})

target_link_libraries(crsf_receiver_node
  ${crsf_receiver_msg_TARGETS}
  rclcpp::rclcpp
  ${serial_driver_LIBRARIES}
  ${io_context_LIBRARIES}
)

install(TARGETS crsf_receiver_node 
  DESTINATION lib/${PROJECT_NAME} 
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_crsf_parser
    test/test_crsf_parser.cpp src/crsf_parser.cpp src/crc8.cpp TIMEOUT 15)
  target_include_directories(test_crsf_parser PRIVATE include)
  ament_add_gtest(test_crsf_messages test/test_crsf_messages.cpp src/utils.cpp)
  target_include_directories(test_crsf_messages PRIVATE include)
  target_link_libraries(test_crsf_messages ${crsf_receiver_msg_TARGETS})
endif()

ament_package()
