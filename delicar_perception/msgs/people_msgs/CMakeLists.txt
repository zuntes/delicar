cmake_minimum_required(VERSION 3.8)
project(people_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedPerson.msg"
  "msg/DetectedPersonArray.msg"
  "msg/TrackPerson.msg"
  "msg/TrackPersonArray.msg"

  DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)


ament_package()
