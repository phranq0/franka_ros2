cmake_minimum_required(VERSION 3.5)
project(franka_continua)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(franka_interfaces REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.8.0 REQUIRED)

include_directories(
  include
)

# add libraries
add_library(motion_generator STATIC
  src/MOtionGenerator.cpp
)

add_library(motion_control_server STATIC
  src/MotionControlServer.cpp
)

# add dependencies
target_link_libraries(motion_generator PUBLIC Eigen3::Eigen3 Franka::Franka)
target_link_libraries(motion_control_server 
  motion_generator
  Franka::Franka
)


ament_target_dependencies(motion_control_server
  "franka_interfaces"
  "lifecycle_msgs"
  "rclcpp_lifecycle"
  "rclcpp"
)

# Build apps
set(APPS
  motion_control
)

foreach(target ${APPS})
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target}
    motion_control_server
    )
  ament_target_dependencies(${target}
    "rclcpp")
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endforeach()

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
