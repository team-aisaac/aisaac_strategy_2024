cmake_minimum_required(VERSION 3.8)
project(manual_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(manual_node_component SHARED
        src/manual_node.cpp)

rclcpp_components_register_node(manual_node_component
        PLUGIN "manual_node::ManualNode"
        EXECUTABLE manual_node_exec)


add_backward(manual_node_component)

install(TARGETS
        manual_node_component
        DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)


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

ament_auto_package()
