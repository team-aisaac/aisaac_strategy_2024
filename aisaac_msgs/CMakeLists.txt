cmake_minimum_required(VERSION 3.5)
project(aisaac_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_auto REQUIRED)
# find_package(rosidl_default_generators REQUIRED)

# set(msg_files
#   "msg/AisaacStrategy.msg"
# )
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}

#   # DEPENDENCIES consai_msgs
# )

# ament_auto_find_build_dependencies()
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()
# ament_auto_generate_code()
# ament_export_dependencies(rosidl_default_runtime)


find_package(consai_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/AisaacStrategy.msg"
  "msg/AisaacReferee.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES consai_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
# ament_auto_package()