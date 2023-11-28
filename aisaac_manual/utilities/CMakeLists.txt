cmake_minimum_required(VERSION 3.8)
project(utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_lint_common)
find_package(ament_lint_auto)

include_directories(include)

add_library(${PROJECT_NAME}
        src/can_utils.cpp
        src/utilities.cpp
        include/utilities/can_utils.hpp
        include/utilities/utilities.hpp
        )

find_package(nlohmann_json 3.2.0 REQUIRED)
add_library(nlohmann_json SHARED IMPORTED)

target_include_directories(utilities PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} PUBLIC)

install(TARGETS ${PROJECT_NAME}
        EXPORT "export_${PROJECT_NAME}"
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib
        INCLUDES DESTINATION include
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
        )

ament_export_include_directories(include)

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

#ament_export_libraries(${PROJECT_NAME})
ament_export_targets("export_${PROJECT_NAME}")
ament_package()