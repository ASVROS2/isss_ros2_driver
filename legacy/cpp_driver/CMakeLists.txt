cmake_minimum_required(VERSION 3.8)
project(isss_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(smarc_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ISSSStatus.msg"
  "srv/ConfigureSonar.srv"
  DEPENDENCIES std_msgs
)

add_library(isss_sonar src/isss_sonar.cpp)
target_include_directories(isss_sonar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(isss_sonar PUBLIC c_std_99 cxx_std_17)

add_executable(isss_ros2_driver_node src/isss_ros2_driver_node.cpp)
target_include_directories(isss_ros2_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(isss_ros2_driver_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(isss_ros2_driver_node
  rclcpp
  smarc_msgs
  diagnostic_msgs
  std_srvs
)
target_link_libraries(isss_ros2_driver_node isss_sonar)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(isss_ros2_driver_node "${cpp_typesupport_target}")

install(TARGETS isss_sonar isss_ros2_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS scripts/plot_isss.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(isss_sonar)
ament_export_dependencies(rclcpp smarc_msgs diagnostic_msgs std_srvs)
ament_package()
