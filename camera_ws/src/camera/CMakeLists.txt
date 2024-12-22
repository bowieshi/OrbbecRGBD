cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(talker
  src/cameraDatastream.cpp
)
add_executable(listener
  src/imageSaver.cpp
)
add_executable(imuReader
  src/cameraIMUStream.cpp
)



set(OrbbecSDK_DIR "/home/bowie/Desktop/OrbbecSDK")
find_package(OrbbecSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
target_link_libraries(talker
  ${OpenCV_LIBS}
  ${OrbbecSDK_LIBS}
)
target_link_libraries(imuReader
  ${OrbbecSDK_LIBS}
)
include_directories(
  include
  # ${OpenCV_INCLUDE_DIRS}
)
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${OrbbecSDK_INCLUDE_DIRS}
)
target_include_directories(imuReader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OrbbecSDK_INCLUDE_DIRS}
)


target_include_directories(listener PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(talker
  yaml-cpp
  OrbbecSDK::OrbbecSDK
  ${OpenCV_LIBS}
  # ${OrbbecSDK_LIBS}
)
target_link_libraries(imuReader
  yaml-cpp
  OrbbecSDK::OrbbecSDK
)

target_link_libraries(listener
  ${OpenCV_LIBS}
)

ament_target_dependencies(talker
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

ament_target_dependencies(listener
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

ament_target_dependencies(imuReader
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS talker listener imuReader
  DESTINATION lib/${PROJECT_NAME}
)
# install(
#   DIRECTORY lib/
#   DESTINATION lib
# )

ament_package()