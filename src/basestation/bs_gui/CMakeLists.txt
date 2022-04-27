cmake_minimum_required(VERSION 3.5)
project(bs_gui)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rov_interfaces REQUIRED)
find_package(compressed_image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

add_executable(bs_gui src/bs_gui.cpp)
target_include_directories(bs_gui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS})
ament_target_dependencies(
  bs_gui
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "rov_interfaces"
  "compressed_image_transport"
  "cv_bridge"
)

target_link_libraries(bs_gui ${OpenCV_LIBS} ${SDL2_LIBRARIES} pthread)

install(TARGETS bs_gui
  DESTINATION lib/${PROJECT_NAME})

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
