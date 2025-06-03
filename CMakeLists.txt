cmake_minimum_required(VERSION 3.8)
project(aruco_models)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Build ArUco marker generator executable  
add_executable(generate_aruco_markers src/generate_aruco_markers.cpp)
target_link_libraries(generate_aruco_markers ${OpenCV_LIBS})

# Install executables
install(TARGETS
  generate_aruco_markers
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  models/
  worlds/
  launch/
  robot_description/
  config/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
