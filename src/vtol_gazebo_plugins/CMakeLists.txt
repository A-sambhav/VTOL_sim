cmake_minimum_required(VERSION 3.8)

project(vtol_gazebo_plugins)

# ==========================================================
#                    COMPILER OPTIONS
# ==========================================================

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(

    -Wall
    -Wextra
    -Wpedantic

  )

endif()

# ==========================================================
#                    FIND DEPENDENCIES
# ==========================================================

find_package(ament_cmake REQUIRED)

find_package(ignition-gazebo6 REQUIRED)

find_package(ignition-plugin1 REQUIRED)

find_package(ignition-common4 REQUIRED)

find_package(ignition-math6 REQUIRED)

# ==========================================================
#                     BUILD PLUGIN
# ==========================================================

add_library(

  multicopter_motor_model SHARED

  src/multicopter_motor_model.cpp

)

# ==========================================================
#                  INCLUDE DIRECTORIES
# ==========================================================

target_include_directories(

  multicopter_motor_model

  PUBLIC

  include

)

# ==========================================================
#                    LINK LIBRARIES
# ==========================================================

target_link_libraries(

  multicopter_motor_model

  ignition-gazebo6::ignition-gazebo6

  ignition-plugin1::ignition-plugin1

  ignition-common4::ignition-common4

  ignition-math6::ignition-math6

)

# ==========================================================
#                  INSTALL PLUGIN
# ==========================================================

install(

  TARGETS multicopter_motor_model

  LIBRARY DESTINATION lib

)

# ==========================================================
#                        TESTING
# ==========================================================

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()

# ==========================================================
#                    AMENT PACKAGE
# ==========================================================

ament_package()