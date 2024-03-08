cmake_minimum_required(VERSION 3.5)
project(gnss_interface)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-anonymous-struct -Wno-nested-anon-types )
endif()


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(tier4_system_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

find_package(Boost COMPONENTS signals)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

pkg_check_modules(PROJ REQUIRED proj)


ament_auto_add_library(gnss_interface SHARED
  src/gnss_interface/gnss_interface_node.cpp
)

target_link_libraries(gnss_interface ${PROJ_LIBRARIES})



if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(pointcloud_interface PRIVATE
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()
find_package(PCL REQUIRED COMPONENTS common)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
)


rclcpp_components_register_node(gnss_interface 
  PLUGIN "GnssInterface"
  EXECUTABLE gnss_interface_node
)


ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

