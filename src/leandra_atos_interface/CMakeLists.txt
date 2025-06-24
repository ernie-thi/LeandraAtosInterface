cmake_minimum_required(VERSION 3.8)
project(leandra_atos_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(LEANDRA_ATOS_INTERFACE_NODE_SRC src/leandra_atos_interface_node.cpp)
set(LEANDRA_ATOS_INTERFACE_SRC src/leandra_atos_interface.cpp)

set(LEANDRA_ATOS_INTERFACE_NODE_HEADERS include/${PROJECT_NAME}/leandra_atos_interface_node.hpp)
set(LEANDRA_ATOS_INTERFACE_HEADERS include/${PROJECT_NAME}/leandra_atos_interface.hpp)

set(LEANDRA_ATOS_INTERFACE_NODE_LIB leandra_atos_interface_node)

ament_auto_add_library(${LEANDRA_ATOS_INTERFACE_NODE_LIB} SHARED 
  ${LEANDRA_ATOS_INTERFACE_NODE_SRC}
  ${LEANDRA_ATOS_INTERFACE_SRC}
  ${LEANDRA_ATOS_INTERFACE_NODE_HEADERS}
  ${LEANDRA_ATOS_INTERFACE_HEADERS}
)

target_include_directories(leandra_atos_interface_node PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_compile_definitions(leandra_atos_interface_node PRIVATE ${GeographicLib_DEFINITIONS})
target_link_libraries     (leandra_atos_interface_node         ${GeographicLib_LIBRARIES})

set(NODE_NAME ${LEANDRA_ATOS_INTERFACE_NODE_LIB}_exe)

rclcpp_components_register_node(${LEANDRA_ATOS_INTERFACE_NODE_LIB}
  PLUGIN "LeandraAtosInterface::LeandraAtosInterfaceNode"
  EXECUTABLE ${NODE_NAME}
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

#ament_package()
ament_auto_package(INSTALL_TO_SHARE launch)