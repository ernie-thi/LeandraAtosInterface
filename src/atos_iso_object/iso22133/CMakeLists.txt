cmake_minimum_required(VERSION 3.10)
project(iso22133 LANGUAGES C CXX)

# Compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)

# Swig flags
option(WITH_SWIG "Enable swigging" OFF)
option(SWIG_WITH_JAVA "Swig to target-language java" OFF)
option(SWIG_WITH_PYTHON "Swig to target-language python" OFF)

if(SWIG_WITH_JAVA)
    set(SWIG_TARGET_LANG java)
elseif(SWIG_WITH_PYTHON)
    set(SWIG_TARGET_LANG python)
endif()

include(GNUInstallDirs)

# Collect all source files
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)

# Create the library
add_library(${PROJECT_NAME} SHARED
    ${CMAKE_CURRENT_SOURCE_DIR}/positioning.c
    ${CMAKE_CURRENT_SOURCE_DIR}/iso22133.c
    ${SRC_FILES}
)

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link against math library
target_link_libraries(${PROJECT_NAME} PUBLIC m)

# Set public headers
set_target_properties(${PROJECT_NAME} PROPERTIES
    PUBLIC_HEADER "iso22133.h;positioning.h;include/header.h;include/traj.h"
)

# Installation rules for the additional include directory structure
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# Installation
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    RUNTIME DESTINATION lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

# Export the targets
install(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake
)

# Create and install package config
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/iso22133Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/iso22133Config.cmake
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/iso22133Config.cmake
    DESTINATION share/${PROJECT_NAME}/cmake
)

# Export dependencies and includes for downstream packages
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()