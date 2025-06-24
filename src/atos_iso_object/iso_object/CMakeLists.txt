cmake_minimum_required(VERSION 3.14)
project(iso_object LANGUAGES C CXX)

# Compiler settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--disable-new-dtags")

# Build options
option(BUILDING_FOR_ANDROID "Building for Android" OFF)
option(BUILD_TESTING "Enable testing" OFF)
option(WITH_SWIG "Enable swigging" OFF)
option(SWIG_WITH_JAVA "Swig to target-language java" OFF)
option(SWIG_WITH_PYTHON "Swig to target-language python" OFF)

# Required packages
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(iso22133 REQUIRED)

get_target_property(ISO22133_INCLUDE_DIRS iso22133::iso22133 INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "iso22133 include directories: ${ISO22133_INCLUDE_DIRS}")

# Add after find_package(iso22133 REQUIRED)
message(STATUS "iso22133 include dirs: ${iso22133_INCLUDE_DIRS}")
message(STATUS "iso22133 libraries: ${iso22133_LIBRARIES}")

if(SWIG_WITH_JAVA)
    set(SWIG_TARGET_LANG java)
    add_compile_definitions(WITH_SWIG)
elseif(SWIG_WITH_PYTHON)
    set(SWIG_TARGET_LANG python)
    add_compile_definitions(WITH_SWIG)
endif()

# Set variables
set(ISOOBJECT_LIBRARY ${PROJECT_NAME})
set(ISOOBJECT_TEST ${ISOOBJECT_LIBRARY}Demo)

# RPATH settings
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include(GNUInstallDirs)

# Main library target
add_library(${ISOOBJECT_LIBRARY} SHARED
    src/iso22133object.cpp
    src/iso22133state.cpp
    src/trajDecoder.cpp
)

# Include directories
target_include_directories(${ISOOBJECT_LIBRARY} PUBLIC
    # Local includes
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/sigslot/include/sigslot>
    $<INSTALL_INTERFACE:include>
)

# Link against iso22133 and inherit its include directories
target_link_libraries(${ISOOBJECT_LIBRARY} PUBLIC
    iso22133::iso22133
    atomic
    pthread
    Boost::system
)

# # Platform specific dependencies
# if(BUILDING_FOR_ANDROID)
#     add_library(boost_system SHARED IMPORTED)
#     set_target_properties(boost_system PROPERTIES IMPORTED_LOCATION
#                          ${ANDROID_BOOST_LIB_DIR}/libboost_system.so)
#     include_directories(${ANDROID_BOOST_INCLUDE_DIR})
#     target_link_libraries(${ISOOBJECT_LIBRARY}
#         iso22133::iso22133
#         atomic
#         ${boost_system}
#     )
# else()
#     target_link_libraries(${ISOOBJECT_LIBRARY} PUBLIC
#         iso22133::iso22133
#         atomic
#         pthread
#         Boost::system
#     )
# endif()

# Set public headers
set_target_properties(${ISOOBJECT_LIBRARY} PROPERTIES
    PUBLIC_HEADER "inc/iso22133object.hpp;inc/iso22133state.hpp;inc/trajDecoder.hpp;sigslot/include/sigslot/signal.hpp;inc/tcpServer.hpp;inc/udpServer.hpp"
)

# Demo executable (if not SWIG and not Android)
if(NOT WITH_SWIG AND NOT BUILDING_FOR_ANDROID)
    find_package(Boost REQUIRED COMPONENTS program_options)
    add_executable(${ISOOBJECT_TEST} demoIsoObject.cpp)
    target_include_directories(${ISOOBJECT_TEST} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/inc
    )
    target_link_libraries(${ISOOBJECT_TEST} PRIVATE
        ${ISOOBJECT_LIBRARY}
        Boost::program_options
    )
    
    # Install demo executable
    install(TARGETS ${ISOOBJECT_TEST}
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
endif()

# SWIG support
if(WITH_SWIG)
    find_package(SWIG REQUIRED)
    if(SWIG_FOUND)
        include(${SWIG_USE_FILE})
        
        if(SWIG_WITH_JAVA)
            if(NOT ${CMAKE_SYSTEM_NAME} STREQUAL "Android")
                find_package(JNI REQUIRED)
            endif()
            set(CMAKE_SWIG_FLAGS -package org.asta.iso_object)
        elseif(SWIG_WITH_PYTHON)
            find_package(Python COMPONENTS Development)
            set(CMAKE_SWIG_FLAGS "-threads")
        endif()

        set(ISOOBJECT_SWIGGED iso_object_wrap)
        set_property(SOURCE iso_object.i PROPERTY CPLUSPLUS ON)
        
        swig_add_library(${ISOOBJECT_SWIGGED}
            TYPE SHARED
            LANGUAGE ${SWIG_TARGET_LANG}
            SOURCES iso_object.i
        )
        
        target_link_libraries(${ISOOBJECT_SWIGGED}
            ${ISOOBJECT_LIBRARY}
            ${Python_LIBRARIES}
            ${JNI_LIBRARIES}
        )

        if(SWIG_WITH_PYTHON)
            execute_process(
                COMMAND python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
                OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
                OUTPUT_STRIP_TRAILING_WHITESPACE
            )
            install(TARGETS ${ISOOBJECT_SWIGGED} DESTINATION ${PYTHON_SITE_PACKAGES})
            install(FILES ${CMAKE_CURRENT_BINARY_DIR}/iso_object.py DESTINATION ${PYTHON_SITE_PACKAGES})
        endif()
    endif()
endif()

# Installation
install(
    TARGETS ${ISOOBJECT_LIBRARY}
    EXPORT export_${PROJECT_NAME}
    RUNTIME DESTINATION lib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    PUBLIC_HEADER DESTINATION include
)

# # Make sure ISO22133 is installed
# install(TARGETS ${ISO22133_LIBRARY}
#     RUNTIME DESTINATION lib
#     LIBRARY DESTINATION lib
#     ARCHIVE DESTINATION lib
#     PUBLIC_HEADER DESTINATION include
# )

# Export targets for downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${ISO22133_LIBRARY})

ament_export_dependencies(iso22133) # This ensures that any package that depends on iso_object will also automatically get the include directories and libraries from iso22133.

# Package configuration
ament_package()