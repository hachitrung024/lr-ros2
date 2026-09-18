cmake_minimum_required(VERSION 3.8)
project(lr_display_rviz2)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
    set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
    set(ROS2_FOUND TRUE)
    #message("* Found ROS2 ${FOUND_ROS2_DISTRO}")
else()
    message("* ROS2 distro variable not set. Trying to figure it out...")
    set(ROS2_DISTROS "ardent;bouncy;crystal;dashing;eloquent;foxy;galactic;humble;iron;jazzy;kilted;lyrical;rolling")
    set(ROS2_FOUND FALSE)
    foreach(distro ${ROS2_DISTROS})
        if(NOT ROS2_FOUND)
            find_path(RCLCPP_H rclcpp.hpp PATHS /opt/ros/${distro}/include/rclcpp)
            if(RCLCPP_H)
                message("* Found ROS2 ${distro}")
                set(FOUND_ROS2_DISTRO ${distro})
                set(ROS2_FOUND TRUE)
            endif()
        endif()
    endforeach()
endif()

if(ROS2_FOUND)
    if(${FOUND_ROS2_DISTRO} STREQUAL "foxy")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_FOXY)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "humble")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_HUMBLE)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "jazzy")
        #message("* ROS2 ${FOUND_ROS2_DISTRO} is officially supported by this package.")
        add_definitions(-DFOUND_JAZZY)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "lyrical")
        add_definitions(-DFOUND_LYRICAL)
    elseif(${FOUND_ROS2_DISTRO} STREQUAL "rolling")
        add_definitions(-DFOUND_ROLLING)
    else()
        message("*** WARNING *** Unsupported ROS2 ${FOUND_ROS2_DISTRO}. '${PROJECT_NAME}' may not work correctly.")
    endif()
else()
    message("*** WARNING *** ROS2 distro is unknown. This package could not work correctly.")
endif()
################################################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)

ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  include/lr_display_rviz2/svo_control_panel.hpp
  src/svo_control_panel.cpp
)
target_include_directories(${PROJECT_NAME}_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_plugins Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_pytest_test(test_display_zed_cam_launch
    test/test_display_zed_cam_launch.py
  )
  ament_add_pytest_test(test_distributed_launch
    test/test_distributed_launch.py
  )
endif()

###############################################################################
#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE all_files ${CMAKE_CURRENT_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

###############################################################################
# INSTALL

# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz2
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
