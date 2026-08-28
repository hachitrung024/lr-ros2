cmake_minimum_required(VERSION 3.8)
project(lr_display_rviz2)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)
find_package(zed_msgs REQUIRED)

add_library(svo_playback_panel SHARED
  src/svo_playback_panel.cpp
)
target_compile_features(svo_playback_panel PUBLIC cxx_std_17)
target_include_directories(svo_playback_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(svo_playback_panel
  pluginlib
  rclcpp
  rviz_common
  std_srvs
  zed_msgs
)
target_link_libraries(svo_playback_panel Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS svo_playback_panel
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_svo_playback_logic
    test/test_svo_playback_logic.cpp
  )
  if(TARGET test_svo_playback_logic)
    target_compile_features(test_svo_playback_logic PUBLIC cxx_std_17)
    target_include_directories(test_svo_playback_logic PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(svo_playback_panel)
ament_export_dependencies(pluginlib rclcpp rviz_common std_srvs zed_msgs)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
