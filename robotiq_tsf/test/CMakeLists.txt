find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_add_gtest(test_madgwick_ahrs
  test_madgwick_ahrs.cpp
  gtest_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES
)
target_link_libraries(test_madgwick_ahrs ${PROJECT_NAME}_algorithms)

ament_add_gtest(test_device_autodetect
  test_device_autodetect.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/device_autodetect.cpp
  gtest_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES
)
target_include_directories(test_device_autodetect
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)

ament_add_gtest(test_sdk_bridge
  test_sdk_bridge.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/sdk_bridge.cpp
  gtest_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES
)
target_include_directories(test_sdk_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
  ${TSF_SDK_DIR})
target_link_libraries(test_sdk_bridge ${cpp_typesupport_target})

# Unit tests for scripts/tactile_viz_node (drives the node callbacks directly;
# needs this package's generated Python messages, hence the deferred import
# path added by colcon at test time).
ament_add_pytest_test(test_tactile_viz test_tactile_viz.py TIMEOUT 120)
