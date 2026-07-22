find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

# Each gtest links the compiled libraries (poll_data_sdk_lib / _algorithms)
# rather than recompiling the sources; header include paths come with them.

# MadgwickAHRS math helpers.
ament_add_gtest(test_madgwick_ahrs test_madgwick_ahrs.cpp)
target_link_libraries(test_madgwick_ahrs ${PROJECT_NAME}_algorithms)

# Device autodetect (udev symlink / sysfs-tty descriptor scanning).
ament_add_gtest(test_device_autodetect test_device_autodetect.cpp)
target_link_libraries(test_device_autodetect poll_data_sdk_lib)

# SDK Fingers -> ROS message mapping.
ament_add_gtest(test_sdk_bridge test_sdk_bridge.cpp)
target_link_libraries(test_sdk_bridge poll_data_sdk_lib)

# AHRS fusion helpers (dt derivation + clamp, still-detection, bias trim).
ament_add_gtest(test_fusion test_fusion.cpp)
target_link_libraries(test_fusion poll_data_sdk_lib)

# Pins the node's advertised topic/service set (constructor-only; no hardware).
ament_add_gtest(test_poll_data_sdk_node test_poll_data_sdk_node.cpp)
target_link_libraries(test_poll_data_sdk_node poll_data_sdk_lib)

# Unit tests for scripts/tactile_viz_node (drives the node callbacks directly;
# needs this package's generated Python messages, hence the deferred import
# path added by colcon at test time).
ament_add_pytest_test(test_tactile_viz test_tactile_viz.py TIMEOUT 120)
