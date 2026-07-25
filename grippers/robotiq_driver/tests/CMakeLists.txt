# Add support for GTest.
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ament_lint_auto REQUIRED)

ament_lint_auto_find_test_dependencies()

# GMock throws an error if we don't switch off this option in tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(-Wno-sign-conversion)
endif()

###############################################################################
# test_robotiq_gripper_hardware_interface

ament_add_gmock(test_robotiq_gripper_hardware_interface
    test_robotiq_gripper_hardware_interface.cpp
)
target_include_directories(test_robotiq_gripper_hardware_interface
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(
    test_robotiq_gripper_hardware_interface
    robotiq_driver
    hardware_interface::hardware_interface
)

###############################################################################
# robotiq_driver_tests

# One gmock binary for the whole package, so the suite runs — and debugs — as a
# single target. Pick a subset with --gtest_filter.
ament_add_gmock(${PROJECT_NAME}_tests
    test_gripper_scaling.cpp      # register <-> joint arithmetic
    test_hardware_parameters.cpp  # HardwareInfo -> ConnectionConfig
    test_rclcpp_logger.cpp        # SDK log sink -> /rosout
)
target_include_directories(${PROJECT_NAME}_tests
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(${PROJECT_NAME}_tests robotiq_driver)
