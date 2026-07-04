find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(test_madgwick_ahrs
  test_madgwick_ahrs.cpp
  gtest_main.cpp
  SKIP_LINKING_MAIN_LIBRARIES
)
target_link_libraries(test_madgwick_ahrs ${PROJECT_NAME}_algorithms)
