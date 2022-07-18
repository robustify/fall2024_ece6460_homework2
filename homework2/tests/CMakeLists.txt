find_package(rostest REQUIRED)

add_rostest_gtest(test_homework2
  homework2.test
  homework2_test.cpp
)
add_dependencies(test_homework2
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_homework2
  ${catkin_LIBRARIES}
)

