cmake_minimum_required(VERSION 2.8)
project(unreal_camera_info_pub)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport sensor_msgs roscpp camera_info_manager)

catkin_package(CATKIN_DEPENDS cv_bridge image_transport sensor_msgs camera_info_manager)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the publisher example
add_executable(my_publisher src/my_publisher.cpp)
add_dependencies(my_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(my_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber example
add_executable(my_subscriber src/my_subscriber.cpp)
add_dependencies(my_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the cv editing example
add_executable(image_converter src/image_converter.cpp)
add_dependencies(image_converter ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(image_converter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the camera info publisher
add_executable(camera_info_publisher src/camera_info_publisher.cpp)
add_dependencies(camera_info_publisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(camera_info_publisher ${catkin_LIBRARIES})
# ${OpenCV_LIBRARIES})

# camera info publisher rgbd
add_executable(camera_info_publisher_rgbd src/camera_info_publisher_rgbd.cpp)
add_dependencies(camera_info_publisher_rgbd ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(camera_info_publisher_rgbd ${catkin_LIBRARIES})


# Mark executables and/or libraries for installation
install(TARGETS my_publisher my_subscriber image_converter camera_info_publisher camera_info_publisher_rgbd
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

