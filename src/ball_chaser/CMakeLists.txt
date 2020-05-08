cmake_minimum_required(VERSION 2.8.3)
project(ball_chaser)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_compile_options(-std=c++11)

add_service_files(
    FILES
    DriveToTarget.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drive_bot src/drive_bot.cpp)
target_link_libraries(drive_bot ${catkin_LIBRARIES})
add_dependencies(drive_bot ball_chaser_generate_messages_cpp)

add_executable(process_image src/process_image.cpp)
target_link_libraries(process_image ${catkin_LIBRARIES})
add_dependencies(process_image ball_chaser_generate_messages_cpp)