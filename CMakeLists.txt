cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(TBB REQUIRED)


include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

target_link_libraries(path_publisher ${TBB_LIBRARIES})

add_library(hello_world SHARED hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES})

add_library(model_push SHARED model_push.cc)
target_link_libraries(model_push ${GAZEBO_LIBRARIES})

add_executable(path_publisher path_publisher.cc)
target_link_libraries(path_publisher ${GAZEBO_LIBRARIES})
target_link_libraries(path_publisher tbb)


add_library(teleport_model SHARED teleport_model.cc)
target_link_libraries(teleport_model ${GAZEBO_LIBRARIES})


add_library(model_move SHARED model_move.cc)
target_link_libraries(model_move ${GAZEBO_LIBRARIES})