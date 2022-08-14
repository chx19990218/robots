cmake_minimum_required(VERSION 3.0.2)
project(communication_uwb)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES communication_uwb
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_receive src/receive.cpp)
target_include_directories(${PROJECT_NAME}_receive
	PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_receive
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_transmit src/transmit.cpp)
target_include_directories(${PROJECT_NAME}_transmit
	PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_transmit
	${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test src/test.cpp)
target_include_directories(${PROJECT_NAME}_test
	PUBLIC 
	${CMAKE_CURRENT_SOURCE_DIR}
	${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_test
	${catkin_LIBRARIES}
)



