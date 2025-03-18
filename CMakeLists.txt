
cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME eigen_example)
project(${PROJECT_NAME})

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O0 -g  -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O0 -g -march=native")

set(CMAKE_CXX_STANDARD 17)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
message("Eigen dir " ${EIGEN3_INCLUDE_DIR})
LIST(APPEND INCLUDE_DIRS
${EIGEN3_INCLUDE_DIR}
)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  tf
)

include_directories(${INCLUDE_DIRS} 	${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})
catkin_package(
  CATKIN_DEPENDS
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(${PROJECT_NAME} src/simNoisyTransform.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(conversion src/conversion.cpp)
target_link_libraries(conversion ${catkin_LIBRARIES})
