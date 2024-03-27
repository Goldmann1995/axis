cmake_minimum_required(VERSION 2.8)
project(sloveaxxb)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)


file(GLOB SRC
    include/*
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Eigen
set(MIN_EIGEN_VERSION 3.2.0)
find_package(Eigen ${MIN_EIGEN_VERSION} REQUIRED)
if (EIGEN_FOUND)
  message("-- Found Eigen version ${EIGEN_VERSION}: ${EIGEN_INCLUDE_DIRS}")
endif (EIGEN_FOUND)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${glog_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)



add_library(source SHARED ${SRC})
# ## Specify libraries to link a library or executable target against
# target_link_libraries(sloveaxxb
#   ${EIGEN_LIBRARIES}
# )

# install(TARGETS sloveaxxb
#   LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#   ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
#   PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
  
# install(FILES include/*.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
# install(DIRECTORY include/ DESTINATION include/sloveaxxb FILES_MATCHING PATTERN "*.h")

add_executable(sloveaxxb src/main.cpp)

target_link_libraries(sloveaxxb  source  ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${EIGEN_LIBRARIES}  ${glog_LIBRARIES})