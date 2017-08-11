cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(my_test_pcl)
find_package(PCL 1.2 REQUIRED)
set(BOOST_LIBS program_options serialization random system)
find_package(Boost 1.58.0 COMPONENTS ${BOOST_LIBS} REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(kitti2pcd test_kitti2pcd.cpp)
target_link_libraries(kitti2pcd ${Boost_LIBRARIES} ${PCL_LIBRARIES})
