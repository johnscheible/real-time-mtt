# The name of our project is "HELLO". CMakeLists files in this project can 
# refer to the root source directory of the project as ${HELLO_SOURCE_DIR} and 
# to the root binary directory of the project as ${HELLO_BINARY_DIR}. 
cmake_minimum_required (VERSION 2.6) 
project (standalone_tracker) 

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)

set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED OFF)
find_package(Boost 1.36.0 COMPONENTS program_options REQUIRED)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVEL_STATE")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCAM_EST")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHEIGHT_PRIOR")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHORIZON_EST")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_DET_RESPONSE")
add_library(tracker src/assignmentoptimal.cpp src/target_manager.cpp src/rjmcmc_proposal.cpp src/rjmcmc_tracker.cpp src/meanshiftTracker.cpp src/PriorDist.cpp src/util.cpp)
add_library(observation src/ObservationManager.cpp src/DetectionReadinNode.cpp src/FeatTracker.cpp src/vpestimator.cpp)

add_executable(ptrack src/people_tracker.cpp)  
target_link_libraries(ptrack tracker observation)
target_link_libraries(ptrack ${OpenCV_LIBS})
target_link_libraries(ptrack ${Boost_LIBRARIES})

include_directories(include)
