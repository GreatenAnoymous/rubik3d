cmake_minimum_required(VERSION 3.16)
project(rubik CXX)

#find_package(yaml-cpp REQUIRED)
#add_subdirectory(./graph)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )


# set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")



add_executable(main common.cpp labp.cpp main.cpp mp3d.cpp rth2d.cpp rubik3d.cpp umapf3d.cpp search.cpp formation.cpp)
target_compile_features(main PUBLIC cxx_std_17)
target_link_libraries(main  ${Boost_LIBRARIES} Threads::Threads /usr/local/lib/libortools.so)
