cmake_minimum_required(VERSION 3.16)
project(rubik CXX)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)
#find_package(yaml-cpp REQUIRED)
#add_subdirectory(./graph)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )


set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")

# add_executable(test common.cpp testMain.cpp rubik13.cpp labp.cpp mp.cpp umapf.cpp mapf_post.cpp rubik.cpp rubik12.cpp)

# #target_include_directories(test "./")
# target_compile_features(test PUBLIC cxx_std_17)
# target_link_libraries(test yaml-cpp ${Boost_LIBRARIES})

# add_executable(main common.cpp main.cpp rubik13.cpp labp.cpp mp.cpp umapf.cpp rubik.cpp rubik12.cpp)
# target_compile_features(main PUBLIC cxx_std_17)
# target_link_libraries(main yaml-cpp ${Boost_LIBRARIES})


add_executable(main common.cpp labp.cpp main.cpp mp3d.cpp rth2d.cpp rubik3d.cpp umapf3d.cpp)
target_compile_features(main PUBLIC cxx_std_17)
target_link_libraries(main yaml-cpp ${Boost_LIBRARIES})
# add_executable(test_ir ir_test.cpp)
# #include_directories(${YAML_CPP_INCLUDE_DIRS})
# target_compile_features(test_ir PUBLIC cxx_std_17)
# target_link_libraries(test_ir lib-mapf yaml-cpp)

#add_executable(test_main ./tests/test_main.cpp)
#target_compile_features(app PUBLIC cxx_std_17)
#target_link_libraries(test_main lib-mapf )


# format


# test

# set(TEST_MAIN_FUNC ./googletest/googletest/src/gtest_main.cc)
# set(TEST_ALL_SRC ${TEST_MAIN_FUNC})
# macro(add_test name target)
#   add_executable(${name} ${target} ${TEST_MAIN_FUNC})
#   target_link_libraries(${name} lib-mapf lib-graph gtest)
#   list(APPEND TEST_ALL_SRC ${target})
# endmacro(add_test)
#add_test(test_graph ./tests/test_graph.cpp)