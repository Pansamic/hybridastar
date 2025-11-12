cmake_minimum_required(VERSION 3.10)

project(HybridAStar
    VERSION 1.0.0
    LANGUAGES C CXX
)

option(BUILD_TESTS "Build tests" ON)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug Release MinSizeRel RelWithDebInfo." FORCE)
endif()

find_package(Eigen3 3.3 REQUIRED)

# Changed from INTERFACE to STATIC to actually compile the source files
add_library(HybridAStar STATIC
    src/a_star_planner.cpp
    src/dubins_curve.cpp
    src/hybrid_a_star_planner.cpp
    src/kdtree.cpp
)

target_include_directories(HybridAStar PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

if (BUILD_TESTS)
    add_executable(test_empty_map tests/test_empty_map.cpp)
    target_include_directories(test_empty_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(test_empty_map PRIVATE HybridAStar)
    
    add_executable(test_obstacle_map tests/test_obstacle_map.cpp)
    target_include_directories(test_obstacle_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(test_obstacle_map PRIVATE HybridAStar)
    
    add_executable(test_a_star tests/test_a_star.cpp)
    target_include_directories(test_a_star PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(test_a_star PRIVATE HybridAStar)
    
    add_executable(test_a_star_save tests/test_a_star_save.cpp)
    target_include_directories(test_a_star_save PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(test_a_star_save PRIVATE HybridAStar)
endif()