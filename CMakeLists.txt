cmake_minimum_required(VERSION 3.10)

project(HybridAStar
    VERSION 1.0.0
    LANGUAGES C CXX
)

option(BUILD_TESTS "Build tests" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug Release MinSizeRel RelWithDebInfo." FORCE)
endif()

find_package(Eigen3 3.3 REQUIRED)

set(HYBRIDASTAR_HEADERS
    include/hybridastar/a_star_planner.h
    include/hybridastar/dubins_curve.h
    include/hybridastar/hybrid_a_star_planner.h
    include/hybridastar/kdtree.hpp
)

set(HYBRIDASTAR_SOURCES
    src/a_star_planner.cpp
    src/dubins_curve.cpp
    src/hybrid_a_star_planner.cpp
    src/kdtree.cpp
)

# Changed from INTERFACE to STATIC to actually compile the source files
if (BUILD_SHARED_LIBS)
    add_library(${PROJECT_NAME} SHARED ${HYBRIDASTAR_SOURCES})
else()
    add_library(${PROJECT_NAME} STATIC ${HYBRIDASTAR_SOURCES})
endif()

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${EIGEN_INCLUDE_DIRS}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

if (BUILD_TESTS)
    add_executable(test_empty_map tests/test_empty_map.cpp)
    target_include_directories(test_empty_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(test_empty_map PRIVATE ${PROJECT_NAME})

    add_executable(test_obstacle_map tests/test_obstacle_map.cpp)
    target_include_directories(test_obstacle_map PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(test_obstacle_map PRIVATE ${PROJECT_NAME})

    add_executable(test_a_star tests/test_a_star.cpp)
    target_include_directories(test_a_star PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(test_a_star PRIVATE ${PROJECT_NAME})

    add_executable(test_a_star_save tests/test_a_star_save.cpp)
    target_include_directories(test_a_star_save PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
    target_link_libraries(test_a_star_save PRIVATE ${PROJECT_NAME})

    install(TARGETS test_empty_map test_obstacle_map test_a_star test_a_star_save
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Install targets with export
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install headers
install(FILES ${HYBRIDASTAR_HEADERS} 
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hybridastar
)

# Export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Generate the version file for the config file
write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)

# Configure the config file
configure_package_config_file(
    "cmake/${PROJECT_NAME}Config.cmake.in"
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# Install config files
install(FILES
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)