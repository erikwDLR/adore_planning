cmake_minimum_required(VERSION 3.8)
project(adore_planning VERSION 0.0.1 LANGUAGES CXX)

# -------------------------------------------------------------------
# C++ configuration
# -------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

# -------------------------------------------------------------------
# Dependencies
# -------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(osqp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(OpenDrive REQUIRED)
find_package(MultiAgentSolver REQUIRED)
find_package(adore_math REQUIRED)
find_package(adore_map REQUIRED)
find_package(adore_dynamics REQUIRED)
find_package(adore_controllers REQUIRED)

# -------------------------------------------------------------------
# Library
# -------------------------------------------------------------------
file(GLOB_RECURSE ADORE_PLANNING_SOURCES CONFIGURE_DEPENDS
  src/*.cpp
  src/*.c
)

add_library(${PROJECT_NAME} SHARED ${ADORE_PLANNING_SOURCES})
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    MultiAgentSolver::MultiAgentSolver
    adore_map::adore_map
    adore_math::adore_math
    adore_dynamics::adore_dynamics
    adore_controllers::adore_controllers
    # OsqpEigen / osqp linkage is likely pulled in via the sources;
    # add explicit linking here if you expose their symbols publicly.
)

# -------------------------------------------------------------------
# Tests
# -------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Library under test, used in test/CMakeLists.txt
  set(ADORE_TEST_LIB_TARGET ${PROJECT_NAME})

  add_subdirectory(test)
endif()

# -------------------------------------------------------------------
# Install & export
# -------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(EXPORT export_${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(include)
ament_export_dependencies(MultiAgentSolver adore_dynamics adore_map adore_math adore_controllers)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
