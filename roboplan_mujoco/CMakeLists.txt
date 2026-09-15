cmake_minimum_required(VERSION 3.22)
project(roboplan_mujoco VERSION 0.6.1)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
  add_compile_options(/clang:-Wall /clang:-Wextra /clang:-Wpedantic)
elseif(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

if(NOT TARGET roboplan::roboplan)
  find_package(roboplan REQUIRED)
endif()
if(NOT TARGET roboplan_hardware_interface::roboplan_hardware_interface)
  find_package(roboplan_hardware_interface REQUIRED)
endif()
find_package(ament_cmake QUIET)

include(FetchContent)

find_package(mujoco 3.12.0 EXACT QUIET CONFIG)
if (NOT mujoco_FOUND)
  message(STATUS "Mujoco 3.12.0 not found, Fetching it ...")
  set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "Disable Mujoco examples" FORCE)
  set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "Disable Mujoco simulate" FORCE)
  set(MUJOCO_BUILD_STUDIO OFF CACHE BOOL "Disable Mujoco studio" FORCE)
  set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "Disable Mujoco tests" FORCE)
  set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "Disable Mujoco python utility tests" FORCE)
  set(MUJOCO_WITH_USD OFF CACHE BOOL "Disable Mujoco OpenUSD support" FORCE)
  set(MUJOCO_USE_FILAMENT OFF CACHE BOOL "Disable Mujoco filament renderer" FORCE)
  set(MUJOCO_USE_FILAMENT_MJR_COMPAT OFF CACHE BOOL "Disable Mujoco filament mjr compatibility" FORCE)
  set(MUJOCO_ENABLE_LTO OFF CACHE BOOL "Disable Mujoco LTO in dependency builds" FORCE)
  FetchContent_Declare(
    mujoco
    GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
    GIT_TAG 13827e9ee56f097f57acf69ae52b078f9839682d # v3.12.0
  )
  FetchContent_MakeAvailable(mujoco)
  if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mujoco PRIVATE -Wno-error)
  endif()
endif()

find_package(glfw3 REQUIRED CONFIG)
find_package(TinyXML2 REQUIRED)

add_library(roboplan_mujoco SHARED
  src/mujoco_hardware_interface.cpp
  src/mujoco_joint_map.cpp
  src/mujoco_kinematic_interface.cpp
  src/mujoco_model_builder.cpp
  src/mujoco_renderer.cpp
  src/mujoco_simulation.cpp
)
add_library(roboplan_mujoco::roboplan_mujoco ALIAS roboplan_mujoco)
target_include_directories(roboplan_mujoco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_mujoco PUBLIC
  roboplan::roboplan
  roboplan_hardware_interface::roboplan_hardware_interface
  mujoco::mujoco
)
target_link_libraries(roboplan_mujoco PRIVATE glfw tinyxml2::tinyxml2)
target_compile_features(roboplan_mujoco PUBLIC cxx_std_20)

install(
  TARGETS roboplan_mujoco
  EXPORT roboplan_mujoco-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)
install(
  PROGRAMS scripts/prepare_urdf.py
  DESTINATION bin
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/roboplan_mujocoConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_mujocoConfig.cmake
  INSTALL_DESTINATION lib/cmake/roboplan_mujoco
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_mujocoConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_mujocoConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_mujocoConfigVersion.cmake
  DESTINATION lib/cmake/roboplan_mujoco
)
install(
  EXPORT roboplan_mujoco-targets
  FILE roboplan_mujocoTargets.cmake
  NAMESPACE roboplan_mujoco::
  DESTINATION lib/cmake/roboplan_mujoco
)

if(ament_cmake_FOUND)
  ament_package()
endif()