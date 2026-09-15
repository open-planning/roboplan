cmake_minimum_required(VERSION 3.22)
project(roboplan_hardware_interface VERSION 0.6.1)

if(NOT TARGET roboplan::roboplan)
  find_package(roboplan REQUIRED)
endif()
find_package(ament_cmake QUIET)

add_library(roboplan_hardware_interface INTERFACE)
add_library(
  roboplan_hardware_interface::roboplan_hardware_interface
  ALIAS roboplan_hardware_interface)
target_include_directories(roboplan_hardware_interface INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_hardware_interface INTERFACE roboplan::roboplan)
target_compile_features(roboplan_hardware_interface INTERFACE cxx_std_20)

install(
    TARGETS roboplan_hardware_interface
    EXPORT roboplan_hardware_interface-targets
    INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_hardware_interfaceConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_hardware_interfaceConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_hardware_interface
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_hardware_interfaceConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/roboplan_hardware_interfaceConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/roboplan_hardware_interfaceConfigVersion.cmake
    DESTINATION lib/cmake/roboplan_hardware_interface
)
install(
    EXPORT roboplan_hardware_interface-targets
    FILE roboplan_hardware_interfaceTargets.cmake
    NAMESPACE roboplan_hardware_interface::
    DESTINATION lib/cmake/roboplan_hardware_interface
)

if (ament_cmake_FOUND)
    ament_package()
endif()