add_executable(example_scene example_scene.cpp)
target_link_libraries(example_scene
    roboplan::roboplan
    roboplan_example_models::roboplan_example_models
)
set_property(TARGET example_scene PROPERTY CXX_STANDARD 20)
target_include_directories(example_scene PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(example_ik example_ik.cpp)
target_link_libraries(example_ik
    roboplan::roboplan
    roboplan_example_models::roboplan_example_models
    roboplan_simple_ik::roboplan_simple_ik
)
set_property(TARGET example_ik PROPERTY CXX_STANDARD 20)
target_include_directories(example_ik PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(example_oink example_oink.cpp)
target_link_libraries(example_oink
    roboplan::roboplan
    roboplan_example_models::roboplan_example_models
    roboplan_oink::roboplan_oink
)
set_property(TARGET example_oink PROPERTY CXX_STANDARD 20)
target_include_directories(example_oink PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_executable(example_rrt example_rrt.cpp)
target_link_libraries(example_rrt
    roboplan::roboplan
    roboplan_example_models::roboplan_example_models
    roboplan_rrt::roboplan_rrt
    roboplan_toppra::roboplan_toppra
)
set_property(TARGET example_rrt PROPERTY CXX_STANDARD 20)
target_include_directories(example_rrt PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

install(
  TARGETS
  example_scene
  example_ik
  example_oink
  example_rrt
  DESTINATION lib/roboplan_examples
)
