add_executable(example_pick_place example_pick_place.cpp)
target_link_libraries(example_pick_place
    roboplan::roboplan
    roboplan_cartesian_planning::roboplan_cartesian_planning
    roboplan_example_models::roboplan_example_models
    roboplan_mujoco::roboplan_mujoco
)
set_property(TARGET example_pick_place PROPERTY CXX_STANDARD 20)
install(TARGETS example_pick_place DESTINATION lib/roboplan_examples)