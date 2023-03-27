project(ROBOT_SIM C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
include_directories(${ros_app_MISSION_DIR}/fsw/platform_inc)


# Create the app module
add_cfe_app(robot_sim fsw/src/robot_sim.c)
target_link_libraries(robot_sim m)
