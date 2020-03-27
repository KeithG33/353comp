import anki_vector

with anki_vector.Robot() as robot:
    robot.world.connect_cube()

    if robot.world.connected_light_cube:
        robot.behavior.dock_with_cube(robot.world.connected_light_cube)