from sr.robot import *
from typing import Union, Tuple
from virtual_bot import VirtualBot

main_cubes = [[47, 45], [45, 46], [46, 44], [44, 47]]
secondary_cubes = [[44, 46], [47, 44], [45, 47], [46, 45]] # for extending cube range
#return_markers = [27, 6, 13, 21] initially so it didn't push first cube out of zone when going for second but now can't see start zone so drops in opponent zone
return_markers = [[27, 0], [6, 7], [13, 14], [20, 21]]
mid_term_markers = [[[23], [4]], [[2], [11]], [[9], [18]], [[16], [25]]] # for extending cube range
seek_markers = [20, 27, 6, 13]

return_marker_distance = 1.2

"""Get to cubes"""
R = VirtualBot()
iteration = 0
R.openGripper()
R.lift_motor.power = 100
"""
R.turn(-5)
R.sleep(0.5)
R.stopDriveMotors()
R.setDriveMotors(80)
R.sleep(1.3)
R.stopDriveMotors()
R.openGripper()
"""
target = main_cubes[R.zone][iteration]
return_marker = None
"""
R.seek_marker(target, -10, 10)
"""
num_primary = len(main_cubes[R.zone]) # the number of cubes the robot should be able to directly see without having to use path finding algorithms
while True and iteration < num_primary + len(secondary_cubes[R.zone]):
    # selecting cube to collect
    if iteration < num_primary:
        target = main_cubes[R.zone][iteration]
    else:
        target = secondary_cubes[R.zone][iteration - num_primary]
    # collecting cube
    while not R.have_cube:
        if iteration < num_primary:
            R.seek_marker(target)
            R.drive_to_marker(target, 0.4, 80, 0.2, 3, 5, False)
            R.turn_to_marker(target, 0.2, True)
        else:
            R.seek_markers_with_priority([target], mid_term_markers[R.zone][iteration - num_primary])
            R.drive_to_markers_with_priority([target], mid_term_markers[R.zone][iteration - num_primary], 1.4, 80, 0.2, 3, 5, False)
            R.seek_marker(target)
            R.drive_to_marker(target, 0.4, 80, 0.2, 3, 5, False)
            R.turn_to_marker(target, 0.2, True)
        R.setDriveMotors(50)
        R.sleep(0.4)
        R.stopDriveMotors()
        R.closeGripper()
    # returning after collecting cube
    if iteration < num_primary:
        R.seek_markers(return_markers[R.zone], 10, 10)
        R.drive_to_markers(return_markers[R.zone], return_marker_distance, 80, 0.2, 3, 20, False)
        R.openGripper()
        R.setDriveMotors(-40)
        R.sleep(0.2)
        R.stopDriveMotors()
    elif iteration < num_primary + len(secondary_cubes) -1:
        R.seek_markers_with_priority(return_markers[R.zone], mid_term_markers[R.zone][iteration - num_primary], 10, 10)
        R.drive_to_markers_with_priority(return_markers[R.zone], mid_term_markers[R.zone][iteration - num_primary], 1.4, 80, 0.2, 3, 20, False)
        R.seek_markers(return_markers[R.zone], 10, 10)
        R.drive_to_markers(return_markers[R.zone], return_marker_distance, 80, 0.2, 3, 20, False)
        R.openGripper()
        R.setDriveMotors(-60)
        R.sleep(0.2)
        R.stopDriveMotors()
    else:
        R.seek_markers_with_priority(return_markers[R.zone], mid_term_markers[R.zone][iteration - num_primary], 10, 10)
        R.drive_to_markers_with_priority(return_markers[R.zone], mid_term_markers[R.zone][iteration - num_primary], 1.4, 80, 0.2, 3, 20, False)
        R.seek_markers(return_markers[R.zone], 10, 10)
        R.drive_to_markers(return_markers[R.zone], return_marker_distance, 80, 0.2, 3, 20, False)
        R.openGripper()
        R.sleep(1)
    iteration += 1
