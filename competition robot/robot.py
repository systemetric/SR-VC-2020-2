from sr.robot import *
from typing import Union, Tuple
from virtual_bot import VirtualBot

main_cubes = [[47, 45], [45, 46], [46, 44], [44, 47]]
secondary_cubes = [[44, 46], [47, 44], [45, 47], [46, 45]] # for extending cube range
#return_markers = [27, 6, 13, 21] initially so it didn't push first cube out of zone when going for second but now can't see start zone so drops in opponent zone
return_markers = [[27, 0], [6, 7], [13, 14], [20, 21]]
mid_term_markers = [[[24, 25], [2, 3]], [[3, 4], [9, 10]], [[10, 11], [16, 17]], [[17, 18], [23, 24]]] # for extending cube range
seek_markers = [20, 27, 6, 13]

"""Get to cubes"""
R = VirtualBot()
iteration = 0
R.openGripper()
R.lift_motor.power = 100
R.turn(10)
R.sleep(0.1)
R.stopDriveMotors()
R.setDriveMotors(80)
R.sleep(1.3)
R.stopDriveMotors()
R.openGripper()
target = main_cubes[R.zone][iteration]
return_marker = None
R.seek_marker(target, 10, -10)
while True and iteration < len(main_cubes[R.zone]):
    target = main_cubes[R.zone][iteration]
    while not R.have_cube:
        R.seek_marker(target)
        R.drive_to_marker(target, 0.4, 80)
        R.turn_to_marker(target, 0.5, True)
        R.setDriveMotors(50)
        R.sleep(0.4)
        R.stopDriveMotors()
        R.closeGripper()
    if iteration < len(main_cubes[R.zone]) - 1:
        R.seek_markers(return_markers[R.zone], 10, 10)
        R.drive_to_markers(return_markers[R.zone], 1.3, 80, 0.2, 3, 20)
        R.openGripper()
        R.setDriveMotors(-60)
        R.sleep(0.2)
        R.stopDriveMotors()
    else:
        R.seek_markers(return_markers[R.zone], 10, 10)
        R.drive_to_markers(return_markers[R.zone], 1.3, 80, 0.2, 3, 100)
        R.openGripper()
        R.sleep(1)
    iteration += 1
