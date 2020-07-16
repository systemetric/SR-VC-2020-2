from sr.robot import *
from typing import Union, Tuple

e = 2.718

def signum(x):
    if x < 0:
        return - 1
    return 1

# sigmoid function. Has range (0, 1) and changes most between the domain of -4 to 4
def Sigmoid(x):
    return 1/(1 + e ** -x)

class VirtualBot(Robot):

    def __init__(self):
        super().__init__()
        self.have_cube = False
        self.direction = 0
        self.largest_angle = 5.0 # the angle where the robot becomes most sensitive to a change in angle.
        self.motor_turn_max = 5 # max power motors will turn at in turn to marker or turn to markers function

    def digitalReadRuggeduino(self, pin: int) -> bool:
        return self.ruggeduinos[0].digital_read(pin)

    def analogReadRuggeduino(self, pin: int) -> float:
        return self.ruggeduinos[0].analogue_read(pin)

    @property
    def left_motor(self):
        return self.motors[0].m0

    @property
    def right_motor(self):
        return self.motors[0].m1

    @property
    def lift_motor(self):
        return self.motors[1].m0

    @property
    def gripper_motor(self):
        return self.motors[1].m1

    @property
    def front_switch(self) -> bool:
        return self.digitalReadRuggeduino(2)

    @property
    def back_switch(self) -> bool:
        return self.digitalReadRuggeduino(3)

    @property
    def between_gripper_switch(self) -> bool:
        return self.digitalReadRuggeduino(4)

    @property
    def left_finger_switch(self) -> bool:
        return self.digitalReadRuggeduino(5)

    @property
    def right_finger_switch(self) -> bool:
        return self.digitalReadRuggeduino(6)

    @property
    def front_left_distance(self) -> float:
        return self.analogReadRuggeduino(0)

    @property
    def front_right_distance(self) -> float:
        return self.analogReadRuggeduino(1)

    @property
    def left_distance(self) -> float:
        return self.analogReadRuggeduino(2)

    @property
    def right_distance(self) -> float:
        return self.analogReadRuggeduino(3)

    @property
    def back_left_distance(self) -> float:
        return self.analogReadRuggeduino(4)

    @property
    def back_right_distance(self) -> float:
        return self.analogReadRuggeduino(5)

    def setDriveMotors(self, power: Union[float, Tuple[float, float]]):
        try:
            self.left_motor.power = power[0]
            self.right_motor.power = power[1]
        except:
            self.left_motor.power = power
            self.right_motor.power = power

    def stopDriveMotors(self):
        self.setDriveMotors(0)
        # self.left_motor.power = 0
        # self.right_motor.power = 0

    def turn(self, power):
        self.setDriveMotors((power, -power))

    def raiseGripper(self):
        self.lift_motor.power = -100
        self.sleep(0.3)
        self.lift_motor.power = 0

    def lowerGripper(self):
        self.lift_motor.power = 100
        self.sleep(0.3)
        self.lift_motor.power = 0

    def closeGripper(self):
        self.gripper_motor.power = -100
        for _ in range(15):
            if not self.left_finger_switch or not self.right_finger_switch:
                self.sleep(0.1)
            else:
                break
        else:
            print(f"HR Team 2 in zone {self.zone}: No cube in gripper")
            self.openGripper()
            self.have_cube = False
            return
        print(f"HR Team 2 in zone {self.zone}: Got cube")
        self.have_cube = True

    def openGripper(self):
        self.gripper_motor.power = 100
        self.have_cube = False

    def find_marker(self, code: int, markers=None):
        """Returns the marker object that has the given code if not returns None.
        The list of markers can be given if not R.see() will be used.
        """
        if markers == None:
            markers = self.see()
        for m in markers:
            if m.info.code == code:
                return m
        return None

    def find_markers(self, codes, markers=None):
        """Returns the marker object that has the given code if not returns None.
        The list of markers can be given if not R.see() will be used.
        """
        if markers == None:
            markers = self.see()
        for m in markers:
            if m.info.code in codes:
                return m
        return None

    def find_markers_with_priority(self, priority, less_important, markers=None):
        """Returns the marker object that has the given code if not returns None.
        The list of markers can be given if not R.see() will be used.
        """
        found = None
        if markers == None:
            markers = self.see()
        for m in markers:
            if m.info.code in priority:
                return m
            elif m.info.code in less_important:
                found = m
        return found

    def turn_to_marker(self, code: int, dist, see, epsilon: int = 2):
        """Turns to the marker of the given code and returns its object.
        Returns None if the cube cannot be seen.
        Epsilon is the accuracy at which the robot should be facing the marker at.
        e.g. When epsilon is 1 the angle the robot is facing when this function exits will be less than 1 but greater than -1."""
        m = self.find_marker(code)
        if m is None:
            print(f"HR Team 2 in zone {self.zone}: ERROR: Cannot see marker {code}")
            return None
        times = 0
        while not (-epsilon < m.rot_y and m.rot_y < epsilon) and times < 20:
            self.direction = signum(m.rot_y)
            value = self.direction * (Sigmoid(8 * (abs(m.rot_y)/ self.largest_angle) - 4) * self.motor_turn_max)
            self.turn(value) 
            self.sleep(0.00125)
            if see and m.dist < dist:
                self.turn(-value)
                self.sleep(0.00125)
            self.stopDriveMotors()
            self.setDriveMotors(10)
            self.sleep(0.001)
            self.stopDriveMotors()
            times += 1
            m = self.find_marker(code)
            if m is None:
                print(f"HR Team 2 in zone {self.zone}: ERROR: Can no longer see marker {code}")
                return None

        print(f"HR Team 2 in zone {self.zone}: Done, at degree: {m.rot_y}")
        return m

    def turn_to_markers(self, codes, dist, see, epsilon: int = 2):
        """Turns to the marker of the given code and returns its object.
        Returns None if the cube cannot be seen.
        Epsilon is the accuracy at which the robot should be facing the marker at.
        e.g. When epsilon is 1 the angle the robot is facing when this function exits will be less than 1 but greater than -1."""
        m = self.find_markers(codes)
        if m is None:
            print(f"HR Team 2 in zone {self.zone}: ERROR: Cannot see marker a marker in {codes}")
            return None
        times = 0
        while not (-epsilon < m.rot_y and m.rot_y < epsilon) and times < 20:
            self.direction = signum(m.rot_y)
            value = self.direction * (Sigmoid(8 * (abs(m.rot_y)/ self.largest_angle) - 4) * self.motor_turn_max)
            self.turn(value)
            self.sleep(0.00125)
            if see and m.dist < dist:
                self.turn(-value)
                self.sleep(0.00125)
            self.stopDriveMotors()
            self.setDriveMotors(40)
            self.sleep(0.001)
            self.stopDriveMotors()
            times += 1
            m = self.find_markers(codes)
            if m is None:
                print(f"HR Team 2 in zone {self.zone}: ERROR: Can no longer see marker in {codes}")
                return None

        print(f"HR Team 2 in zone {self.zone}: Done, at degree: {m.rot_y}")
        return m

    def turn_to_markers_with_priority(self, priority, less_important, dist, see, epsilon: int = 2):
        """Turns to the marker of the given code and returns its object.
        Returns None if the cube cannot be seen.
        Epsilon is the accuracy at which the robot should be facing the marker at.
        e.g. When epsilon is 1 the angle the robot is facing when this function exits will be less than 1 but greater than -1."""
        m = self.find_markers_with_priority(priority, less_important)
        if m is None:
            print(f"HR Team 2 in zone {self.zone}: ERROR: Cannot see marker a marker in {priority} or {less_important}")
            return None
        times = 0
        while not (-epsilon < m.rot_y and m.rot_y < epsilon) and times < 20:
            self.direction = signum(m.rot_y)
            value = self.direction * (Sigmoid(8 * (abs(m.rot_y)/ self.largest_angle) - 4) * self.motor_turn_max)
            self.turn(value)
            self.sleep(0.00125)
            if see and m.dist < dist:
                self.turn(-value)
                self.sleep(0.00125)
            self.stopDriveMotors()
            self.setDriveMotors(40)
            self.sleep(0.001)
            self.stopDriveMotors()
            times += 1
            m = self.find_markers_with_priority(priority, less_important)
            if m is None:
                print(f"HR Team 2 in zone {self.zone}: ERROR: Can no longer see marker in {priority} or in {less_important}")
                return None

        print(f"HR Team 2 in zone {self.zone}: Done, at degree: {m.rot_y}")
        return m

    def seek_marker(self, code: int, power: int = 10, default_power: int = 10, repeats: int = None, interval: float = 0.02):
        """Turns until the marker is found. Power to turn at and number of turns can be given.
        If repeats is None it will keep going forever until the marker is found.
        """
        m = self.find_marker(code)
        while m is None:
            if self.direction != 0:
                self.turn(power * self.direction)
            else:
                self.turn(default_power)
            self.sleep(interval)
            #self.stopDriveMotors()
            if repeats is not None:
                repeats -= 1
                if repeats <= 0:
                    print(
                        f"HR Team 2 in zone {self.zone}: ERROR: Could not find marker {code} with in alloted steps")
                    break
            m = self.find_marker(code)
        print(
                        f"HR Team 2 in zone {self.zone}: Found marker {code} with in alloted steps")
        return

    def seek_markers(self, codes, power: int = 10, default_power: int = 10, repeats: int = None, interval: float = 0.02):
        """Turns until the marker is found. Power to turn at and number of turns can be given.
        If repeats is None it will keep going forever until the marker is found.
        """
        m = self.find_markers(codes)
        while m is None:
            if self.direction != 0:
                self.turn(power * self.direction)
            else:
                self.turn(power)
            self.sleep(interval)
            #self.stopDriveMotors()
            if repeats is not None:
                repeats -= 1
                if repeats <= 0:
                    print(f"HR Team 2 in zone {self.zone}: ERROR: Could not find a return to marker with in alloted steps")
                    break
            m = self.find_markers(codes)
        print(f"HR Team 2 in zone {self.zone}: Found a return to marker with in alloted steps")
        return m

    def seek_markers_with_priority(self, priority, less_important, power: int = 10, default_power: int = 10, repeats: int = None, interval: float = 0.02):
        """Turns until the marker is found. Power to turn at and number of turns can be given.
        If repeats is None it will keep going forever until the marker is found.
        """
        m = self.find_markers_with_priority(priority, less_important)
        while m is None:
            if self.direction != 0:
                self.turn(power * self.direction)
            else:
                self.turn(power)
            self.sleep(interval)
            #self.stopDriveMotors()
            if repeats is not None:
                repeats -= 1
                if repeats <= 0:
                    print(f"HR Team 2 in zone {self.zone}: ERROR: Could not find a return to marker with in alloted steps")
                    break
            m = self.find_markers_with_priority(priority, less_important)
        print(f"HR Team 2 in zone {self.zone}: Found a return to marker with in alloted steps")
        return m

    def drive_to_marker(self, code: int, dist: float = 0.3, power: int = 10, interval: float = 0.2, epsilon: int = 3, errors = 5):
        """Drives straight towards a marker of the given code and stops a given distance away.
        interval is the time between checking if the robot is facing the marker and driving.
        """
        lowered_cube = False
        tries = 0
        m = self.turn_to_marker(code, 0.2, True, epsilon=epsilon)
        found = None
        if m is None:
            m = self.turn_to_marker(code, 0.2, True, epsilon=epsilon)
            if m is None:
                m = self.turn_to_marker(code, 0.2, True, epsilon=epsilon)
            if m is None and self.have_cube and not lowered_cube:
                self.lowerGripper()
                m = self.turn_to_marker(code, 0.2, True, epsilon=epsilon)
            if m is None:
                return m
        print("HR Team 2 in zone", str(self.zone) + ":", self.right_distance, "right distance sensor")
        print("HR Team 2 in zone", str(self.zone) + ":", self.left_distance, "left distance sensor")
        while m.dist > dist and ((self.left_distance > dist and self.right_distance > dist) or dist >= 0.3):
            print("HR Team 2 in zone", str(self.zone) + ":", self.right_distance, "right distance sensor")
            print("HR Team 2 in zone", str(self.zone) + ":", self.left_distance, "left distance sensor")
            self.setDriveMotors(power)
            #self.sleep(interval)
            #self.stopDriveMotors()
            self.seek_marker(code, 10, 20, 20)
            found = self.turn_to_marker(code, 0.2, True, epsilon=epsilon)
            if found is None:
                tries += 1
                if tries == errors:
                    return
            else:
                tries = 0
                m = found
            print("HR Team 2 in zone", str(self.zone) + ":", m.dist, "m away is shortest known distance")
        self.stopDriveMotors()
        print(f"HR Team 2 in zone {self.zone}: Done, {m.dist}m away")
        self.direction = 0
        return m

    def drive_to_markers(self, codes, dist: float = 0.3, power: int = 10, interval: float = 0.2, epsilon: int = 3, errors = 5):
        """Drives straight towards a marker of the given code and stops a given distance away.
        interval is the time between checking if the robot is facing the marker and driving.
        """
        lowered_cube = False
        tries = 0
        m = self.turn_to_markers(codes, 0.2, True, epsilon=epsilon)
        found = None
        if m is None:
            m = self.turn_to_markers(codes, 0.2, True, epsilon=epsilon)
            if m is None:
                m = self.turn_to_markers(codes, 0.2, True, epsilon=epsilon)
            if m is None and self.have_cube and not lowered_cube:
                self.lowerGripper()
                m = self.turn_to_markers(codes, 0.2, True, epsilon=epsilon)
            if m is None:
                return m
        print("HR Team 2 in zone", str(self.zone) + ":", self.right_distance, "right distance sensor")
        print("HR Team 2 in zone", str(self.zone) + ":", self.left_distance, "left distance sensor")
        while m.dist > dist and ((self.left_distance > dist and self.right_distance > dist) or dist >= 0.3):
            print("HR Team 2 in zone", str(self.zone) + ":", self.right_distance, "right distance sensor")
            print("HR Team 2 in zone", str(self.zone) + ":", self.left_distance, "left distance sensor")
            self.setDriveMotors(power)
            #self.sleep(interval)
            #self.stopDriveMotors()
            self.seek_markers(codes, 10, 20, 20)
            found = self.turn_to_markers(codes, 0.2, True, epsilon=epsilon)
            if found is None:
                tries += 1
                if tries == errors:
                    return
            else:
                tries = 0
                m = found
            print("HR Team 2 in zone", str(self.zone) + ":", m.dist, "m away is shortest known distance")
        self.stopDriveMotors()
        print(f"HR Team 2 in zone {self.zone}: Done, {m.dist}m away")
        self.direction = 0
        return m

    def drive_to_markers_with_priority(self, priority, less_important, dist: float = 0.3, power: int = 10, interval: float = 0.2, epsilon: int = 3, errors = 5):
        """Drives straight towards a marker of the given code and stops a given distance away.
        interval is the time between checking if the robot is facing the marker and driving.
        """
        lowered_cube = False
        tries = 0
        m = self.turn_to_markers_with_priority(priority, less_important, 0.2, True, epsilon=epsilon)
        found = None
        if m is None:
            m = self.turn_to_markers_with_priority(priority, less_important, 0.2, True, epsilon=epsilon)
            if m is None:
                m = self.turn_to_markers_with_priority(priority, less_important, 0.2, True, epsilon=epsilon)
            if m is None and self.have_cube and not lowered_cube:
                self.lowerGripper()
                m = self.turn_to_markers_with_priority(priority, less_important, 0.2, True, epsilon=epsilon)
            if m is None:
                return m
        print("HR Team 2 in zone", str(self.zone) + ":", self.right_distance, "right distance sensor")
        print("HR Team 2 in zone", str(self.zone) + ":", self.left_distance, "left distance sensor")
        while m.dist > dist and ((self.left_distance > dist and self.right_distance > dist) or dist >= 0.3):
            print("HR Team 2 in zone", str(self.zone) + ":", self.right_distance, "right distance sensor")
            print("HR Team 2 in zone", str(self.zone) + ":", self.left_distance, "left distance sensor")
            self.setDriveMotors(power)
            #self.sleep(interval)
            #self.stopDriveMotors()
            self.seek_markers_with_priority(priority, less_important, 10, 20, 20)
            found = self.turn_to_markers_with_priority(priority, less_important, 0.2, True, epsilon=epsilon)
            if found is None:
                tries += 1
                if tries == errors:
                    return
            else:
                tries = 0
                m = found
            print("HR Team 2 in zone", str(self.zone) + ":", m.dist, "m away is shortest known distance")
        self.stopDriveMotors()
        print(f"HR Team 2 in zone {self.zone}: Done, {m.dist}m away")
        self.direction = 0
        return m
