import ev3dev.ev3 as ev3

from enum import Enum

class RobotState(Enum):
    LINE_COLOUR_EVALUATE = 1
    LINE_FOLLOW = 2
    OBSTACLE_DETECT = 3
    OBSTACLE_TRAVERSE = 4
    LINE_SEARCH = 5
    OBSTACLE_SCAN = 6

class Robot:

    def __init__(self):

        self.motorLeft = ev3.LargeMotor('outB')
        self.motorRight = ev3.LargeMotor('outD')
        self.motorMiddle = ev3.MediumMotor('outA')
        self.sensorLight = ev3.ColorSensor(ev3.INPUT_1)
        self.sensorLight.mode='COL-REFLECT'
        self.sensorGyro = ev3.GyroSensor(ev3.INPUT_2)
        # self.sensorSonar = ev3.SonarSensor()
        self.sensorGyro.mode = 'GYRO-ANG'
        self.state = RobotState.LINE_COLOUR_EVALUATE

    def driveMotors(self, powerLeft, powerRight, powerMiddle):
        self.driveLeftMotor(powerLeft)
        self.driveRightMotor(powerRight)

    def driveMainMotors(self, powerLeft, powerRight):
        self.driveLeftMotor(powerLeft)
        self.driveRightMotor(powerRight)

    def driveLeftMotor(self, power):
        self.motorLeft.run_timed(duty_cycle_sp=power, time_sp=50)

    def driveRightMotor(self, power):
        self.motorRight.run_timed(duty_cycle_sp=power, time_sp=50)

    def driveMiddleMotor(self, power):
        self.motorMiddle.run_timed(duty_cycle_sp=power, time_sp=50)

    def getLightValue(self):
        return self.sensorLight.value()

    def getGyroValue(self):
        return self.sensorGyro.value()

    # def getSonarValue(self):
    #     return self.sensorSonar.value()

    def getMainAvgEncoderValue(self):
        return (self.motorLeft.position + self.motorRight.position) / 2

    def getMiddleAvgEncoderValue(self):
        return self.driveMiddleMotor.position

    def setState(self, newState):

        if self.state != newState:
            # TODO: make the robot declare its new state
            self.state = newState
