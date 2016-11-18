from Controller import Controller
from PIDcontroller import PIDController

from Robot import RobotState

import time

class LineFollowController(Controller):

    def __init__(self, robot):
        Controller.__init__(self, robot)

        self.PIDCOntroller_light = PIDController(k_proportional=15.2, k_integral=0.02,k_derivative=0)#30)#, k_integral=0.07, k_derivative=5)
        # self.PIDCOntroller_light = PIDController(k_proportional=10.2, k_integral=0.01,k_derivative=0)#30)#, k_integral=0.07, k_derivative=5)

        self.PIDCOntroller_motorMiddle = PIDController(k_proportional=0.7)
        # self.startMotorMiddle = self.robot.motorMiddle.position

        self.light = self.robot.getLightValue()

        self.BLACK_LIGHT_VAL = -1#50
        self.WHITE_LIGHT_VAL = -1#40
        self.BASE_DRIVE_POWER = 25

        self.BASE_NORMALISER = -1

        self.light_target = -1#(self.BLACK_LIGHT_VAL + self.WHITE_LIGHT_VAL) / 2

        self.positionTracer = []

    def update(self):

        # if the black/white light values have not been set, search for them
        if self.robot.state == RobotState.LINE_COLOUR_EVALUATE: #self.light_target  == -1:
            self.findLightvalues()
        elif self.robot.state == RobotState.LINE_FOLLOW:
            #TODO: fix light sensor values after uncommenting to fix this bug
            self.light = self.robot.getLightValue() # / self.BASE_NORMALISER

            light_error = (self.light_target - self.light) / self.BASE_NORMALISER
            light_drive = self.PIDCOntroller_light.updatePosition(light_error, self.light)

            motorMiddleError = (self.robot.motorMiddleStartPosition + light_drive*3) - self.robot.motorMiddle.position
            middle_drive = self.PIDCOntroller_motorMiddle.updatePosition(motorMiddleError, self.robot.motorMiddle.position)
            # print(middle_drive)
            self.robot.motorMiddle.run_direct(duty_cycle_sp=min(100, max(middle_drive, -100)))

            # print(light_drive)
            self.__drive(light_drive)

            self.positionTracer.append(self.robot.getLightValue())
        elif self.robot.state == RobotState.OBSTACLE_TRACE:
            if self.robot.getLightValue() - self.BLACK_LIGHT_VAL < 5:
                self.robot.motorLeft.stop()
                self.robot.motorRight.stop()
                time.sleep(1)
                print('FOUND LINE!')
                self.robot.setState(RobotState.OFF_LINE)
                # self.robot.setState(RobotState.LINE_FOLLOW)

    def findLightvalues(self):
        lightValuesRange = []

        # spin in a full circle and record a range of light sensor values
        endGyroVal = self.robot.getGyroValue() + 360

        while self.robot.getGyroValue() < endGyroVal:
            # drive in a circle
            self.robot.motorLeft.run_timed(duty_cycle_sp=25, time_sp=50)
            self.robot.motorRight.run_timed(duty_cycle_sp=-25, time_sp=50)

            # record light values
            lightValuesRange.append(self.robot.getLightValue())

        # for i in range(0, len(lightValuesRange)):
        #     print(lightValuesRange)

        # make a histogram of all the light values
        buckets = {} # dictionary of all light values

        # add each light value to a histogram bucket
        for i in range(0, len(lightValuesRange)):
            if lightValuesRange[i] in buckets.keys():
                buckets[lightValuesRange[i]] += 1
            else:
                buckets[lightValuesRange[i]] = 1

        # print the histogram values
        for i in buckets:
            print('{}: {}'.format(i, buckets[i]))

        dWhite = dict(buckets.items()[len(buckets)/2:])
        dBlack = dict(buckets.items()[:len(buckets)/4])

        self.WHITE_LIGHT_VAL = max(dWhite, key=dWhite.get)
        self.BLACK_LIGHT_VAL = max(dBlack, key=dBlack.get)
        self.light_target = (self.BLACK_LIGHT_VAL + self.WHITE_LIGHT_VAL) / 2
        self.BASE_NORMALISER = float((self.WHITE_LIGHT_VAL - self.BLACK_LIGHT_VAL) / 2)

        print('final vals: b:{}, w:{}, t:{}'.format(self.BLACK_LIGHT_VAL, self.WHITE_LIGHT_VAL, self.light_target))

        # find the two maximum values, these are black and white values
        # maxWhite = max(buckets)
        # del buckets[max(buckets)]
        #
        # maxBlack = max(buckets)
        # del buckets[max(buckets)]
        #
        # self.WHITE_LIGHT_VAL = maxWhite
        # self.BLACK_LIGHT_VAL = maxBlack
        # self.light_target = (self.BLACK_LIGHT_VAL + self.WHITE_LIGHT_VAL) / 2

        self.robot.setState(RobotState.LINE_FOLLOW)

    def __drive(self, d_light):

        # if d_light > self.BASE_DRIVE_POWER:
        #     d_light = self.BASE_DRIVE_POWER
        # elif d_light < -self.BASE_DRIVE_POWER:
        #     d_light = -self.BASE_DRIVE_POWER

        self.robot.motorLeft.run_timed(duty_cycle_sp=max(-100, min(self.BASE_DRIVE_POWER - d_light, 100)), time_sp=50)
        self.robot.motorRight.run_timed(duty_cycle_sp=max(-100, min(self.BASE_DRIVE_POWER + d_light, 100)), time_sp=50)

        # self.light = self.robot.getLightValue()
