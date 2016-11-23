from Controller import Controller
from PIDcontroller import PIDController

from Robot import RobotState

import time

class LineFollowController(Controller):

    def __init__(self, robot, actWhenOffLine=True, lineSensitivity=23):
        Controller.__init__(self, robot)

        self.PIDCOntroller_light = None #PIDController(k_proportional=12.2, k_integral=0.52, k_derivative=0.0)#30)#, k_integral=0.07, k_derivative=5)
        # self.PIDCOntroller_light = PIDController(k_proportional=10.2, k_integral=0.01,k_derivative=0)#30)#, k_integral=0.07, k_derivative=5)

        self.PIDCOntroller_motorMiddle = None #PIDController(k_proportional=0.7)
        # self.startMotorMiddle = self.robot.motorMiddle.position

        self.resetPIDs()

        self.light = self.robot.getLightValue()

        self.BLACK_LIGHT_VAL = -1#50
        self.WHITE_LIGHT_VAL = -1#40
        self.BASE_DRIVE_POWER = 30#50

        self.BASE_NORMALISER = -1

        self.light_target = -1#(self.BLACK_LIGHT_VAL + self.WHITE_LIGHT_VAL) / 2

        self.positionTracer = []

        self.negErrorStreak = 0

        self.lineFollowPolarity = 1

        # do something when off the line
        self.actWhenOffLine = actWhenOffLine

        self.lineSensitivity = lineSensitivity

    def resetPIDs(self):

        self.PIDCOntroller_light = PIDController(k_proportional=12.2, k_integral=0.52, k_derivative=0.0)#30)#, k_integral=0.07, k_derivative=5)
        # self.PIDCOntroller_light = PIDController(k_proportional=10.2, k_integral=0.01,k_derivative=0)#30)#, k_integral=0.07, k_derivative=5)

        self.PIDCOntroller_motorMiddle = PIDController(k_proportional=0.7)

    def update(self):

        if self.robot.stateChangeOccurred:
            self.resetPIDs()
            self.robot.stateChangeOccurred = False

        # if the black/white light values have not been set, search for them
        if self.robot.state == RobotState.LINE_COLOUR_EVALUATE: #self.light_target  == -1:
            self.findLightvalues()
        elif self.robot.state == RobotState.LINE_FOLLOW:
            #TODO: fix light sensor values after uncommenting to fix this bug
            self.light = self.robot.getLightValue() # / self.BASE_NORMALISER

            light_error = ((self.light_target - self.light) / self.BASE_NORMALISER)  * self.lineFollowPolarity
            light_drive = self.PIDCOntroller_light.updatePosition(light_error, self.light)

            motorMiddleError = (self.robot.motorMiddleStartPosition + light_drive*3) - self.robot.motorMiddle.position
            middle_drive = self.PIDCOntroller_motorMiddle.updatePosition(motorMiddleError, self.robot.motorMiddle.position)
            # print(middle_drive)
            self.robot.motorMiddle.run_direct(duty_cycle_sp=min(100, max(middle_drive, -100)))

            # print('ki: {}'.format(light_error))

            # self.positionTracer.append(light_drive)
            # print(light_drive)
            self.__drive(light_drive)

            # if light_error <= -1:
            #     self.negErrorStreak += 1
            # else:
            #     self.negErrorStreak = 0
            #
            # print(self.negErrorStreak)

# UNCOMMENT FOR OFF LINE DETECTION-------------------------------------
            if self.actWhenOffLine:
                # constant at value of 23
                off_edge_const = self.lineSensitivity
                if (light_drive < -off_edge_const and self.lineFollowPolarity == 1) or (light_drive > off_edge_const and self.lineFollowPolarity == -1):
                    self.resetPIDs()
                    self.lineFollowPolarity *= -1
                    self.robot.setState(RobotState.OFF_LINE)
                    self.robot.motorLeft.stop()
                    self.robot.motorRight.stop()
                    self.robot.motorMiddle.stop()
                    print('OFFLINE!!!!!')
                    time.sleep(2)
# UNCOMMENT FOR OFF LINE DETECTION-------------------------------------

            # self.positionTracer.append(self.robot.getLightValue())
        # elif self.robot.state == RobotState.OBSTACLE_TRACE:
        #     if self.robot.getLightValue() - self.BLACK_LIGHT_VAL < 5:
        #         self.robot.motorLeft.stop()
        #         self.robot.motorRight.stop()
        #         time.sleep(1)
        #         print('FOUND LINE!')
        #         self.robot.setState(RobotState.DEAD)
                # self.robot.setState(RobotState.LINE_FOLLOW)

    def findLightvalues(self):
        lightValuesRange = []

        # spin in a full circle and record a range of light sensor values
        endGyroVal = self.robot.getGyroValue() + 340

        # start robot on black line always
        # startBlack = self.robot.getLightValue()
        prevLightVal = self.robot.getLightValue()
        itter = 0
        while self.robot.getGyroValue() < endGyroVal or (prevLightVal - self.robot.getLightValue()) < 3:
            print( self.robot.getLightValue())
            # drive in a circle
            self.robot.motorLeft.run_timed(duty_cycle_sp=40, time_sp=50)
            self.robot.motorRight.run_timed(duty_cycle_sp=-40, time_sp=50)

            # record light values
            lightValuesRange.append(self.robot.getLightValue())

            itter += 1
            if itter % 2 == 0:
                prevLightVal = self.robot.getLightValue()

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

        for i in range(1, 101):
            if i in buckets.keys():
                self.positionTracer.append(buckets[i])
            else:
                self.positionTracer.append(0)

        dWhite = dict(buckets.items()[len(buckets)/2:])
        dBlack = dict(buckets.items()[:len(buckets)/4])

        self.WHITE_LIGHT_VAL = max(dWhite, key=dWhite.get)
        self.BLACK_LIGHT_VAL = min(lightValuesRange)#max(dBlack, key=dBlack.get)
        self.light_target = (self.BLACK_LIGHT_VAL + self.WHITE_LIGHT_VAL) / 2
        self.BASE_NORMALISER = float((self.WHITE_LIGHT_VAL - self.BLACK_LIGHT_VAL) / 2)

        self.robot.LIGHT_BLACK_VAL = self.BLACK_LIGHT_VAL
        self.robot.LIGHT_WHITE_VAL = self.WHITE_LIGHT_VAL
        self.robot.LIGHT_MID_VAL = self.light_target

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
