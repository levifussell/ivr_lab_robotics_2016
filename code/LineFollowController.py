from Controller import Controller
from PIDcontroller import PIDController

class LineFollowController(Controller):

    def __init__(self, robot):
        Controller.__init__(self, robot)

        self.PIDCOntroller_light = PIDController(k_proportional=0.7)

        self.light = self.robot.getLightValue()

        self.BLACK_LIGHT_VAL = -1#50
        self.WHITE_LIGHT_VAL = -1#40
        self.BASE_DRIVE_POWER = 25

        self.light_target = -1#(self.BLACK_LIGHT_VAL + self.WHITE_LIGHT_VAL) / 2

        self.positionTracer = []

    def update(self):

        # if the black/white light values have not been set, search for them
        if self.light_target  == -1:
            self.findLightvalues()
        else:
            light_error = self.light_target - self.light
            light_drive = self.PIDCOntroller_light.updatePosition(light_error, self.light)

            self.__drive(light_drive)

            self.positionTracer.append(self.robot.getLightValue())

    def findLightvalues(self):
        lightValuesRange = []

        # spin in a full circle and record a range of light sensor values
        endGyroVal = self.robot.getGyroValue() + 360

        while self.robot.getGyroValue() < endGyroVal:
            # drive in a circle
            self.robot.motorLeft.run_timed(duty_cycle_sp=25, time_sp=500)
            self.robot.motorRight.run_timed(duty_cycle_sp=-25, time_sp=500)

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

        dBlack = dict(buckets.items()[len(buckets)/2:])
        dWhite = dict(buckets.items()[:len(buckets)/2])

        self.WHITE_LIGHT_VAL = max(dWhite, key=dWhite.get)
        self.BLACK_LIGHT_VAL = max(dBlack, key=dBlack.get)
        self.light_target = (self.BLACK_LIGHT_VAL + self.WHITE_LIGHT_VAL) / 2

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

    def __drive(self, d_light):

        if d_light > self.BASE_DRIVE_POWER:
            d_light = self.BASE_DRIVE_POWER
        elif d_light < -self.BASE_DRIVE_POWER:
            d_light = -self.BASE_DRIVE_POWER

        self.robot.motorLeft.run_timed(duty_cycle_sp=self.BASE_DRIVE_POWER + d_light, time_sp=500)
        self.robot.motorRight.run_timed(duty_cycle_sp=self.BASE_DRIVE_POWER - d_light, time_sp=500)

        self.light = self.robot.getLightValue()
