from Controller import Controller
from PIDcontroller import PIDController
from DeadReckoningController import DeadReckoningController

from Robot import RobotState

import time

import math

# class ObstacleScanDirection(Enum):
#     LEFT = 1
#     RIGHT = 2

class ObstacleDetectController(Controller):

    def __init__(self, robot):
        Controller.__init__(self, robot)

        self.deadRec_controller = DeadReckoningController(robot)

        self.PIDCOntroller_motorSonar = PIDController(k_proportional=4, k_integral=0.05)

        # FOR LOG (BEST RUN)
        self.PIDCOntroller_sonar = PIDController(k_proportional=20, k_integral=0.0005)
        # self.PIDCOntroller_sonar = PIDController(k_proportional=40, k_integral=0.001)

        self.previousSonarValue = self.robot.getSonarValue()
        # self.SONARCHANGE_OBSTVALUE = 200
        self.OBJECTSCAN_MIN = -1

        self.ultraValues = []

        # self.scanDirection = ObstacleScanDirection.LEFT
        # self.scanLeftValue = -1
        # self.scanRightValue = -1

    def timestamp_now (self): return int (time.time () * 1E3)

    def update(self):

        # always update the dead reckoning controller
        # self.deadRec_controller.update()

        # check if the ultrasonic has changed dramatically
        if self.robot.state == RobotState.LINE_FOLLOW:
            diffSonar = abs(self.robot.getSonarValue() - self.previousSonarValue)
            # print(self.robot.getSonarValue())
            if self.robot.getSonarValue() < 60:#(diffSonar > self.SONARCHANGE_OBSTVALUE):
                # object detected
                self.robot.setState(RobotState.OBSTACLE_DETECT)
                self.robot.motorLeft.stop()
                self.robot.motorRight.stop()

                # t_start = self.timestamp_now()
                # # set motor middle to starting value
                # while self.timestamp_now() - t_start < 3000:
                #     middle_error = self.robot.motorMiddleStartPosition - self.robot.motorMiddle.position
                #     d_middle = self.PIDCOntroller_motorSonar.updatePosition(middle_error, self.robot.motorMiddle.position)
                #     self.__drive(d_middle)
                #     # self.robot.motorMiddle.run_direct(duty_cycle_sp=diff_to)
                #     # diff_to = self.robot.motorMiddleStartPosition - self.robot.motorMiddle.position
                #     print(d_middle)

                    # if abs(middle_error) < 1:
                    #     break

                driveSpeed = 100
                t_start = self.timestamp_now()
                while self.timestamp_now() - t_start < 1000:

                    self.robot.motorMiddle.run_direct(duty_cycle_sp=driveSpeed)

                    if self.robot.getTouchValue():
                        driveSpeed *= 0.9

                    if driveSpeed < 4:
                        break

                self.robot.motorMiddle.stop()
                self.robot.setState(RobotState.OBSTACLE_SCAN)
                print('settled')

                self.previousSonarValue = self.robot.getSonarValue()

        elif self.robot.state == RobotState.OBSTACLE_SCAN:
            self.scanObject()

        elif self.robot.state == RobotState.OBSTACLE_TRACE:
            self.traceObject()
                # set robot to 90 degree orientation
                # endGyroVal = self.robot.getGyroValue() + 360
                #
                # while self.robot.getGyroValue() < endGyroVal:
                #     # drive in a circle
                #     self.robot.motorLeft.run_timed(duty_cycle_sp=25, time_sp=50)
                #     self.robot.motorRight.run_timed(duty_cycle_sp=-25, time_sp=50)

    def scanObject(self):

        endGyroVal = self.robot.getGyroValue() + 100
        sonarValuesRange = []

        while self.robot.getGyroValue() < endGyroVal:
            # drive in a circle
            self.robot.motorLeft.run_timed(duty_cycle_sp=20, time_sp=50)
            self.robot.motorRight.run_timed(duty_cycle_sp=-20, time_sp=50)

            # if self.robot.getSonarValue() < minSonar:
            #     minSonar = self.robot.getSonarValue()
            # else:
            #     distToMin = self.robot.getSonarValue() - minSonar

            # record light values

            # record light values
            sonarValuesRange.append(self.robot.getSonarValue())

        self.OBJECTSCAN_MIN = float(min(sonarValuesRange))/2550.0

        print('min sonar value:{}'.format(self.OBJECTSCAN_MIN))
        self.robot.setState(RobotState.OBSTACLE_TRACE)

    def traceObject(self):

        sonar_val = float(self.robot.getSonarValue())/2550.0
        sonar_val_log = math.log10(sonar_val)
        sonar_error = math.log10(self.OBJECTSCAN_MIN) - sonar_val_log #normalise the ultra value

        # edge case if the sonar value reads 255cm
        if sonar_val >= 1:
            sonar_error = 0

        # print(sonar_error)
        # pol = 1
        # if sonar_error < 0:
        #     pol = -1
        # if sonar_error != 0:
        #     sonar_error = math.log10(abs(sonar_error)) * pol
        d_sonar = self.PIDCOntroller_sonar.updatePosition(sonar_error, sonar_val_log)
        print(d_sonar)
        baseDrive = 55

        self.ultraValues.append(sonar_val_log)

        self.robot.motorLeft.run_timed(duty_cycle_sp=min(max(baseDrive + d_sonar, -100), 100), time_sp=50)
        self.robot.motorRight.run_timed(duty_cycle_sp=min(max(baseDrive - d_sonar, -100), 100), time_sp=50)

    def __drive(self, d_middle):

        self.robot.motorMiddle.run_direct(duty_cycle_sp=min(max(d_middle, -100), 100))
