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
        self.PIDCOntroller_sonar = PIDController(k_proportional=20.0, k_derivative=0.0)#k_integral=0.01)
        # self.PIDCOntroller_sonar = PIDController(k_proportional=25, k_integral=0.0005)
        # self.PIDCOntroller_sonar = PIDController(k_proportional=40, k_integral=0.001)

        self.previousSonarValue = self.robot.getSonarValue()
        # self.SONARCHANGE_OBSTVALUE = 200
        self.OBJECTSCAN_MIN = -1

        self.ultraValues = []

        self.hitObjectGyroValue = -1

        self.previousLightVal = -1

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
            if self.robot.getSonarValue() < 50:
            # if self.robot.getSonarValue() < 40:#(diffSonar > self.SONARCHANGE_OBSTVALUE):
                # object detected
                self.robot.setState(RobotState.OBSTACLE_DETECT)
                self.robot.motorLeft.stop()
                self.robot.motorRight.stop()
                self.hitObjectGyroValue = self.robot.getGyroValue();

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
                        driveSpeed *= 0.7

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
            if self.previousLightVal - self.robot.getLightValue() < -8:
                # self.robot.motorLeft.stop()
                # self.robot.motorRight.stop()
                # time.sleep(1)
                # print('FOUND LINE!')
                # self.robot.setState(RobotState.DEAD)
                # endGyroVal = self.robot.getGyroValue() + 100
                # sonarValuesRange = []

                while self.robot.getGyroValue() < self.hitObjectGyroValue:
                    # drive in a circle
                    self.robot.motorLeft.run_timed(duty_cycle_sp=40, time_sp=50)
                    # self.robot.motorRight.run_timed(duty_cycle_sp=-30, time_sp=50)
                time.sleep(1)
                # drive forward for a second
                self.robot.motorLeft.run_timed(duty_cycle_sp=50, time_sp=500)
                self.robot.motorRight.run_timed(duty_cycle_sp=50, time_sp=500)
                time.sleep(1)

                self.previousLightVal = self.robot.getLightValue()
                while self.previousLightVal - self.robot.getLightValue() < 3:
                    # drive in a circle
                    self.robot.motorLeft.run_timed(duty_cycle_sp=30, time_sp=50)
                    self.robot.motorRight.run_timed(duty_cycle_sp=-30, time_sp=50)
                    self.previousLightVal = self.robot.getLightValue()

                time.sleep(1)
                self.robot.setState(RobotState.LINE_FOLLOW)

            self.previousLightVal = self.robot.getLightValue()


        # elif self.robot.state == RobotState.OBSTACLE_TRACE:
        #     if self.robot.getLightValue() - self.BLACK_LIGHT_VAL < 5:
        #         self.robot.motorLeft.stop()
        #         self.robot.motorRight.stop()
        #         time.sleep(1)
        #         print('FOUND LINE!')
        #         self.robot.setState(RobotState.DEAD)

    def scanObject(self):

        endGyroVal = self.robot.getGyroValue() + 100
        sonarValuesRange = []

        while self.robot.getGyroValue() < endGyroVal:
            # drive in a circle
            self.robot.motorLeft.run_timed(duty_cycle_sp=30, time_sp=50)
            self.robot.motorRight.run_timed(duty_cycle_sp=-30, time_sp=50)

            # if self.robot.getSonarValue() < minSonar:
            #     minSonar = self.robot.getSonarValue()
            # else:
            #     distToMin = self.robot.getSonarValue() - minSonar

            # record light values

            # record light values
            sonarValuesRange.append(self.robot.getSonarValue())

        self.OBJECTSCAN_MIN = float(min(sonarValuesRange))

        print('min sonar value:{}'.format(self.OBJECTSCAN_MIN))
        self.previousLightVal = self.robot.getLightValue();
        self.robot.setState(RobotState.OBSTACLE_TRACE)

    def traceObject(self):

        base_sonar = float(self.robot.getSonarValue())
        if base_sonar > self.OBJECTSCAN_MIN * 2:
            base_sonar = self.OBJECTSCAN_MIN * 2

        sonar_val = float(base_sonar)/(self.OBJECTSCAN_MIN * 2)

        sonar_val_log = sonar_val
        sonar_error = 0.5 - sonar_val_log #normalise the ultra value

        # edge case if the sonar value reads 255cm
        # if sonar_val >= 1:
        #     sonar_error = 0

        # print(sonar_error)
        # pol = 1
        # if sonar_error < 0:
        #     pol = -1
        # if sonar_error != 0:
        #     sonar_error = math.log10(abs(sonar_error)) * pol
        d_sonar = self.PIDCOntroller_sonar.updatePosition(sonar_error, sonar_val_log)
        print(sonar_val * 2550.0)
        baseDrive = 32
        # baseDrive = 28

        self.ultraValues.append(sonar_val_log)

        self.robot.motorLeft.run_timed(duty_cycle_sp=min(max(baseDrive + d_sonar, -100), 100), time_sp=50)
        self.robot.motorRight.run_timed(duty_cycle_sp=min(max(baseDrive - d_sonar, -100), 100), time_sp=50)

    def __drive(self, d_middle):

        self.robot.motorMiddle.run_direct(duty_cycle_sp=min(max(d_middle, -100), 100))
