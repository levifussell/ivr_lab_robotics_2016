from Robot import RobotState

from Controller import Controller

from DeadReckoningController import DeadReckoningController

from PIDcontroller import PIDController

import time
import math

class LineState:
    LEFT = 1
    RIGHT = 2

class LineSearcherController(Controller):

    def __init__(self, robot):
        Controller.__init__(self, robot)

        # always start on the leftmost line
        self.lineState = LineState.RIGHT
        # number of times the robot has moved between a line
        self.numLinesMoved = 0

        # get the initial angle of the robot so we can
        #  turn 90dgrs relative to this
        self.deadRec_controller = DeadReckoningController(robot)
        self.deadRec_controller.yaw += 360
        # self.startingGyro = self.robot.getGyroValue()
        # self.aimToLeft = self.startingGyro - 90
        # self.aimToRight = self.startingGyro + 90

        self.turn_PID = PIDController(k_proportional=1)

        self.previousLightVal = -1

    def update(self):

        # only run if robot is offline
        if self.robot.state == RobotState.OFF_LINE:

            if self.lineState == LineState.LEFT:
                targetDegree = self.robot.sensorGyroStartValue + 90
                print(targetDegree)
                # aim to the right
                while True:
                    d_error = float(targetDegree - self.robot.getGyroValue())
                    d_turn = self.turn_PID.updatePosition(d_error, self.robot.getGyroValue())
                    print(d_error)
                    if d_error > 0:
                        self.robot.motorLeft.run_timed(duty_cycle_sp=25, time_sp=50)
                        self.robot.motorRight.run_timed(duty_cycle_sp=-25, time_sp=50)
                    else:
                        self.robot.motorLeft.run_timed(duty_cycle_sp=-25, time_sp=50)
                        self.robot.motorRight.run_timed(duty_cycle_sp=25, time_sp=50)


                    if abs(d_error) < 5:
                        # self.lineState = LineState.RIGHT
                        break
            elif self.lineState == LineState.RIGHT:
                targetDegree = self.robot.sensorGyroStartValue - 90
                print(targetDegree)
                # aim to the right
                while True:
                    d_error = float(targetDegree - self.robot.getGyroValue())
                    d_turn = self.turn_PID.updatePosition(d_error, self.robot.getGyroValue())

                    if d_error > 0:
                        self.robot.motorLeft.run_timed(duty_cycle_sp=25, time_sp=50)
                        self.robot.motorRight.run_timed(duty_cycle_sp=-25, time_sp=50)
                    else:
                        self.robot.motorLeft.run_timed(duty_cycle_sp=-25, time_sp=50)
                        self.robot.motorRight.run_timed(duty_cycle_sp=25, time_sp=50)


                    if abs(d_error) < 5:
                        # self.lineState = LineState.LEFT
                        break
                # dDiff =  float(targetDegree - self.robot.getGyroValue())
                # dDiffrad = (dDiff / 360.0) * (2 * 3.141)
                # xT = math.cos(dDiffrad) * 100
                # self.deadRec_controller.newRelativeTarget(xT, 0)

                # print('target:{}'.format(targetDegree))
                # print('current:{}'.format(self.robot.getGyroValue()))

            # begin line search
            self.robot.setState(RobotState.LINE_SEARCH)
            self.previousLightVal = self.robot.getLightValue()

        elif self.robot.state == RobotState.LINE_SEARCH or self.robot.state == RobotState.OBSTACLE_TRACE:

            # check for drastic change in light
            # print(abs(self.previousLightVal - self.robot.getLightValue()))
            # if self.robot.getLightValue() < self.robot.LIGHT_MID_VAL:
                # self.robot.motorLeft.stop()
                # self.robot.motorRight.stop()
                # self.robot.motorMiddle.stop()
                # print('FOUND NEW LINE')
                # self.previousLightVal = self.robot.getLightValue()

            if self.robot.getLightValue() < self.robot.LIGHT_MID_VAL and self.lineState == LineState.LEFT and (self.previousLightVal - self.robot.getLightValue()) < -3:
                self.robot.setState(RobotState.LINE_FOLLOW)
                self.robot.motorLeft.stop()
                self.robot.motorRight.stop()
                self.robot.motorMiddle.stop()
                print('FOUND NEW LINE')
                targetDegree = self.robot.sensorGyroStartValue
                print(targetDegree)

                self.robot.motorLeft.run_timed(duty_cycle_sp=50, time_sp=300)
                self.robot.motorRight.run_timed(duty_cycle_sp=50, time_sp=300)
                time.sleep(1)

                # turn to face middle
                while True:#(self.previousLightVal - self.robot.getLightValue()) > -3:
                    d_error = float(targetDegree - self.robot.getGyroValue())
                    d_turn = self.turn_PID.updatePosition(d_error, self.robot.getGyroValue())

                    if d_error > 0:
                        self.robot.motorLeft.run_timed(duty_cycle_sp=25, time_sp=50)
                        self.robot.motorRight.run_timed(duty_cycle_sp=-25, time_sp=50)
                    else:
                        self.robot.motorLeft.run_timed(duty_cycle_sp=-25, time_sp=50)
                        self.robot.motorRight.run_timed(duty_cycle_sp=25, time_sp=50)

                    self.previousLightVal = self.robot.getLightValue()
                    # if abs(d_error) < 5:
                    # self.robot.getLightValue() < self.robot.LIGHT_MID_VAL and
                    if abs(d_error) < 5:
                        self.lineState = LineState.RIGHT
                        self.robot.setState(RobotState.LINE_FOLLOW)

                        # add a line to the line count
                        self.numLinesMoved += 1
                        break

            elif self.robot.getLightValue() < self.robot.LIGHT_MID_VAL and self.lineState == LineState.RIGHT and (self.previousLightVal - self.robot.getLightValue()) < -3:
                self.robot.setState(RobotState.LINE_FOLLOW)
                self.robot.motorLeft.stop()
                self.robot.motorRight.stop()
                self.robot.motorMiddle.stop()
                print('FOUND NEW LINE')
                targetDegree = self.robot.sensorGyroStartValue
                print(targetDegree)

                self.robot.motorLeft.run_timed(duty_cycle_sp=50, time_sp=300)
                self.robot.motorRight.run_timed(duty_cycle_sp=50, time_sp=300)
                time.sleep(1)

                # turn to face middle
                while True:#(self.previousLightVal - self.robot.getLightValue()) > -3:
                    d_error = float(targetDegree - self.robot.getGyroValue())
                    d_turn = self.turn_PID.updatePosition(d_error, self.robot.getGyroValue())

                    if d_error > 0:
                        self.robot.motorLeft.run_timed(duty_cycle_sp=25, time_sp=50)
                        self.robot.motorRight.run_timed(duty_cycle_sp=-25, time_sp=50)
                    else:
                        self.robot.motorLeft.run_timed(duty_cycle_sp=-25, time_sp=50)
                        self.robot.motorRight.run_timed(duty_cycle_sp=25, time_sp=50)

                    self.previousLightVal = self.robot.getLightValue()
                    # if abs(d_error) < 5:
                    # self.robot.getLightValue() < self.robot.LIGHT_MID_VAL and
                    if abs(d_error) < 5:
                        self.lineState = LineState.LEFT
                        self.robot.setState(RobotState.LINE_FOLLOW)

                        # add a line to the line count
                        self.numLinesMoved += 1
                        break

                    # self.previousLightVal = self.robot.getLightValue()
                # self.robot.setState(RobotState.LINE_FOLLOW)

            else:
                self.robot.motorLeft.run_timed(duty_cycle_sp=20, time_sp=50)
                self.robot.motorRight.run_timed(duty_cycle_sp=20, time_sp=50)

            self.previousLightVal = self.robot.getLightValue()

            # self.deadRec_controller.update()
