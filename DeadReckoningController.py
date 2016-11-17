from PIDcontroller import PIDController

import math
import time

import ev3dev.ev3 as ev3

from Controller import Controller

# Turning Constant:
#  62 (mult. by time)
# Driving Constant:
#

class DeadReckoningController(Controller):

    def __init__(self, robot):
        Controller.__init__(self, robot)

        self.position = 0
        # self.yaw = 0

        self.position_target = 0
        self.yaw_target = 0

        # create basic PID controllers for now
        self.PIDController_position = PIDController(k_proportional=1, k_integral=0, k_derivative=0)
        self.PIDController_yaw = PIDController(k_proportional=0.5, k_integral=0.01, k_derivative=0.1)

        # self.leftMotor = leftMotor #ev3.LargeMotor('outB')
        # self.rightMotor = rightMotor #ev3.LargeMotor('outC')
        # self.sensorGyro = sensorGyro

        # define starting position of robot as origin
        self.yaw = self.robot.getGyroValue()
        self.position = self.robot.getAvgEncoderValue()

    def update(self):

        posX_drive = 0
        posY_drive = 0
        yaw_drive = 0
        # to position
            # pos_error =
        # first rotate to the expected rotation
        yaw_error =  self.yaw_target - self.yaw
        yaw_drive = self.PIDController_yaw.updatePosition(yaw_error, self.yaw)
        # print(self.yaw)
        print('yaw target:{}'.format(self.yaw_target))

        # print(yaw_drive)
        # update drive position only if error is low enough on yaw
        if yaw_error < 1:
            # then drive to position
            position_error = self.position_target - self.position
            position_drive = self.PIDController_position.update(position_error, self.position)

            # posX_error =  self.posX_target - self.posX
            # posX_drive = self.PIDController_posX.updatePosition(posX_error, self.posX)
        
            # posY_error =  self.posY_target - self.posY
            # posY_drive = self.PIDController_posY.updatePosition(posY_error, self.posY)

        self.__drive(position_drive, yaw_drive)

    def __drive(self, d_position, d_yaw):

        # distance = speed * time
        # speed = 80
        yaw_const = 62
        time_to_change_yaw = abs((d_yaw / yaw_const) * 1000)
        # currently not used

        yaw_polarity = 0

        if d_yaw > 0:
            yaw_polarity = 1
        else:
            yaw_polarity = -1

        # if yaw change is positive, rotate clockwise
        # if d_yaw > 0:
        self.robot.motorLeft.run_timed(duty_cycle_sp=25 * yaw_polarity, time_sp=500)
        self.robot.motorRight.run_timed(duty_cycle_sp=-25 * yaw_polarity, time_sp=500)
        # # otherwise rotate counter-clockwise
        # else:
        #     self.robot.motorLeft.run_timed(duty_cycle_sp=-25, time_sp=time_to_change_yaw)
        #     self.robot.motorRight.run_timed(duty_cycle_sp=25, time_sp=time_to_change_yaw)

        # pos_const = 20.385
        # currently not used

        pos_polarity = 0

        if d_position > 0:
            pos_polarity = 1
        else:
            pos_polarity = -1

        self.robot.motorLeft.run_timed(duty_cycle_sp=25 * pos_polarity, time_sp=500)
        self.robot.motorRight.run_timed(duty_cycle_sp=25 * pos_polarity, time_sp=500)

        # update the current yaw to the sensor value of the robot
        self.yaw = self.robot.getGyroValue()

        # update the current position to the sensor value of the robot
        self.position = self.robot.getAvgEncoderValue()

        # display current values for debugging
        print('yaw:{}, position:{}'.format(self.yaw, self.position))

        # print(time_to_change_yaw)
        # time.sleep(time_to_change_yaw / 1000 + 1)

        # # first drive yaw
        # self.leftMotor.run_timed(duty_cycle_sp=50, time_sp=500)
        # self.rightMotor.run_timed(duty_cycle_sp=-50, time_sp=500)
        # time.sleep(1)
        #
        # # then drive position
        # self.leftMotor.run_timed(duty_cycle_sp=100, time_sp=500)
        # self.rightMotor.run_timed(duty_cycle_sp=100, time_sp=500)
        # time.sleep(1)

    def newRelativeTarget(self, d_x, d_y):
        self.position_target = self.position + math.sqrt(d_x*d_x + d_y*d_y)

        angle = math.atan2(d_y, d_x)
        self.yaw_target = self.yaw + (angle * 360 / (2 * 3.141) + 90)
        # print(self.yaw)
        # print(angle)

    # def getPosition():
