from PIDcontroller import PIDController

import math
import time

import ev3dev.ev3 as ev3

# Turning Constant:
#  62 (mult. by time)
# Driving Constant:
#

class DeadReckoningController:

    def __init__(self, leftMotor, rightMotor, sensorGyro):
        self.position = 0
        # self.yaw = 0

        self.position_target = 0
        self.yaw_target = 0

        # create basic PID controllers for now
        self.PIDController_position = PIDController(k_proportional=0.5, k_integral=0.5)
        self.PIDController_yaw = PIDController(k_proportional=0.5, k_integral=0.01, k_derivative=0.1)

        self.leftMotor = ev3.LargeMotor('outB')
        self.rightMotor = ev3.LargeMotor('outC')
        self.sensorGyro = sensorGyro

        self.yaw = self.sensorGyro.value()

    def update(self):

        posX_drive = 0
        posY_drive = 0
        yaw_drive = 0

        # first rotate to the expected rotation
        yaw_error =  self.yaw_target - self.yaw
        yaw_drive = self.PIDController_yaw.updatePosition(yaw_error, self.yaw)
        # print(self.yaw)
        print('yaw target:{}'.format(self.yaw_target))
        # print(yaw_drive)
        # update drive position only if error is low enough on yaw
        # if yaw_error < 1:
        #     # then drive to position
        #     # pos_error =
        #
        #     posX_error =  self.posX_target - self.posX
        #     posX_drive = self.PIDController_posX.updatePosition(posX_error, self.posX)
        #
        #     posY_error =  self.posY_target - self.posY
        #     posY_drive = self.PIDController_posY.updatePosition(posY_error, self.posY)

        self.__drive(posX_drive, posY_drive, yaw_drive)

    def __drive(self, d_x, d_y, d_yaw):

        # distance = speed * time
        # speed = 80
        yaw_const = 62
        time_to_change_yaw = abs((d_yaw / yaw_const) * 1000)

        if d_yaw > 0:
            self.leftMotor.run_timed(duty_cycle_sp=25, time_sp=time_to_change_yaw)
            self.rightMotor.run_timed(duty_cycle_sp=-25, time_sp=time_to_change_yaw)
        else:
            self.leftMotor.run_timed(duty_cycle_sp=-25, time_sp=time_to_change_yaw)
            self.rightMotor.run_timed(duty_cycle_sp=25, time_sp=time_to_change_yaw)

        self.yaw = self.sensorGyro.value()
        print('yaw:{}'.format(self.yaw))
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
