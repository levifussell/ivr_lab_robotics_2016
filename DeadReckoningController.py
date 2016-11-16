from PIDcontroller import PIDController

import math

class DeadReckoningController:

    def __init__(self, leftMotor, rightMotor, sensorGyro):
        self.position = 0
        self.yaw = 0

        self.position_target = 0
        self.yaw_target = 0

        self.PIDController_position = PIDController()
        self.PIDController_yaw = PIDController()

        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.sensorGyro = sensorGyro

    def update():

        posX_drive = 0
        posY_drive = 0
        yaw_drive = 0

        # first rotate to the expected rotation
        yaw_error =  self.yaw_target - self.yaw
        yaw_drive = self.PIDController_yaw.updatePosition(yaw_error, self.yaw)

        # update drive position only if error is low enough on yaw
        if yaw_error < 1:
            # then drive to position
            pos_error = 

            posX_error =  self.posX_target - self.posX
            posX_drive = self.PIDController_posX.updatePosition(posX_error, self.posX)

            posY_error =  self.posY_target - self.posY
            posY_drive = self.PIDController_posY.updatePosition(posY_error, self.posY)

        self.drive(posX_drive, posY_drive, yaw_drive)

    def drive(d_x, d_y, d_yaw):
        

    def newRelativeTarget(d_x, d_y):
        self.position_target = self.position + math.sqrt(d_x*d_x + d_y*d_y)

        self.yaw_target = math.atan2() 

    def getPosition():
