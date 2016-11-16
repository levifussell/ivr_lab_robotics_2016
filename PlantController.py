from DeadReckoningController import DeadReckoningController

class PlantController:

    def __init__(self, motorLeft, motorRight, sensorGyro):
        self.motorLeft = motorLeft
        self.motorRight = motorRight
        self.sensorGyro = sensorGyro

        self.dead_recController = DeadReckoningController()

    def update():

        posX, posY, yaw = self.dead_recController.getPosition()

        # first rotate to the expected rotation
        yaw_error = 
        yaw_drive = self.PIDController_yaw.updatePosition()

        # then drive to position
