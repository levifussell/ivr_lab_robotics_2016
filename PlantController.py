from DeadReckoningController import DeadReckoningController

from Controller import Controller

class PlantController(Controller):

    def __init__(self, robot):
        Controller.__init__(self, robot)

        self.dead_recController = DeadReckoningController(robot)

    def update(self):
        self.dead_recController.update()
        # posX, posY, yaw = self.dead_recController.getPosition()
        #
        # # first rotate to the expected rotation
        # yaw_error =
        # yaw_drive = self.PIDController_yaw.updatePosition()

        # then drive to position

    def newRelativeTarget(self, d_x, d_y):
        self.dead_recController.newRelativeTarget(d_x, d_y)
