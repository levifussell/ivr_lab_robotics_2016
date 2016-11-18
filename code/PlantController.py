from DeadReckoningController import DeadReckoningController
from LineFollowController import LineFollowController
from ObstacleDetectController import ObstacleDetectController

from Controller import Controller

from Robot import RobotState

class PlantController(Controller):

    def __init__(self, robot):
        Controller.__init__(self, robot)

        self.dead_recController = DeadReckoningController(robot)

        self.line_followController = LineFollowController(robot)

        self.obstacle_detectController = ObstacleDetectController(robot)

    def update(self):

        # self.dead_recController.update()

        self.line_followController.update()

        self.obstacle_detectController.update()

        lightReadings = ""
        lightReadings_file = open('lightReadings.txt', 'w')

        for i in range(0, len(self.line_followController.positionTracer)):
            lightReadings += str(self.line_followController.positionTracer[i]) + ','

        lightReadings_file.write(lightReadings)
        lightReadings_file.close() # Will write to a text file in a column

        # posX, posY, yaw = self.dead_recController.getPosition()
        #
        # # first rotate to the expected rotation
        # yaw_error =
        # yaw_drive = self.PIDController_yaw.updatePosition()

        # then drive to position

    def newRelativeTarget(self, d_x, d_y):
        self.dead_recController.newRelativeTarget(d_x, d_y)
