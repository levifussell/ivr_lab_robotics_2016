from DeadReckoningController import DeadReckoningController
from LineFollowController import LineFollowController
from LineSearcherController import LineSearcherController
from ObstacleDetectController import ObstacleDetectController

from Controller import Controller

from Robot import RobotState

import time

class PlantController(Controller):

    def __init__(self, robot, useLineFollow=True, useLineSearch=True, useObstacleDetect=True, actWhenOffLine=True, lineSensitivity=23):
        Controller.__init__(self, robot)

        # self.dead_recController = DeadReckoningController(robot)

        self.useLineFollow = useLineFollow
        self.useLineSearch = useLineSearch
        self.useObstacleDetect = useObstacleDetect

        if self.useLineFollow:
            self.line_followController = LineFollowController(robot, actWhenOffLine=actWhenOffLine, lineSensitivity=lineSensitivity)

        if self.useLineSearch:
            self.line_searchController = LineSearcherController(robot)

        if self.useObstacleDetect:
            self.obstacle_detectController = ObstacleDetectController(robot)

    def timestamp_now (self): return int (time.time () * 1E3)

    def update(self):

        # self.dead_recController.update()


        if self.useLineFollow:
            self.line_followController.update()

        if self.useLineSearch:
            self.line_searchController.update()

        if self.useObstacleDetect:
            self.obstacle_detectController.update()


    def taskOver(self):
        return self.robot.state == RobotState.DEAD

        # posX, posY, yaw = self.dead_recController.getPosition()
        #
        # # first rotate to the expected rotation
        # yaw_error =
        # yaw_drive = self.PIDController_yaw.updatePosition()

        # then drive to position

    # def newRelativeTarget(self, d_x, d_y):
    #     self.dead_recController.newRelativeTarget(d_x, d_y)
