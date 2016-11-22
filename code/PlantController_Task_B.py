from PlantController import PlantController

from Robot import Robot, RobotState

import ev3dev.ev3 as ev3

import time

class PlantController_Task_B(PlantController):

    def __init__(self):

        robot = Robot()

        PlantController.__init__(self, robot, useLineFollow=True, useLineSearch=True, useObstacleDetect=False, actWhenOffLine=True)

        self.begin()

    def begin(self):
        t = self.timestamp_now()

        btn = ev3.Button()

        while (self.timestamp_now () - t < 70000) and (not btn.backspace) and (not self.taskOver()):
            self.update()

    def update(self):

        super(PlantController_Task_B, self).update()

        # if the robot has moved between lines 4 times and left the final line, the task is complete
        if self.line_searchController.numLinesMoved == 4 and self.robot.state == RobotState.LINE_FOLLOW:
            print('TASK B COMPLETE!!!')
            self.robot.setState(RobotState.DEAD)
