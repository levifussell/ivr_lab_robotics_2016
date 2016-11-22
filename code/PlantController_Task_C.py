from PlantController import PlantController

from Robot import Robot, RobotState

import ev3dev.ev3 as ev3

import time

class PlantController_Task_C(PlantController):

    def __init__(self):

        robot = Robot()

        PlantController.__init__(self, robot, useLineFollow=True, useLineSearch=True, useObstacleDetect=True, actWhenOffLine=False)

        self.begin()

    def begin(self):
        t = self.timestamp_now()

        btn = ev3.Button()

        while (self.timestamp_now () - t < 90000) and (not btn.backspace) and (not self.taskOver()):
            self.update()

    def update(self):

        super(PlantController_Task_C, self).update()

        # unknown at the moment (maybe have to use dead reckoning system??)
        # if self.line_searchController.numLinesMoved == 4 and self.robot.state == RobotState.OFF_LINE:
        #     print('TASK B COMPLETE!!!')
        #     self.robot.setState(RobotState.DEAD)
