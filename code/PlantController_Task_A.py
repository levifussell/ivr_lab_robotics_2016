from PlantController import PlantController

from Robot import Robot, RobotState

import ev3dev.ev3 as ev3

import time

class PlantController_Task_A(PlantController):

    def __init__(self):

        robot = Robot()

        PlantController.__init__(self, robot, useLineFollow=True, useLineSearch=False, useObstacleDetect=False, actWhenOffLine=True, lineSensitivity=28)

        self.begin()

    def begin(self):
        t = self.timestamp_now()

        btn = ev3.Button()

        while (self.timestamp_now() - t < 5000) and (not btn.backspace) and (not self.taskOver()):
        # while (self.timestamp_now() - t < 60000) and (not btn.backspace) and (not self.taskOver()):
            self.update()

        lightReadings = ""
        lightReadings_file = open('lightHistogram.txt', 'w')

        for i in range(0, len(self.line_followController.positionTracer)):
            lightReadings += str(self.line_followController.positionTracer[i]) + ','

        lightReadings_file.write(lightReadings)
        lightReadings_file.close()

    def update(self):

        super(PlantController_Task_A, self).update()

        # if the robot has left the line, the task is complete
        if self.robot.state == RobotState.OFF_LINE:
            print('TASK A COMPLETE!!!')
            self.robot.setState(RobotState.DEAD)
