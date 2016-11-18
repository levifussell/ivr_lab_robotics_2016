from Controller import Controller
from DeadReckoningController import DeadReckoningController

from Robot import RobotState

from enum import Enum

# class ObstacleScanDirection(Enum):
#     LEFT = 1
#     RIGHT = 2

class ObstacleDetectController(Controller):

    def __init__(self, robot)RobotState:
        Controller.__init__(self, robot)

        self.deadRec_controller = DeadReckoningController(robot)

        self.PIDCOntroller_motorSonar = PIDController()

        self.previousSonarValue = self.robot.getSonarValue()
        self.SONARCHANGE_OBSTVALUE = 20
        # self.scanDirection = ObstacleScanDirection.LEFT
        # self.scanLeftValue = -1
        # self.scanRightValue = -1

    def update(self):

        # always update the dead reckoning controller
        self.deadRec_controller.update()

        # check if the ultrasonic has changed dramatically 
        if (abs(self.robot.getSonarValue() - self.previousSonarValue) < self.SONARCHANGE_OBSTVALUE) and (self.robot.setState != RobotState.OBSTACLE_DETECT):
            # object detected
            self.robot.setState = RobotState.OBSTACLE_DETECT

        if self.robot.state == RobotState.OBSTACLE_DETECT:
            d_middle = 0

            targetOffset = 0
            if self.scanDirection == ObstacleScanDirection.LEFT:
                # move ultrasonic to the right and detect
                targetOffset = 90 #encoder value that corresponds to 90 degrees (TODO)
            elif self.scanDirection == ObstacleScanDirection.RIGHT:
                # move ultrasonic to the left and detect
                targetOffset = -90 #encoder value that corresponds to -90 degrees (TODO)

            targetTurn = self.robot.getMiddleAvgEncoderValue() + targetOffset 
            turn_error = targetTurn - self.robot.getMiddleEncoderValue()
            d_middle = self.PIDCOntroller_motorSonar(turn_error, self.robot.getMiddleEncoderValue())

            if turn_error < 1:
                if self.scanDirection == ObstacleScanDirection.LEFT:
                    self.scanLeftValue = self.robot.getSonarValue()
                    self.scanDirection = ObstacleScanDirection.RIGHT
                elif self.scanDirection == ObstacleScanDirection.RIGHT:
                    self.scanRightValue = self.robot.getSonarValue()

            self.__drive(d_middle)

    def __drive(self, d_middle):

        self.robot.motorMiddle.run_timed(duty_cycle_sp=d_middle, time_sp=50)

    