
import ev3dev.ev3 as ev3

class Robot:

    def __init__(self):

        self.motorLeft = ev3.LargeMotor('outB')
        self.motorRight = ev3.LargeMotor('outC')
        self.sensorGyro = ev3.GyroSensor()
        self.sensorSonar = ev3.SonarSensor()
        self.sensorGyro.mode = 'GYRO-ANG'

    def getGyroValue(self):
        return self.sensorGyro.value()

    def getSonarValue(self):
        return self.sensorSonar.value()

    def getAvgEncoderValue(self):
        return (self.motorLeft.position + self.motorRight.position) / 2
