
import ev3dev.ev3 as ev3

class Robot:

    def __init__(self):

        self.motorLeft = ev3.LargeMotor('outB')
        self.motorRight = ev3.LargeMotor('outD')
        self.sensorLight = ev3.ColorSensor(ev3.INPUT_1)
        self.sensorLight.mode='COL-REFLECT'
        self.sensorGyro = ev3.GyroSensor(ev3.INPUT_2)
        # self.sensorSonar = ev3.SonarSensor()
        self.sensorGyro.mode = 'GYRO-ANG'

    def driveMotors(self, powerLeft, powerRight):
        self.driveLeftMotor(powerLeft)
        self.driveRightMotor(powerRight)

    def driveLeftMotor(self, power):
        self.motorLeft.run_timed(duty_cycle_sp=power, time_sp=500)

    def driveRightMotor(self, power):
        self.motorRight.run_timed(duty_cycle_sp=power, time_sp=500)

    def getLightValue(self):
        return self.sensorLight.value()

    def getGyroValue(self):
        return self.sensorGyro.value()

    # def getSonarValue(self):
    #     return self.sensorSonar.value()

    def getAvgEncoderValue(self):
        return (self.motorLeft.position + self.motorRight.position) / 2
