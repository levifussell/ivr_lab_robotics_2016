# scp -r Documents/ivr_2016/robotics/tests/ivr_robotics/ robot@192.168.17.129:~/


#! /usr/bin/env python
# Core imports
import time
import ev3dev.ev3 as ev3

from PlantController import PlantController
from Robot import Robot

def timestamp_now (): return int (time.time () * 1E3)
# Local Imports
# import openLoopControl as olc

# print ('Welcome to ev3')

# ev3.Sound.speak('Hello is it me your loo king for').wait()

# test1.driveWiggleTest()
# test1.servoRangeTest()
# test1.driveWiggleTest()
# test1.driveAndRecordUltrasonic()
# test1.controlMain()
# motorB = ev3.LargeMotor('outB')
# motorC = ev3.LargeMotor('outC')
# gyro = ev3.GyroSensor()
# gyro.connected
# gyro.mode = 'GYRO-ANG'
#
# startval = gyro.value()

robot = Robot()

robotPlant = PlantController(robot)#motorB, motorC, gyro)
robotPlant.newRelativeTarget(30, 9)

t = timestamp_now ()
while timestamp_now () - t < 20000:
    robotPlant.update()

# motorB.run_timed(duty_cycle_sp=50, time_sp=2000)
# motorB_startPos = motorB.position
# motorC.run_timed(duty_cycle_sp=50, time_sp=2000)
# motorC_startPos = motorC.position

# t = timestamp_now ()
# while timestamp_now () - t < 2500:
#     print(gyro.value())
# time.sleep(2)
#
# diffPosB = motorB.position - motorB_startPos
# diffPosC = motorC.position - motorC_startPos
# print('final: B:{}, C:{}'.format(diffPosB, diffPosC))
#
# motorB.stop(stop_action='brake')
# motorC.stop(stop_action='brake')


#
# Step A: Basic open driving
# tutorial.operateWheelsBasic()

# Step B: Turn on an off an LED using a switch
# tutorial.makeLightSwitch()

# Step C: Use switches to drive robot back and forward
#tutorial.makeLightAndMotorSwitch()


# Step D: Use a class to develop a bigger program with a state
# o = olc.openLoopControl()
## execute (with default params)
# o.operateWheels()
#
## update parameters
# o.time_to_spin = 3.0
# o.duty_cycle_sp = 70
#
## execute again
# o.operateWheels()

# Step E: Record values from the ultrasonic to a text file
# tutorial.recordUltraSonic()

# remove this if you want it to exit as soon as its done:
print("wait 1sec, then end")
time.sleep(1)
