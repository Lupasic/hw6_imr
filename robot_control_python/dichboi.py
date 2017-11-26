#!/usr/bin/env python3
import ev3dev.ev3 as ev3
from time import sleep

# devices
btn = ev3.Button()
leftSonar = ev3.UltrasonicSensor('in2')
rightSonar = ev3.UltrasonicSensor('in1')
forwardSonar = ev3.UltrasonicSensor('in3')
gyro = ev3.GyroSensor('in4')

leftWheel = ev3.LargeMotor('outB')
rightWheel = ev3.LargeMotor('outC')

# parameters
fullSpeed = 600

# inital_values
der = 0;
integ = 0;


writeFile = open('data.txt', 'w')
writeFile.write("Start\n")

# def turnSide(delta):
#     print("Dich move")
#     leftWheel.run_forever(speed_sp=fullSpeed)
#     rightWheel.run_forever(speed_sp=fullSpeed)
#     sleep(1)
#     if delta<0:
#         leftWheel.run_forever(speed_sp=0)
#         rightWheel.run_forever(speed_sp=fullSpeed)
#     else:
#         leftWheel.run_forever(speed_sp=fullSpeed)
#         rightWheel.run_forever(speed_sp=0)
#     sleep(0.75)
#     leftWheel.run_forever(speed_sp=fullSpeed)
#     rightWheel.run_forever(speed_sp=fullSpeed)
#     sleep(2)


# one_step
def turn(kp=3, kd=20, ki=0.001):
    global der, integ

    # measure_the_distance
    leftData=leftSonar.distance_centimeters
    rightData=rightSonar.distance_centimeters
    forwardData = forwardSonar.distance_centimeters
    delta = rightData - leftData

    writeFile.write("L "+str(leftData) + " R " + str(rightData) + " F " + str(forwardData) + " G " + str(gyro.angle) +  '\n')


    extra_rot_coef = 150
    # find_error

    err = delta
    if err > 50:
        err = 50
    w=0
    print('Delta %.1f' % (err))
    if forwardData<12:

        if leftData < rightData:
            print("Wall in front, turn right")
            w = extra_rot_coef
        else:
            print("Wall in front, turn left")
            w = -extra_rot_coef
    #    pass
    # calculate proportional element

    P = kp * err

    # calculate derivative element

    D = kd * (err - der)
    der = err

    # calculate integral element

    integ += err
    I = - ki * integ

    # calculate full correction

    w += P + I + D

    print('PID param result %.1f' % (w))
    # calculate left wheel speed

    left = int(fullSpeed / 2 + w)
    if left > fullSpeed:
        left = fullSpeed
    if left < 0:
        left = 0

        # calculate right wheel speed

    right = fullSpeed - left
    writeFile.write("LW perM " + str(leftWheel.count_per_rot) + " full " + str(leftWheel.position) + " speed "+left+"\n")
    writeFile.write("RW perM " + str(rightWheel.count_per_rot) + " full " + str(rightWheel.position) + " speed " + right + "\n")
    # set wheel speed

    leftWheel.run_forever(speed_sp=left)
    rightWheel.run_forever(speed_sp=right)

# while loop


while not btn.any():
    turn()
    sleep(0.2)

# stop

leftWheel.stop()
rightWheel.stop()