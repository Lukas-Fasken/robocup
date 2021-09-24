#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.B) #Left motor in port B
right_motor = Motor(Port.C) # Right motor in port C
claw_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE) #Claw motor in port A
line_sensor = ColorSensor(Port.S2) #Line sensor in port
#push_sensor = TouchSensor(Port.S3)
ultra_sensor = UltrasonicSensor(Port.S3)
gyro_sensor = GyroSensor(Port.S1) #Gyro sensor in port 1

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=50, axle_track=138)

timer = StopWatch()


#grå 30 25 26    // 48 49
#hvid 52 46 65   // 86 87
#sort 6 9        // 9 8

GREY = 48
WHITE = 80
BLACK = 15

threshold = (GREY + WHITE) / 2

def follow_line(DRIVE_SPEED=150, P_GAIN=1.2, direction=1, breakable=0):
    #direction 1 gives bias to the right side, 0 to the left side
    i = 0
    #Drive_speed is in mm pr second
    
    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.

    # Set the gain of the proportional line controller. This means that for every
    # percentage point of light deviating from the threshold, we set the turn
    # rate of the drivebase to 1.2 degrees per second.

    # For example, if the light value deviates from the threshold by 10, the robot
    # steers at 10*1.2 = 12 degrees per second.

    # Start following the line endlessly.

    while i == 0:
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = P_GAIN * deviation * direction

        # Set the drive base speed and turn rate.
        robot.drive(DRIVE_SPEED, turn_rate)
        if breakable == 1:
            break
    
        if line_sensor.reflection() < BLACK:
            i = 1

def switch_lane(TURN_SIDE='RIGHT'): #BASIC CODE FOR CHANGING LANE - USED IN SEKVENS 1 & 4

    i=0

    """
    switch_lane is used to change lanes, by ignoring other lanes.
    Defined by TURN_SIDE, it'll turn either 45 degrees right, left or not at all.

    An idea might be to add at "forwards or backwards" argument, so it can function for sequence 5 as well.
    """

    robot.straight(50)
    #Following lines defines which way to turn. If no side is given, it'll beep and stop the function
    while True:
        if TURN_SIDE == 'RIGHT':
            turnRight(TURNANGLE = 45)
        elif TURN_SIDE == 'LEFT':
            turnLeft(TURNANGLE = 45)
        else:
            break
        find_color(COLOR ='WHITE')
        find_color()
        print('Out of the loop')
        robot.stop()
        print('stopped ok')

        robot.straight(80)
        print('8cm')

        
        if TURN_SIDE == 'RIGHT':
            print('correction left')
            turnLeft(TURNANGLE = 45)
        elif TURN_SIDE == 'LEFT':
            print('correcting right')
            turnRight(TURNANGLE = 45)
        else:
            print('not correcting')
        
        print('breaking')
        break
    
def pick_up():
    claw_motor.run_until_stalled(100, then=stop.HOLD, duty_limit=50)

def sekvens1():
    switch_lane(TURN_SIDE='RIGHT')
    follow_line(P_GAIN=1.2)

def sekvens2():
    switch_lane(TURN_SIDE='LEFT')
    follow_line(DRIVE_SPEED=150, P_GAIN=2.5)

    """
    if line_sensor.reflection() <60:
        turnrate=2
    elif line_sensor.reflection() > 60:
        turnrate=1
    follow_line(DRIVE_SPEED=150,P_GAIN=turnrate,)
    """
    print('finished seq 2')

def sekvens3():
    
    '''
    The sequence used by the robot to finish task 3, the waterbottle task
    '''

    robot.stop()
    gyro_sensor.reset_angle(0)
    sensorVar = ultra_sensor.distance()
    print(sensorVar)
    print('after sensor var')

    turnRight(TURNANGLE=45)
    print('after turn succes')
    robot.straight(50)
    print('driving 10cm')

    find_color(COLOR='WHITE')
    find_color(COLOR='GREY')
    find_color(COLOR='WHITE')
    robot.stop()
    turnRight(TURNANGLE = 45)

    approch_bottle()
    robot.stop()

    grab()

    robot.drive(100,0)
    while line_sensor.reflection() > BLACK:
        wait(1)
    robot.stop()

    grab(direction = 'DOWN')

    robot.straight(-100)
    """

    approch_bottle(ultra_sensor.distance())

    grab()
    """
    turnLeft(TURNANGLE = 135)
    find_color()
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = 45)
    follow_line()

def sekvens3reserve():
    robot.straight(100)
    follow_line()

def sekvens4():
    """
    robot.straight(-100)

    find_color(SPEED = - 150, COLOR='WHITE')

    turnLeft(ROTATION_SPEED = 60, TURNANGLE = 180)

    robot.drive(150,0)
    grey_counter(NUMBER = 4)

    turnRight(TURNANGLE = 5)

    grab(ROTATIONS = -2)

    robot.straight(-500)

    turnLeft(ROTATION_SPEED = 60, TURNANGLE = 180)

    find_color()

    turnLeft(TURNANGLE = 85)

    follow_line()
    """
    wait(1)

def grey_counter(NUMBER = 1):
    i = 0
    while i <= NUMBER:
        if line_sensor.reflection() < 60:
            if previous_grey == False:
                i = i + 1
            previous_grey = True
        else:
            previous_grey = False

    #claw_motor.run_until_stalled(100, then=stop.HOLD, duty_limit=50)
    claw_motor.run_angle(1000,4000)

def find_color(SPEED = 150, COLOR = 'GREY', TURNANGLE = 0):
    robot.drive(SPEED,TURNANGLE) #drive with 360deg/sec
    if COLOR == 'WHITE':
        while line_sensor.reflection() < 70:
            #print('LINESENSOR (WHITE)')
            #print(line_sensor.reflection())
            wait(1)
    
    elif COLOR == 'GREY':
        robot.drive(SPEED,TURNANGLE) #drive with 360deg/sec
        while line_sensor.reflection() > 55:
            #print('LINESENSOR (GREY)')
            #print(line_sensor.reflection())
            wait(1)

def sekvens5():
    print('entered 5')
    turnLeft(TURNANGLE=45)
    find_color(COLOR = 'WHITE')
    find_color()
    turnLeft(TURNANGLE=25)
    follow_line()

def sekvens6():
    timer.reset()
    robot.straight(100)
    while timer.time() < 12000:
        follow_line(DRIVE_SPEED = 150, breakable = 1)
        print(timer.time())
    reset_time = timer.time()

    while True:
        follow_line(DRIVE_SPEED = 100, breakable = 1)
        current_time = timer.time()
        if line_sensor.reflection() < 65:
            print(line_sensor.reflection())
            print('Color is grey')
            reset_time = timer.time()
        elif current_time - reset_time > 2000:
            print('WHITE!')
            turnLeft(TURNANGLE = 125)
            break
        print(timer.time())
    
    find_color()
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = 45)
    follow_line()

def sekvens7():
    current_time = timer.time()
    while timer.time() - current_time < 6000:
        follow_line(DRIVE_SPEED = 100, P_GAIN= 0.8, breakable=1, direction=1)
    turnLeft(TURNANGLE = 45)
    find_color(COLOR = 'WHITE')
    find_color()
    find_color(COLOR = 'WHITE')
    find_color()
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = 45) 

    current_time = timer.time()
    while timer.time() - current_time < 3000:
        follow_line(P_GAIN=1.5)
    follow_line(P_GAIN = 2.5)

def sekvens8():
    robot.straight(50)
    follow_line()

def sekvens9():
    wait(1)

def sekvens10(version='normal'):
    if version == 'normal':
        prin('10, normal')
        around_bottle(rotation='RIGHT')
        robot.straight(100)
        follow_line()
    elif version == 'ShitsAndGiggles':
        print('10, SAG')
        turnRight(TURNANGLE = 40)
        robot.straight(100)
        find_color()
        robot.stop()
        find_color(COLOR = 'WHITE')
        robot.stop()
        turnRight(TURNANGLE = 110)
        follow_line(direction= -1,DRIVE_SPEED = 100, P_GAIN = 1.5)

def sekvens11():
    robot.stop()
    grab(rotations=15)
    print('liftup')
    robot.straight(600)
    turnLeft(TURNANGLE=45)
    print('left90')
    robot.straight(350)
    turnRight(TURNANGLE=90)
    print('right90')
    robot.straight(300)
    turnLeft(TURNANGLE=45)
    print('left45')
    find_color()
    print('find grey')
    robot.stop()
    grab(direction='DOWN', rotations=15)
    print('de-claw')
    follow_line()

def sekvens12():
    around_bottle(rotation='RIGHT')
    print('11, normal')
    robot.straight(50)
    around_bottle(rotation = 'LEFT')
    follow_line(P_GAIN=2.5)

def approch_bottle(sensorVar = 1000):
    i = 0

    while sensorVar > 100:
        print('Start quick approch')
        sensorVar = ultra_sensor.distance()
        # Calculate the deviation from the threshold.
        deviation = line_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = 1.2 * deviation*-1
        # Set the drive base speed and turn rate.
        drive_speed_bottle = (sensorVar)/5
        robot.drive(drive_speed_bottle, turn_rate)
        print(sensorVar)

    print('Turning left')
    turnLeft(TURNANGLE = 5)
    
    print('start slow approch')
    while sensorVar > 55:
        print(sensorVar)
        sensorVar = ultra_sensor.distance()
        drive_speed_bottle = (sensorVar)/10
        robot.drive(drive_speed_bottle, 0)

def around_bottle(rotation='LEFT'):
    if rotation == 'LEFT':
        turnLeft(TURNANGLE=45)
        find_color(TURNANGLE = 15)
    elif rotation == 'RIGHT':
        turnRight(TURNANGLE=45)
        find_color(TURNANGLE = -15)

def loop(): #MAIN CODE - THIS CODE CALLS THE SUBFUNCTIONS
    follow_line()
    for sekvens in range(1,13):
        print(sekvens)

        if sekvens == 1:
            sekvens1()  

        elif sekvens == 2:
            sekvens2()        

        elif sekvens == 3:
            sekvens3()         

        elif sekvens == 4:
            sekvens4()            

        elif sekvens == 5:
            sekvens5()

        elif sekvens == 6:
            sekvens6()

        elif sekvens == 7:
            sekvens7()

        elif sekvens == 8:
            sekvens8()

        elif sekvens == 9:
            sekvens9()
        
        elif sekvens == 10:
            sekvens10(version = 'ShitsAndGiggles')
            
        elif sekvens == 11:
            sekvens11()
            
        elif sekvens == 12:
            sekvens12()
            
        elif sekvens == 13:
            sekvens13()

        else:
            ev3.speaker.play_file(SoundFile.FANFARE)

def turnRight(TURNANGLE = 90, ROTATION_SPEED = 75):
    ang = 0
    gyro_sensor.reset_angle(0)
    robot.drive(0,ROTATION_SPEED)
    print('begin right turn')

    while  ang < TURNANGLE:
        ang = gyro_sensor.angle()
    print('finished right turn')
    robot.stop()

def turnLeft(TURNANGLE = 90, ROTATION_SPEED = 75):
    ang = 0
    gyro_sensor.reset_angle(0)
    robot.drive(0,-ROTATION_SPEED)
    print('begin left turn')

    while  ang > -TURNANGLE:
        ang = gyro_sensor.angle()
    print('finished left turn')
    robot.stop()

def grab(SPEED=1000, rotations=11, direction = 'UP'):
    claw_motor.reset_angle(0)
    if direction == 'DOWN':
        SPEED = SPEED *-1
    claw_motor.run_angle(speed=SPEED, rotation_angle=360*rotations,wait=True)

def line_test():
    while True:
        deviation = line_sensor.reflection()
        print(deviation)

def angle_test():
    for i in range(0,180,15):
        gyro_sensor.reset_angle(0)
        robot.turn(i)
        ang = gyro_sensor.angle()

def ultra_test():
    while True:
        sensorVar = ultra_sensor.distance()
        print(sensorVar)

def test_loop(): #MAIN CODE - THIS CODE CALLS THE SUBFUNCTIONS
    follow_line()
    for sekvens in range(10,13):
        print(sekvens)      
        
        if sekvens == 3:
            sekvens3()         

        elif sekvens == 4:
            sekvens4()            
        
        elif sekvens == 5:
            sekvens5()

        elif sekvens == 6:
            sekvens6()

        elif sekvens == 7:
            sekvens7()

        elif sekvens == 8:
            sekvens8()

        elif sekvens == 9:
            sekvens9()
        
        elif sekvens == 10:
            sekvens10(version = 'ShitsAndGiggles')
            
        elif sekvens == 11:
            sekvens11()
            
        elif sekvens == 12:
            sekvens12()
            
        elif sekvens == 13:
            sekvens13()

        else:
            ev3.speaker.play_file(SoundFile.FANFARE)

def grab_const(SPEED=1000, rotations=11, direction = 'UP'):
    claw_motor.reset_angle(0)
    if direction == 'DOWN':
        SPEED = SPEED *-1
    claw_motor.run_angle(speed=SPEED, rotation_angle=360*rotations)
    claw_motor.run_time(speed=50, time=5000, wait=False)

def grabtest():
    grab_const()
    wait(5000)
    grab(direction = 'DOWN')

#grabtest()

loop()

#test_loop()

#ultra_test()

#angle_test()

#line_test()

#turnLeft()     

#follow_line(DRIVE_SPEED=150, P_GAIN=2.5, direction=-1)
