#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from threading import Thread


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

GREY = 49
WHITE = 80
BLACK = 13

threshold = (GREY + WHITE) / 2

ev3.speaker.set_volume(volume=150, which= '_all_')

def sequence1(): # SWITCH LANE TWICE
    """
    This sequence 
    """
    switch_lane(TURN_SIDE='RIGHT')
    follow_line(SIDE = 'RIGHT', P_GAIN = 1.2)
    switch_lane(TURN_SIDE='LEFT')

def sequence2(): # MOVE THE BOTTLE  
    robot.stop()
    gyro_sensor.reset_angle(0)
    sensorVar = average_ultra_sensor()
    print(sensorVar)
    print('after sensor var')

    turnRight(TURNANGLE=25)
    print('after turn succes')
    robot.straight(50)
    print('driving 5cm')

    find_color(COLOR='WHITE')
    find_color(COLOR='GREY')
    find_color(COLOR='WHITE')
    robot.straight(30)

    turnRight(TURNANGLE = 55)

    fail = approach_bottle()

    robot.stop()
    
    grab()
    find_color(COLOR = 'WHITE')

    robot.drive(100,0)
    while average_line_sensor() > BLACK:
        wait(1)
    robot.stop()

    grab(DIRECTION = 'DOWN')

    robot.straight(-100)
    robot.stop()
    turnLeft(TURNANGLE = 155)
    robot.straight(100)
    find_color()
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = 55)

def sequence3(): # OVER THE BRIDGE
    """
    This sequence is split up into four sub-parts
    """
    print('entered 3')
    robot.stop()

    robot.straight(50)
    follow_turn(TURNANGLE=180, TURNSIDE='LEFT', FOLLOW_TURN_P_GAIN= 2.5, FOLLOW_TURN_SPEED= 100)
    #grab(ROTATIONS=15)
    """
    turnLeft(TURNANGLE=45)
    find_color(COLOR = 'WHITE')
    find_color()
    find_color(COLOR = 'WHITE')
    robot.straight(30)
    turnLeft(TURNANGLE=45)
    robot.stop()
    robot.straight(50) #changed

    #Timer is reset, and the robot follows the line for 15 seconds, approximately the time it takes to cross the bridge
    timer.reset() #removes robot from black line, as not to trigger it twice.
    while timer.time() < 15000: #17000 works
        follow_line(SPEED = 150, BREAKABLE = 1, P_GAIN = 1, SIDE = 'RIGHT') #follows the line, while watching for time
        print(timer.time())


    #Once the brigde has been crossed, the robot moves forwards, untill it sees whtie continuesly for 2 seconds.
    reset_time = timer.time()
    while True:
        follow_line(SPEED = 80, BREAKABLE = 1, SIDE = 'RIGHT', P_GAIN = 1.5)
        current_time = timer.time()
        if average_line_sensor() < threshold: #If the robot sees a grey line, it resets the time
            #print(line_sensor.reflection())
            #print('Color is grey')
            reset_time = timer.time()
            print('reset.timer')
        elif current_time - reset_time > 3000: #After seing white for 2 seconds, the robot turns 125degrees, and goes back.
            print('WHITE!')
            turnLeft(TURNANGLE = 125)
            break
        print(current_time - reset_time)
    
    #Find the grey line, cross it, turn towards the next obstacle, and follow the line.
    find_color()
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = 45)
    """

def sequence4(): # CROSS LINES
    current_time = timer.time()
    while timer.time() - current_time < 6000:
        follow_line(SPEED = 150, P_GAIN= 0.8, BREAKABLE=1)
    turnLeft(TURNANGLE = 45)
    find_color(COLOR = 'WHITE')
    find_color()
    find_color(COLOR = 'WHITE')
    find_color()
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = 45) 

    current_time = timer.time()
    while timer.time() - current_time < 3000:
        follow_line(P_GAIN=1.5, BREAKABLE = 1)
        wait(1)

def sequence5(): # DARTBOARD
    fail = 0

    turnLeft(TURNANGLE = 45)
    robot.straight(50)
    find_color()
    find_color(COLOR = 'WHITE')
    turnLeft(TURNANGLE = 35)

    follow_line(P_GAIN = 0.8, SIDE = 'RIGHT', SPEED = 75)

    robot.straight(600)
    robot.reset()
    robot.stop()
    turnLeft(TURNANGLE = 30)
    robot.straight(400)
    robot.stop()
    average_dist = 1000
    find_bottle(FULL_ANGLE = 60)
    reset_time = timer.time()
    while average_dist > 175:
        average_dist = average_ultra_sensor()
        robot.drive(50,0)
        current_time = timer.time() - reset_time
        if current_time >5000:
            fail = 1
            break
    #find_bottle(FULL_ANGLE = 30)
    reset_time = timer.time()
    while average_dist > 45:
        average_dist = average_ultra_sensor()
        robot.drive(50,0)
        current_time = timer.time() - reset_time
        if current_time > 5000:
            fail = 1
            break
    robot.stop()
    if fail == 0:
        grab(ROTATIONS=11.5)
        robot.straight(-(robot.distance()+130))
        grab(DIRECTION='DOWN', ROTATIONS=11.5)
        robot.straight(-600)
    else:
        robot.straight(-(robot.distance()+130))
        robot.straight(-600)
    turnLeft(TURNANGLE = 150)
    find_color()
    find_color(COLOR = 'WHITE')
    turnLeft(TURNANGLE = 80)

def sequence6(): # AROUND THE BOTTLE 1
    around_bottle(rotation='RIGHT')
    follow_turn(TURNANGLE=180, TURNSIDE='RIGHT', FOLLOW_TURN_P_GAIN= 2.5, FOLLOW_TURN_SPEED= 100)
    next_side= 'RIGHT'

def sequence7(): # THE LIBARY TASK

    robot.stop()
    grab(ROTATIONS=13)
    print('liftup')
    robot.straight(560)
    turnLeft(TURNANGLE=45)
    print('left45')
    robot.straight(340)
    #turnRight(TURNANGLE=90)
    for i in range(0,70,10):
        turnRight(TURNANGLE = 10)
        robot.straight(10)
    print('right 90')
    robot.straight(550)
    turnLeft(TURNANGLE=45)
    print('left45')
    angleTurned = find_color(TURNANGLE=-30)
    print('find grey')
    robot.stop()
    grab(DIRECTION='DOWN', ROTATIONS=13)
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = angleTurned)
    print('de-claw')

    '''
    
    robot.stop()
    grab(ROTATIONS=13)
    print('liftup')
    robot.straight(575)
    turnLeft(TURNANGLE=45)
    print('left45')
    robot.straight(350)

    turnRight(TURNANGLE=90)
    print('right 90')
    robot.straight(600)

    turnLeft(TURNANGLE=45)
    print('left45')
    angleTurned = find_color(TURNANGLE=-30)
    print('find grey')
    robot.stop()
    grab(DIRECTION='DOWN', ROTATIONS=13)
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = angleTurned)
    print('de-claw')
    '''

def sequence7test(): # THE LIBARY TASK

    robot.stop()
    grab(ROTATIONS=13)
    print('liftup')
    robot.straight(560)
    turnLeft(TURNANGLE=45)
    print('left45')
    robot.straight(320)
    #turnRight(TURNANGLE=90)
    for i in range(0,70,10):
        turnRight(TURNANGLE = 10)
        robot.straight(15)
    print('right 90')
    robot.straight(550)
    turnLeft(TURNANGLE=45)
    print('left45')
    angleTurned = find_color(TURNANGLE=-30)
    print('find grey')
    robot.stop()
    grab(DIRECTION='DOWN', ROTATIONS=13)
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = angleTurned)
    print('de-claw')

    '''
    
    robot.stop()
    grab(ROTATIONS=13)
    print('liftup')
    robot.straight(575)
    turnLeft(TURNANGLE=45)
    print('left45')
    robot.straight(350)

    turnRight(TURNANGLE=90)
    print('right 90')
    robot.straight(600)

    turnLeft(TURNANGLE=45)
    print('left45')
    angleTurned = find_color(TURNANGLE=-30)
    print('find grey')
    robot.stop()
    grab(DIRECTION='DOWN', ROTATIONS=13)
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = angleTurned)
    print('de-claw')
    '''

def sequence8(): # AROUND THE BOTTLE 2
    around_bottle(rotation='LEFT')
    follow_turn(TURNSIDE='LEFT', TURNANGLE=180, FOLLOW_TURN_P_GAIN=2.5,FOLLOW_TURN_SPEED=100)

def sequence9(): # THE LANDING STIP
    turnLeft(TURNANGLE = 45)
    robot. straight(50)
    find_color()
    find_color(COLOR = 'WHITE')
    turnRight(TURNANGLE = 45)

    robot.reset()
    sensorVar = 1000
    
    while sensorVar > 150:
        follow_line(SIDE = 'LEFT', BREAKABLE = 1)
        sensorVar = average_ultra_sensor()
    driven = robot.distance()
    print('Driven:')
    print(driven)
    robot.straight(-100)
    turnRight(TURNANGLE = 180)
    robot.reset()
    while robot.distance() < ((driven / 2) - 400):
        follow_line(SIDE = 'RIGHT', BREAKABLE = 1)
    turnLeft(TURNANGLE = 90)
    robot.straight(150)
    wait(20000)
    

def intenseThread():
    ev3.speaker.play_file(file="ShiverMeTimbers.wav")
    ev3.speaker.play_file(file="PirateSong.wav")

def follow_line(SPEED = 150, P_GAIN = 1.2, SIDE = 'LEFT', BREAKABLE = 0): #FOLLOW LINE, THE BASIC FUNCTION
    """
    Follow_line follows a grey line by continuesly calculating how far from the threshold between
    grey and white the color currently measured by the color sensor is, and correcting for that by 
    back towards the line. It drives the robot forwards untill a black line is found, or the robot 
    only have seen white for 3 seconds

    SIDE defines the side of the grey line the program has bias towards
    BREAKABLE defines whether the program continues to run until a black line, or only runs once
    P_GAIN defines the proportional value the turn factor is multiplied by
    #SIDE 'LEFT' gives bias to the left side of the line, 'RIGHT' to the right side
    """

    #Defines bias side
    if SIDE == 'RIGHT':
        direction = -1
    else:
        direction = 1
        
    reset_time = timer.time()

    #Following is the active drive loop
    while True:
        deviation = average_line_sensor() - threshold #Calculate the deviation from the threshold
        turn_rate = P_GAIN * deviation * direction    # Calculate the turn rate
        current_time = timer.time()                   #Sets timer to the current time

        if average_line_sensor() < threshold:         #resets the timer if the color scanned is grey
            #print(line_sensor.reflection())
            #print('Color is grey')
            reset_time = timer.time()
        elif current_time - reset_time > 10000:        #Starts panic routine after 10 seconds of white
            panic_routine(SIDE)

        robot.drive(SPEED, turn_rate)                 #Set the drivebase speed and turn rate.
        if BREAKABLE == 1:                            #Allows for continues reading of sensor values
            break
            
        if average_line_sensor() < BLACK:             #Breaks the program when a black line is found
            break

def follow_turn(TURNSIDE = 'LEFT', TURNANGLE = 0, FOLLOW_TURN_P_GAIN= 2.25, FOLLOW_TURN_SPEED= 150):
    gyro_sensor.reset_angle(0)
    while abs(gyro_sensor.angle()) < TURNANGLE*0.85:
        print(gyro_sensor.angle())
        print(abs(gyro_sensor.angle()))
        follow_line(SPEED= FOLLOW_TURN_SPEED, SIDE = TURNSIDE, P_GAIN = FOLLOW_TURN_P_GAIN,BREAKABLE = 1)
        if average_line_sensor() < BLACK:
            break

def turnRight(TURNANGLE = 90, ROTATION_SPEED = 75): # TURNS RIGHT, TURNANGLE DEGREES
    ang = 0
    gyro_sensor.reset_angle(0)
    robot.drive(0,ROTATION_SPEED)
    #print('begin right turn')

    while  ang < TURNANGLE:
        ang = gyro_sensor.angle()
    #print('finished right turn')
    robot.stop()

def turnLeft(TURNANGLE = 90, ROTATION_SPEED = 75): # TURNS LEFT, TURNANGLE DEGREES
    ang = 0
    gyro_sensor.reset_angle(0)
    robot.drive(0,-ROTATION_SPEED)
    #print('begin left turn')

    while  ang > -TURNANGLE:
        ang = gyro_sensor.angle()
    print('finished left turn')
    #robot.stop()

def grab(SPEED=1000, ROTATIONS=11.5, DIRECTION = 'UP'): # ROTATION CLAW_MOTOR, TO LIFT UP/DOWN CLAW
    claw_motor.reset_angle(0)
    if DIRECTION == 'DOWN':
        SPEED = SPEED *-1
    claw_motor.run_angle(speed=SPEED, rotation_angle=360*ROTATIONS,wait=True)

def average_ultra_sensor(): # TAKES 10 MEASUREMENTS OF ULTRASOUND SENSOR, AND RETURN THE AVERAGE VALUE
        ave_dist = 0
        meas = 10

        for i in range(meas):
            #print(ultra_sensor.distance())
            ave_dist = ave_dist + ultra_sensor.distance()
            #print(ave_dist)

        ave_dist = ave_dist / meas
        #print('Average distance is ')
        #print(ave_dist)
        return ave_dist

def average_line_sensor(): # TAKES 10 MEASUREMENTS OF COLOR SENSOR, AND RETURN THE AVERAGE VALUE
        ave_light = 0
        meas = 10

        for i in range(meas):
            ave_light = ave_light + line_sensor.reflection()

        ave_light = ave_light / meas
        
        #print(ave_light)

        return ave_light

def switch_lane(TURN_SIDE='RIGHT'): #BASIC CODE FOR CHANGING LANE - USED IN sequence 1 & 2

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
    
def panic_routine(SIDE = 'LEFT'): # ROUTINE DONE WHEN WHITE HAVENT BEEN FOUND FOR 5 SECODNS DURING FOLLOW LINE
    print('PANIK!!!')

    if SIDE == 'RIGHT':
        #på højre side af linjen
        for i in range (4):
            turnLeft(TURNANGLE = 90)
            find_color(TIMER = 2000)
            turnLeft(TURNANGLE = 180)
            find_color(TIMER = 2000)

    else:
        #på venstre side af linjen
        for i in range (4):
            turnLeft(TURNANGLE = 90)
            find_color(TIMER = 2000)
            turnLeft(TURNANGLE = 180)
            find_color(TIMER = 2000)

def find_color(SPEED = 150, COLOR = 'GREY', TURNANGLE = 0, TIMER = 0): #DRIVES STRAIGHT FORWARDS UNTIL IT SEE _COLOR
    """ 
    return_angle og TIMER kan IKKE bruges sammen!
    """
    gyro_sensor.reset_angle(0)
    reset_time = timer.time()

    robot.drive(SPEED,TURNANGLE) #drive with 360deg/sec
    if COLOR == 'WHITE':
        while average_line_sensor() < 70:
            #print('LINESENSOR (WHITE)')
            #print(average_line_sensor())
            current_time = timer.time()
            if (current_time - reset_time > TIMER) and (TIMER != 0):
                break
    
    elif COLOR == 'GREY':
        robot.drive(SPEED,TURNANGLE) #drive with 360deg/sec
        while average_line_sensor() > 55:
            #print('LINESENSOR (GREY)')
            #print(average_line_sensor())
            wait(1)
            current_time = timer.time()
            if (current_time - reset_time > TIMER) and (TIMER != 0):
                break
    angleTurned = gyro_sensor.angle()
    if (TURNANGLE != 0):
        return (abs(angleTurned)-20)

def find_bottle(FULL_ANGLE = 45, TURNSPEED = 1): #SWEEP OF ULTRASOUND TO FIND BOTTLE BY ROTATING
    angle = 0
    low_distance = 1000
    low_angle = 0
    turnLeft(TURNANGLE=FULL_ANGLE/2)
    for angle in range(0,FULL_ANGLE,TURNSPEED):
        turnRight(TURNANGLE = TURNSPEED, ROTATION_SPEED = 25 )
        sensorVar=average_ultra_sensor()
        if sensorVar < low_distance:
            low_distance = sensorVar
            low_angle = angle
            print(angle)
            print(gyro_sensor.angle())
    turnLeft(TURNANGLE=FULL_ANGLE-low_angle)
    """

    print(((-FULL_ANGLE)/2 + low_angle))

    return abs((-FULL_ANGLE)/2 + low_angle)
    """

    """
    angle = 0
    low_distance = 1000
    low_angle = 0

    for angle in range(0,FULL_ANGLE,TURNSPEED):
        if DIRECTION == 'RIGHT':
            turnRight(TURNANGLE = TURNSPEED)
        else:
            turnLeft(TURNANGLE = TURNSPEED)

        sensorVar = average_ultra_sensor()

        if sensorVar < low_distance:
            low_distance = sensorVar
            low_angle = angle

    if DIRECTION == 'RIGHT':
        turnLeft(TURNANGLE = (FULL_ANGLE-low_angle))
    else:
        turnRight(TURNANGLE = (FULL_ANGLE-low_angle))
        
    """

def approach_bottle(sensorVar = 1000): #APPROACHES BOTTLE WITH A SPEED DEFINED BY DISTANCE. 
    bottle_time = 7500
    print('Beginning fast approach!')
    reset_time=timer.time()
    i = 0


    while sensorVar > 150:
        sensorVar = average_ultra_sensor()
        follow_line(BREAKABLE = 1, SPEED = 50, P_GAIN = 2.2)
        current_time = timer.time() - reset_time
        if current_time >bottle_time:
            break
        wait(1)

    while sensorVar > 100:
        sensorVar = average_ultra_sensor()
        follow_line(BREAKABLE = 1, SPEED = 50, P_GAIN = 0.8)
        current_time = timer.time() - reset_time
        if current_time > bottle_time:
            break
        wait(1)

    average_dist = 100
    print('beginning slow approach' )
    while average_dist > 45:

        average_dist = average_ultra_sensor()
        robot.drive(50,0)
        current_time = timer.time() - reset_time
        if current_time >bottle_time:
            break

    #print('bottle has been found!')

def around_bottle(rotation='LEFT'): #GO AROUND THE BOTTLE (SEQUENCE 8 & 10)
    if rotation == 'LEFT':
        print('Around left')
        turnLeft(TURNANGLE=45)
        robot.straight(50)
        angleTurned = find_color(TURNANGLE = 25)
        print(angleTurned)
        turnLeft(angleTurned-10)
    elif rotation == 'RIGHT':
        print('Around right')
        turnRight(TURNANGLE=45)
        robot.straight(50)
        angleTurned = find_color(TURNANGLE = -25)
        print(angleTurned)
        turnRight(angleTurned-10)
    return angleTurned

def loop(): #MAIN CODE - THIS CODE CALLS THE SUBFUNCTIONS
    ST1 = Thread(target=intenseThread)
    wait(10000)
    follow_line(SIDE = 'RIGHT')
    for sequence in range(1,10):

        next_speed = 150
        next_side = 'RIGHT'
        next_breakable = 0
        next_P_GAIN = 1.2


        print('Next sequence: ')
        print(sequence)

        if sequence == 1: # SWITCH LANES
            sequence1()
            follow_turn(TURNSIDE = 'RIGHT', TURNANGLE = 180)
        
        elif sequence == 2: # MOVE BOTTLE
            sequence2()
            next_side = 'LEFT'

        elif sequence == 3: # OVER THE BRIDGE
            sequence3()
            next_side = 'LEFT'

        elif sequence == 4: #SWITCH LANES 
            sequence4()
            follow_turn(TURNSIDE = 'LEFT', TURNANGLE = 90)
            next_side = 'LEFT'    
        
        elif sequence == 5: # DARTBOARD
            sequence5()
            ST1.start()

        elif sequence == 6:
            sequence6()
            follow_turn(TURNSIDE = 'RIGHT', TURNANGLE = 135)

        elif sequence == 7:
            sequence7test()
            next_side = 'LEFT'

        elif sequence == 8:
            sequence8()
            follow_turn(TURNSIDE = 'LEFT', TURNANGLE = 135)
            
        elif sequence == 9:
            sequence9()
            ev3.speaker.play_file(SoundFile.FANFARE)

        else:
            ev3.speaker.play_file(SoundFile.ERROR_ALARM)
            break
        
        print('follow data:')
        print(next_side)
        print(next_P_GAIN)
        follow_line(next_speed, next_P_GAIN, next_side, next_breakable)

def grab_test():
    grab()
    wait(2000)
    grab(DIRECTION = 'DOWN')

def sound_test():
    ST1 = Thread(target=intenseThread)
    wait(5000)
    ST1.start()
    print('start')
    grab()
    grab(DIRECTION='DOWN')
    wait(100)
    ST1.stop()
    grab()
    grab(DIRECTION='DOWN')

def line_color():
    while True:
        average_line_sensor()
        print(average_line_sensor())
        wait(100)
        
#line_color()
loop()

#sound_test()
