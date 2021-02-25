# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import numpy as np
import math
from controller import Robot, Motor, DistanceSensor, GPS

# magnitude of vector
mod = lambda v : ( v[0]**2 + v[1]**2 + v[2]**2 ) ** 0.5

# dot product
dot = lambda v1, v2 : v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

# dot product but normalise first
normDot = lambda v1, v2 : ( v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2] ) / ( mod(v1) * mod(v2) )

# subtract 2 vectors
subv = lambda v1, v2 : [ v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2] ]

# origin and red square locations
origin      = [0, 0, 0]
redSquare   = [0, 0, 0.4]
x_bound     = 1.2
z_bound     = 1.2
sensor_dist = 0.225

# get robot and GPS
robot = Robot()
gps = GPS("gps")

# get the time step in milliseconds of the current world
timestep = int( robot.getBasicTimeStep() )

# dt in timestep in seconds 
dt = timestep / 1000

# get the motors and distance sensors
leftWheelMotor           = robot.getDevice( "LeftWheelMotor"           )
rightWheelMotor          = robot.getDevice( "RightWheelMotor"          )
clawMotor                = robot.getDevice( "ClawMotor"                )
frontLeftDistanceSensor  = robot.getDevice( "DistanceSensorFrontLeft"  )
frontRightDistanceSensor = robot.getDevice( "DistanceSensorFrontRight" )

# start the GPS
gps.enable( timestep )

# enable the distance sensors
frontLeftDistanceSensor.enable(  timestep )
frontRightDistanceSensor.enable( timestep )

# set the motors positions to infinity so we control only the velocity
leftWheelMotor.setPosition(  float('inf') )
rightWheelMotor.setPosition( float('inf') )
clawMotor.setPosition( -0.1 )

# Block detector to be used in while loop. Still need sensor range for it to work
def box_detection (norm_direction, coordinate, dist_limit, distanceSensed, sensor_dist = sensor_dist):

    object_position = norm_direction * (distanceSensed + sensor_dist) + coordinate
        
    if object_position[0] < 1.16 and object_position[1] < 1.16 and distanceSensed < sensorRange:
        return object_position
    else:
        return False


# variables that help the robot work
clawAngle    = -0.1
prevPos      = gps.getValues()
gettingBlock = False
haveBlock    = False

# Main loop
while robot.step(timestep) != -1:

    # get the position from the gps
    pos = gps.getValues()
    x, y, z = pos
    coordinate = np.array([x, z])
    x0, y0, z0 = prevPos
    
    d = np.array([x-x0, z-z0])
    norm_direction = d/((d[0]**2+d[1]**2)**0.5)


    # calculate velocity with single sided difference approximation
    vel = [ (a-b)/dt for a,b in zip(pos, prevPos) ]
    prevPos = pos

    # can print gps position or velocity
    print( "pos: " + " ".join( ["%.2f" % v for v in pos] ) )
    print( "vel: " + " ".join( ["%.2f" % v for v in vel] ) )

    if haveBlock:

        # get vector to red square
        toSquare = subv( redSquare, pos )

        # decide if we should turn
        shouldTurn = normDot( toSquare, vel ) < 0.9

        # turn to point at the square and drive
        leftWheelMotor.setVelocity(  20 - shouldTurn * 11 )
        rightWheelMotor.setVelocity( 20 - shouldTurn * 17 )

        # we are at the square if the distance to it is small
        atSquare = mod(toSquare) < 0.1

        if atSquare:

            # stop the motors and exit
            leftWheelMotor.setVelocity(  0 )
            rightWheelMotor.setVelocity( 0 )
            break

    else:

        # decide if we are about to hit a wall
        nearWall         = abs(x) > 0.8 or abs(z) > 0.8
        goingTowardsWall = normDot( pos, vel ) > 0.1
        shouldTurn       = nearWall and goingTowardsWall

        # find if we have a block on the left or right
        blockOnLeft  = frontLeftDistanceSensor.getValue()  < 700
        blockOnRight = frontRightDistanceSensor.getValue() < 700

        # set motors to turn if neccesary
        leftWheelMotor.setVelocity(  20 )
        rightWheelMotor.setVelocity( 20 - shouldTurn * 15 )

        # actuate claw to grab a block
        if (blockOnLeft or blockOnRight) and not shouldTurn:
            
            clawAngle   += 6
            gettingBlock = True

    # exponentially decay claw angle
    clawAngle = ( clawAngle + 0.1 ) * 0.7 - 0.1
    clawMotor.setPosition( clawAngle )

    # we have captured the block when the claw has come down mostly
    haveBlock = gettingBlock and clawAngle < 0
        
    