# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import math
from controller import Robot, Motor, DistanceSensor, GPS

# dot product
dot = lambda v1, v2 : v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2]

# magnitude of vector
mod = lambda v : ( v[0]**2 + v[1]**2 + v[2]**2 ) ** 0.5

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

clawAngle = -0.1
prevPos   = gps.getValues()

# Main loop
while robot.step(timestep) != -1:

    pos     = gps.getValues()
    x, y, z = pos

    vel = [ (a-b)/dt for a,b in zip(pos, prevPos) ]
    prevPos = pos

    #print( " ".join( ["%.2f" % v for v in pos] ) )
    print( " ".join( ["%.2f" % v for v in vel] ) )

    nearWall         = abs(x) > 0.8 or abs(z) > 0.8
    goingTowardsWall = dot( pos, vel ) / mod(vel) > -0.2
    shouldTurn       = nearWall and goingTowardsWall

    blockOnLeft  = frontLeftDistanceSensor.getValue()  < 700
    blockOnRight = frontRightDistanceSensor.getValue() < 700

    leftWheelMotor.setVelocity(  20 )
    rightWheelMotor.setVelocity( 20 - shouldTurn * 15 )

    if (blockOnLeft or blockOnRight) and not shouldTurn:
        clawAngle += 6

    clawAngle = ( clawAngle + 0.1 ) * 0.7 - 0.1

    clawMotor.setPosition( clawAngle )
