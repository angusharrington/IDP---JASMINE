# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import math
from controller import Robot, Motor, DistanceSensor

robot = Robot()

# get the time step of the current world.
timestep = int( robot.getBasicTimeStep() )

# get the motors and distance sensors
leftWheelMotor           = robot.getDevice( "LeftWheelMotor"           )
rightWheelMotor          = robot.getDevice( "RightWheelMotor"          )
frontLeftDistanceSensor  = robot.getDevice( "DistanceSensorFrontLeft"  )
frontRightDistanceSensor = robot.getDevice( "DistanceSensorFrontRight" )

# enable the distance sensors
frontLeftDistanceSensor.enable(  100 )
frontRightDistanceSensor.enable( 100 )

# set the motors positions so that we can set the velocity
leftWheelMotor.setPosition(  100000 )
rightWheelMotor.setPosition( 100000 )

# Main loop
while robot.step(timestep) != -1:

    leftDist = frontLeftDistanceSensor.getValue()

    # sometimes we get NaN so ignore
    if math.isnan(leftDist):
        continue

    if leftDist >= 999:

        leftWheelMotor.setVelocity(  5 )
        rightWheelMotor.setVelocity( 5 )

    else:

        print("wall!")

        rightWheelMotor.setVelocity( 0 )
