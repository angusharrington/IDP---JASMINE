# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import math
from controller import Robot, Motor, DistanceSensor, LightSensor

robot = Robot()

# get the time step of the current world.
timestep = int( robot.getBasicTimeStep() )

## robot_colour = 1 # 1 meaning red, 0 meaning green. As with blocks

# get the motors and distance sensors
leftWheelMotor           = robot.getDevice( "LeftWheelMotor"           )
rightWheelMotor          = robot.getDevice( "RightWheelMotor"          )
frontLeftDistanceSensor  = robot.getDevice( "DistanceSensorFrontLeft"  )
frontRightDistanceSensor = robot.getDevice( "DistanceSensorFrontRight" )
lightSensor              = robot.getDevice( "LightSensor"              )


# enable the distance sensors
frontLeftDistanceSensor.enable(  100 )
frontRightDistanceSensor.enable( 100 )

# set the motors positions so that we can set the velocity
leftWheelMotor.setPosition(  100000 )
rightWheelMotor.setPosition( 100000 )

#def block_near (left_distance, right_distance):

# Main loop
while robot.step(timestep) != -1:

    leftDist = frontLeftDistanceSensor.getValue()
    rightDist = frontRightDistanceSensor.getValue()
    
    # NB block_colour = -1 means not assigned. = 1 means block is red, = 0 means block is green
##    block_colour = -1
    

    # sometimes we get NaN so ignore
    if math.isnan(leftDist):
        continue
    if math.isnan(rightDist):
        continue
        
    if leftDist >= 999:

        leftWheelMotor.setVelocity(  5 )
        rightWheelMotor.setVelocity( 5 )
 
       
##    if block_near(leftDist, rightDist) == 1:
        
##        irradiance = lightSensor.getValue()
        
##        if irradiance > #some_threshold_tba:
            
##            block_colour = 1
        
##        if irradiance <= #some_threshold_tba:
            
##            block_colour = 0
            
##        if block_colour == robot_colour:
            
##            # pick up process begins
        
##        elif block_colour >= 0:
            
##            # Sends coordinates to other robot
            


    else:

        print("wall!")

        rightWheelMotor.setVelocity( 0 )
        