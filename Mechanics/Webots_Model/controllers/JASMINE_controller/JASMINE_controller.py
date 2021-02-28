
# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import math
import numpy as np
from controller import Robot, Motor, DistanceSensor, GPS

# Jasmine class inherits from Robot class
# so we can use eg self.step instead of robot.step
class Jasmine(Robot):

    def __init__(self):

        # initialise the underlying Robot
        Robot.__init__(self)

        # get the world's timestep and initialise the sensors
        self.timestep = int( self.getBasicTimeStep() )
        self.initialiseSensors()

        # variables that help the robot work
        self.pos          = np.zeros( 3 )
        self.vel          = np.zeros( 3 )
        self.posHist      = np.zeros( (10, 3) )
        self.clawAngle    = -0.1
        self.gettingBlock = False
        self.haveBlock    = False

        # enter the main loop
        self.mainLoop()


    def initialiseSensors(self):

        # get all the robot's sensors
        self.gps             = GPS( "gps" )
        self.leftWheelMotor  = self.getDevice( "LeftWheelMotor"      )
        self.rightWheelMotor = self.getDevice( "RightWheelMotor"     )
        self.clawMotor       = self.getDevice( "ClawMotor"           )
        self.distanceSensor  = self.getDevice( "DistanceSensorFront" )

        # enable the sensors
        self.gps.enable( self.timestep )
        self.distanceSensor.enable( self.timestep )

        # set the motors' positions to infinity so we can use velocity control
        self.leftWheelMotor.setPosition(  float('inf') )
        self.rightWheelMotor.setPosition( float('inf') )

        # claw is using position control but needs to be a bit down so we can see the blocks
        self.clawMotor.setPosition( -0.1 )


    def updatePositionAndVelocity(self):

        # set the current position to the value of the gps (as a numpy array)
        self.pos = np.array( self.gps.getValues() )

        # roll the position history array up 1 spot and assign the current position to the last vector in the array
        self.posHist = np.roll( self.posHist, -1, axis=0 )
        self.posHist[-1] = self.pos

        # set the velocity using the last 2 positions in the position history array
        self.vel = ( self.posHist[-1] - self.posHist[-2] ) / ( self.timestep / 1000 )


    def mainLoop(self):

        # loop until simulation end
        while self.step( self.timestep ) != -1:

            self.updatePositionAndVelocity()

            # can print gps position or velocity
            print( "pos: " + " ".join( ["%.2f" % v for v in self.pos] ) )
            print( "vel: " + " ".join( ["%.2f" % v for v in self.vel] ) )

Jasmine()