
# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import math, threading
import numpy as np
from controller import Robot, Motor, DistanceSensor, GPS

# norm(v) returns v scaled to unit length
norm = lambda v : v / np.linalg.norm(v)

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
        self.posHist      = np.zeros( (10, 3) ) # stores last 10 positions of robot - maybe unneccesary
        self.clawAngle    = -0.1
        self.gettingBlock = False
        self.haveBlock    = False

        # x, y and z coords for convenience
        self.x, self.y, self.z = self.pos

        # some functions to help set wheel speeds
        self.stop = lambda : self.setWheelSpeeds(  0.0, 0.0 )
        self.spin = lambda : self.setWheelSpeeds( -5.0, 5.0 )

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

        # update x, y and z coords
        self.x, self.y, self.z = self.pos

        # roll the position history array up 1 spot and assign the current position to the last vector in the array
        self.posHist = np.roll( self.posHist, -1, axis=0 )
        self.posHist[-1] = self.pos

        # set the velocity using the last 2 positions in the position history array
        # could use last 3 positions to get O(x^2) error
        self.vel = ( self.posHist[-1] - self.posHist[-2] ) / ( self.timestep / 1000 )


    def setWheelSpeeds(self, leftSpeed, rightSpeed):

        # set the two wheel speeds
        self.leftWheelMotor.setVelocity(  leftSpeed  )
        self.rightWheelMotor.setVelocity( rightSpeed )


    def wander(self):

        # just drive around avoiding walls

        # decide if we are about to hit a wall
        nearWall         = abs( self.x ) > 0.8 or abs( self.z ) > 0.8
        goingTowardsWall = np.dot( norm( self.pos ), norm( self.vel ) ) > 0.1
        shouldTurn       = nearWall and goingTowardsWall

        # set motors to turn if neccesary
        self.leftWheelMotor.setVelocity(  20.0 )
        self.rightWheelMotor.setVelocity( 18.0 - shouldTurn * 15 )


    def mainLoop(self):

        # start a spin and stop after 2.7 seconds - equal to 1 revolution
        self.spin()
        threading.Timer( 2.7, self.stop ).start()

        # loop until simulation end
        while self.step( self.timestep ) != -1:

            self.updatePositionAndVelocity()

            # can print position and velocity
            # print( "pos: " + " ".join( ["%.2f" % v for v in self.pos] ) )
            # print( "vel: " + " ".join( ["%.2f" % v for v in self.vel] ) )

Jasmine()