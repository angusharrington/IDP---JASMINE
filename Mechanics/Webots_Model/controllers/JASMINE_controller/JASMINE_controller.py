
# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import math
import numpy as np
import matplotlib.pyplot as plt
from controller import Robot, Motor, DistanceSensor, GPS


# norm(v) returns v scaled to unit length
norm = lambda v : v / np.linalg.norm(v)

# calculate actual distance from the sensor reading
distanceFromReading = lambda x : (0.32 / x) ** (5/6) - 0.1


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
        self.pos            = np.zeros( 3 )
        self.vel            = np.zeros( 3 )
        self.posHist        = np.zeros( (10, 3) ) # stores last 10 positions of robot - maybe unneccesary
        self.distances      = [0, 0, 0]
        self.clawAngle      = 0.0
        self.gettingBlock   = False
        self.haveBlock      = False
        self.simTime        = self.timestep
        self.scheduleTuples = []

        # x, y and z coords for convenience
        self.x, self.y, self.z = self.pos

        # some functions to help set wheel speeds
        self.stop = lambda : self.setWheelSpeeds( 0.0, 0.0 )
        self.spin = lambda : self.setWheelSpeeds( 1.5, 3.0 )

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
        self.leftWheelMotor.setPosition(  float("inf") )
        self.rightWheelMotor.setPosition( float("inf") )

        # start with motors at 0 velocity
        self.setWheelSpeeds( 0.0, 0.0 )


    def schedule(self, delay, func):

        # call func after delay millisceonds

        # add a tuple to the schedule containing the time at which
        # to call (in milliseconds) and the function to call
        self.scheduleTuples.append( ( self.simTime + delay, func ) )


    def runSchedule(self):

        # loop over scheduleTuples and run the ones whose time has been reached
        for time, func in self.scheduleTuples:

            # skip if the tuple's time has not been reached
            if self.simTime < time: continue

            # run function and remove the tuple from the list
            func()
            self.scheduleTuples.remove( (time, func) )


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


    def updateDistance(self):

        # add the distances sensor reading to the distances list
        self.distances.append( distanceFromReading( self.distanceSensor.getValue() ) )

        # if we detect a step change in distance measured by the sensor then a block is in front
        if self.distances[-2] - self.distances[-1] > 0.1:

            # calculae and print the block position
            blockPos = self.box_detection( self.distances[-1] )
            print( f"{self.simTime}: found block at {blockPos}" )


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
        self.leftWheelMotor.setVelocity(  15.0 )
        self.rightWheelMotor.setVelocity( 14.0 - shouldTurn * 10 )


    def box_detection(self, distanceSensed):

        sensor_dist  = 0.232
        sensorRange  = 0.8
        object_position = norm( self.vel ) * (distanceSensed + sensor_dist) + self.pos
        
        if object_position[0] < 1.16 and object_position[1] < 1.16 and distanceSensed < sensorRange:
            return object_position
        else:
            return False


    def mainLoop(self):

        # start a spin and stop after 2.55 seconds - equal to 1 revolution
        # self.spin()
        # self.schedule( 5000, self.stop )
        # self.schedule( 6000, self.plotDists )

        # loop until simulation end
        while self.step( self.timestep ) != -1:

            self.simTime += self.timestep

            self.updatePositionAndVelocity()
            self.updateDistance()
            self.runSchedule()
            self.wander()

            # can print position and velocity
            # print( "pos: " + " ".join( ["%.2f" % v for v in self.pos] ) )
            # print( "vel: " + " ".join( ["%.2f" % v for v in self.vel] ) )


    def plotDists(self):

        plt.plot( self.distances )
        plt.show()


Jasmine()