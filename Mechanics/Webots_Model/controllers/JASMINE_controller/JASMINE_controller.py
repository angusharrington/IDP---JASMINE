
# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import math
import numpy as np
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

        # variables that help the robot work
        self.pos            = np.zeros( 3 )
        self.vel            = np.zeros( 3 )
        self.posHist        = np.zeros( (10, 3) ) # stores last 10 positions of robot - maybe unneccesary
        self.distances      = np.zeros( 10 ) # last 10 distances
        self.clawAngle      = 0.0
        self.gettingBlock   = False
        self.haveBlock      = False
        self.simTime        = self.timestep
        self.scheduleTuples = []
        self.behaviour      = self.wander

        # x, y and z coords for convenience
        self.x, self.y, self.z = self.pos

        # some functions to help set wheel speeds
        self.stop = lambda : self.setWheelSpeeds( 0.0, 0.0 )
        self.spin = lambda : self.setWheelSpeeds( 1.5, 3.0 )

        # function to get distance sensor reading
        self.getDistance = lambda : distanceFromReading( self.distanceSensor.getValue() )

        # initialise the sensors and enter the main loop
        self.initialiseSensors()
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


    def setWheelSpeeds(self, leftSpeed, rightSpeed):

        # set the two wheel speeds
        self.leftWheelMotor.setVelocity(  leftSpeed  )
        self.rightWheelMotor.setVelocity( rightSpeed )



    def schedule(self, delay, func):

        # call func after delay millisceonds

        # add a tuple to the schedule containing the time at which
        # to call (in milliseconds) and the function to call
        self.scheduleTuples.append( ( self.simTime + delay, func ) )

        return ( self.simTime + delay, func )


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
        self.vel = ( self.posHist[-1] - self.posHist[-2] ) / ( self.timestep / 1000 )


    def updateDistance(self):

        # add the distance sensors reading to the distances list
        self.distances     = np.roll( self.distances, -1 )
        self.distances[-1] = self.getDistance()


    def box_detection(self):

        sensor_dist  = 0.232
        sensorRange  = 0.8
        object_position = norm( self.vel ) * (self.distances[-1] + sensor_dist) + self.pos
        
        if abs(object_position[0]) < 1.15 and abs(object_position[2]) < 1.15 and self.distances[-1] < sensorRange:
            print(object_position)
            return object_position
        else:
            return False


    def checkForBox(self):

        print( self.distances[-1])

        # if we detect a step change in the distance sensor's measurement then assume a box was detected
        return self.distances[-2] != 0 and not -0.2 < self.distances[-1] - self.distances[-2] < 0.2


    # --- Behaviours ---
    # functions that can be assigned to the self.behaviour variable to be called once per frame
    # and control what the robot does at different parts of the process


    def wander(self):

        # if we see a box in front of us, start the goToBox behaviour
        if self.checkForBox():
            self.behaviour = self.goToBox

        # decide if we are about to hit a wall
        nearWall         = abs( self.x ) > 0.8 or abs( self.z ) > 0.8
        goingTowardsWall = np.dot( norm( self.pos ), norm( self.vel ) ) > 0.1
        shouldTurn       = nearWall and goingTowardsWall

        # set motors to turn to avoid the wall
        self.setWheelSpeeds( 15.0, 14.0 - shouldTurn * 10 )


    def goToBox(self):

        # if we are close to the box then stop
        if self.distances[-1] < 0.15:
            self.behaviour = self.stop

        # just go forward, towards the box
        self.setWheelSpeeds( 7.0, 7.0 )



    def stop(self):

        # stop the robot
        self.setWheelSpeeds( 0.0, 0.0 )



    def mainLoop(self):

        # loop until simulation end
        while self.step( self.timestep ) != -1:

            self.simTime += self.timestep

            ## update the time, distance sensor reading and scheduled tasks
            self.updatePositionAndVelocity()
            self.updateDistance()
            self.runSchedule()

            # call the current behaviour function
            self.behaviour()

            # can print position and velocity
            # print( "pos: " + " ".join( ["%.2f" % v for v in self.pos] ) )
            # print( "vel: " + " ".join( ["%.2f" % v for v in self.vel] ) )


Jasmine()