
# idp group L203
# Johns Automated Sorter for Meaningless Inconsequential Non-existent Experiments

import math
import numpy as np
from controller import Robot, Motor, DistanceSensor, GPS
import time


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
        self.pos              = np.zeros( 3 )
        self.vel              = np.zeros( 3 )
        self.posHist          = np.zeros( (10, 3) ) # stores last 10 positions of robot - maybe unneccesary
        self.distances        = np.zeros( (10, 2) ) # last 10 distances, 10 rows of [leftDistance, rightDistance]
        self.clawAngle        = 0.0
        self.gettingBlock     = False
        self.haveBlock        = False
        self.simTime          = self.timestep
        self.scheduleTuples   = []
        self.behaviour        = self.startSpin
        self.obj_pos          = np.zeros( 3 )
        self.boc_loc          = np.zeros( 3 )
        self.greenLevel       = 0.0
        self.redLevel         = 0.0
        self.boxFirstEdgeTime = 0

        # x, y and z coords for convenience
        self.x, self.y, self.z = self.pos

        # some functions to help set wheel speeds
        self.stop = lambda : self.setWheelSpeeds( 0.0, 0.0 )
        self.spin = lambda : self.setWheelSpeeds( 1.5, 3.0 )

        # function to get distance sensor readings
        self.getDistance = lambda : distanceFromReading( np.array([ self.leftDistanceSensor.getValue(),
                                                                    self.rightDistanceSensor.getValue() ]) )

        # initialise the sensors and enter the main loop
        self.initialiseSensors()
        self.mainLoop()


    def initialiseSensors(self):

        # get all the robot's sensors
        self.gps                 = GPS( "gps" )
        self.leftWheelMotor      = self.getDevice( "LeftWheelMotor"      )
        self.rightWheelMotor     = self.getDevice( "RightWheelMotor"     )
        self.clawMotor           = self.getDevice( "ClawMotor"           )
        self.rightDistanceSensor = self.getDevice( "DistanceSensorRIGHT" )
        self.leftDistanceSensor  = self.getDevice( "DistanceSensorLEFT"  )
        self.greenSensor         = self.getDevice( "lightSensorGREEN"    )
        self.redSensor           = self.getDevice( "lightSensorRED"      )

        # enable the sensors
        self.gps.enable( self.timestep )
        self.rightDistanceSensor.enable( self.timestep )
        self.rightWheelMotor.enableTorqueFeedback( self.timestep )
        self.leftDistanceSensor.enable( self.timestep )
        self.greenSensor.enable( self.timestep )
        self.redSensor.enable( self.timestep )

        # code for single distance sensor robot
        # self.distanceSensor = self.getDevice( "DistanceSensorFront" )
        # self.distanceSensor.enable( self.timestep )

        # set the motors' positions to infinity so we can use velocity control
        self.leftWheelMotor.setPosition(  float("inf") )
        self.rightWheelMotor.setPosition( float("inf") )
        
        self.clawMotor.setPosition( 0 )

        # start with motors at 0 velocity
        self.setWheelSpeeds( 0.0, 0.0 )


    def setWheelSpeeds(self, leftSpeed, rightSpeed):

        # set the two wheel speeds
        self.leftWheelMotor.setVelocity(  leftSpeed  )
        self.rightWheelMotor.setVelocity( rightSpeed )
        

    def setClawMotor(self, clawAngle):
        
        self.clawMotor.setPosition( clawAngle )


    def setBehaviour(self, newBehaviour):

        self.behaviour = newBehaviour


    def schedule(self, delay, func):

        # call func after delay millisceonds

        # add a tuple to the schedule containing the time at which
        # to call (in milliseconds) and the function to call
        self.scheduleTuples.append( ( self.simTime + delay, func ) )

        return ( self.simTime + delay, func )


    def updateSchedule(self):

        # loop over scheduleTuples and run the ones whose time has been reached
        for time, func in self.scheduleTuples:

            # skip if the tuple's time has not been reached
            if self.simTime < time: continue

            # run function and remove the tuple from the list
            func()
            self.scheduleTuples.remove( (time, func) )


    def updateColourSensors(self):
    
        self.greenLevel = self.greenSensor.getValue() 
        self.redLevel   = self.redSensor.getValue() 


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
        self.distances     = np.roll( self.distances, -1, axis=0 )
        self.distances[-1] = self.getDistance()


    def updateObjPos(self):
    
        sensor_dist  = 0.232
        self.obj_pos = norm( self.vel ) * (self.distances[-1, 1] + sensor_dist) + self.pos


    def checkForBox(self):
        
        # if we detect a step change in the distance sensor's measurement then assume a box was detected
        return self.distances[-2] != 0 and abs(self.obj_pos[0]) < 1.15 and abs(self.obj_pos[1]) < 1.15 and not -0.2 < self.distances[-1] - self.distances[-2] < 0.2


    # --- Behaviours ---
    # functions that can be assigned to the self.behaviour variable to be called once per frame
    # and control what the robot does at different parts of the process


    def startSpin(self):

        # set motors to spin
        self.setWheelSpeeds( 3.0, -3.0 )

        # start the spinning behaviour
        self.behaviour = self.spinAndFindBox


    def spinAndFindBox(self):

        # if the distances array is not yet initialised with data then return
        if self.distances[-2, 1] == 0:
            return

        # check if the right distance sensor detected an upwards step
        if self.distances[-2, 1] - self.distances[-1, 1] < -0.2:

            # record the time that this happened
            self.boxFirstEdgeTime = self.simTime

        # check if the left distance sensor detected a downwards step
        if self.distances[-2, 0] - self.distances[-1, 0] > 0.2:

            # start spinning in the opposite direction
            self.setWheelSpeeds( -0.5, 0.5 )

            # calculate how far back to rotate
            rotationTime = ( self.simTime - self.boxFirstEdgeTime ) / 2

            # after a time of rotationTime, call self.startBoxApproach
            self.schedule( rotationTime, self.startBoxApproach )



    def startBoxApproach(self):

        # open the claw ready to get the box
        self.setClawMotor( 1.8 )

        # start to move forward and start the goToBox behaviour after 1 second (to allow motor torque to settle)
        self.setWheelSpeeds(5.0, 5.0)
        self.schedule( 1000, lambda : self.setBehaviour( self.goToBox ) )


    def wander(self):

        # if we see a box in front of us, start the goToBox behaviour
        if self.checkForBox():
            self.box_loc   = self.obj_pos
            self.behaviour = self.goToBox

        # decide if we are about to hit a wall
        nearWall         = abs( self.x ) > 0.8 or abs( self.z ) > 0.8
        goingTowardsWall = np.dot( norm( self.pos ), norm( self.vel ) ) > 0.1
        shouldTurn       = nearWall and goingTowardsWall

        # set motors to turn to avoid the wall
        self.setWheelSpeeds( 8.0, 8.0 - shouldTurn * 7.0 )


    def goToBox(self):

        # if we touching the box then the motor torque has increased
        # switch to the self.checkBox behaviour when this happens

        if self.rightWheelMotor.getTorqueFeedback() > 0.12:
            
            self.behaviour = self.checkBox


    def checkBox(self):

        # stop the robot
        self.setWheelSpeeds( 0.0, 0.0 )
        self.setClawMotor(0)

        print(self.greenSensor.getValue())
        


    def mainLoop(self):

        # loop until simulation end
        while self.step( self.timestep ) != -1:

            self.simTime += self.timestep

            # update the time, distance sensor reading and scheduled tasks
            self.updateColourSensors()            
            self.updatePositionAndVelocity()
            self.updateDistance()
            self.updateObjPos()
            self.updateSchedule()


            # call the current behaviour function
            self.behaviour()

            # can print position and velocity
            # print( "pos: " + " ".join( ["%.2f" % v for v in self.pos] ) )
            # print( "vel: " + " ".join( ["%.2f" % v for v in self.vel] ) )


Jasmine()