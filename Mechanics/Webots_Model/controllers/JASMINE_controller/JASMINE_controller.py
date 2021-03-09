import math
import numpy as np
from controller import Robot, Motor, DistanceSensor, GPS
from enum import Enum

# norm(v) returns v scaled to unit length
norm = lambda v : v / np.linalg.norm(v)

# calculate actual distance from the sensor reading
distanceFromReading = lambda x : (0.32 / x) ** (5/6) - 0.1

# colours enum
class Colour(Enum):
    RED   = 0
    GREEN = 1

# locations to move the boxes to
greenCentre = np.array( [0, 0, -0.4] )
redCentre   = np.array( [0, 0,  0.4] )


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
        self.angle            = 0 # angle to x axis
        self.forward          = np.zeros( 3 ) # unit vector in the direction we are facing
        self.posHist          = np.zeros( (10, 3) ) # stores last 10 positions of robot - maybe unneccesary
        self.distances        = np.zeros( (10, 2) ) # last 10 distances, 10 rows of [leftDistance, rightDistance]
        self.clawAngle        = 0.0
        self.simTime          = self.timestep
        self.scheduleTuples   = []
        self.behaviour        = lambda : self.turnToDirection( np.array([0, 0, -1]) )
        self.greenLevel       = 0.0
        self.redLevel         = 0.0
        self.boxFirstEdgeTime = 0
        self.prev2vel         = np.zeros( 2 )

        # Robot dependant variables
        self.colour = Colour.RED
        self.home   = redCentre

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
        self.gps                 = self.getDevice( "gps"                 )
        self.gpsOffset           = self.getDevice( "gpsOFFSET"           )
        self.leftWheelMotor      = self.getDevice( "LeftWheelMotor"      )
        self.rightWheelMotor     = self.getDevice( "RightWheelMotor"     )
        self.clawMotor           = self.getDevice( "ClawMotor"           )
        self.rightDistanceSensor = self.getDevice( "DistanceSensorRIGHT" )
        self.leftDistanceSensor  = self.getDevice( "DistanceSensorLEFT"  )
        self.greenSensor         = self.getDevice( "lightSensorGREEN"    )
        self.redSensor           = self.getDevice( "lightSensorRED"      )
        self.emitter            = self.getDevice( "Emitter"             )
        self.receiver            = self.getDevice( "Receiver"            )

        # enable the sensors
        self.gps.enable( self.timestep )
        self.gpsOffset.enable( self.timestep )
        self.rightDistanceSensor.enable( self.timestep )
        self.rightWheelMotor.enableTorqueFeedback( self.timestep )
        self.leftDistanceSensor.enable( self.timestep )
        self.greenSensor.enable( self.timestep )
        self.redSensor.enable( self.timestep )
        self.receiver.enable( self.timestep )


        # code for single distance sensor robot
        # self.distanceSensor = self.getDevice( "DistanceSensorFront" )
        # self.distanceSensor.enable( self.timestep )

        # set the motors' positions to infinity so we can use velocity control
        self.leftWheelMotor.setPosition(  float("inf") )
        self.rightWheelMotor.setPosition( float("inf") )
        
        # start the claw closed
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
        self.prev2vel[-2] = self.prev2vel[-1]        
        self.prev2vel[-1] = np.linalg.norm(self.vel)

        # from the difference in gps readings, calculate the angle we are facing and update the forward vector
        gpsDifference = np.array( [self.gpsOffset.getValues()[0], self.gpsOffset.getValues()[2]]) - np.array([self.pos[0], self.pos[2]])

        self.angle   = np.arctan2( gpsDifference[2], gpsDifference[0] )
        self.forward = norm( gpsDifference )


    def updateDistance(self):

        # add the distance sensors reading to the distances list
        self.distances     = np.roll( self.distances, -1, axis=0 )
        self.distances[-1] = self.getDistance()


    # --- Behaviours ---
    # functions that can be assigned to the self.behaviour variable to be called once per frame
    # and control what the robot does at different parts of the process


    def turnToDirection(self, direction):

        # get a value representing the amount we still need to turn
        turnAmount = np.cross( norm(direction), norm( direction + self.forward ) ) @ np.array( [0,1,0] ) ** 0.5 * 20

        # set the wheel speeds based on this value
        self.setWheelSpeeds( turnAmount, -turnAmount )


    def startSpin(self):

        # set motors to spin
        self.setWheelSpeeds( 1.0, -1.0 )
        print(self.forward)

        # start the spinning behaviour
        self.behaviour = self.spinAndFindBox

    def locationsRoute(self):
    
        # for red robot
        
        PointsOrder = np.array([[0, 0], [0.8, 0], [0.8, 0.8], [0, 0.8], [-0.8, 0.8]])

        # for green robot
        # PointsOrder = np.array([[0.8, -0.8], [0,-0.8], [-0.8,-0.8], [-0.8,0]])
        
        for i in PointsOrder:
        
            self.setBehaviour(self.goToPoint(i))
            
    def goToPoint(self, point):
    
        direction = point - self.pos
        turnToDirection(direction)

        self.setWheelSpeeds(5.0, 5.0)
        
        if np.linalg.norm( point - self.pos ) < 0.1:

            self.behaviour = self.stop

        



    def spinAndFindBox(self):

        # if the distances array is not yet initialised with data then return
        if self.distances[-2, 1] == 0:
            return

        # check if the right distance sensor detected a downwards step
        if self.distances[-1, 1] - self.distances[-2,1] < -0.15:


            # record the time that this happened
            self.boxFirstEdgeTime = self.simTime
            
        # check if the left distance sensor detected an upwards step
        if self.distances[-1, 0] - self.distances[-2, 0] > 0.15:


            # start spinning in the opposite direction
            self.setWheelSpeeds( -1.0, 1.0 )
            
            # calculate how far back to rotate
            rotationTime = ( self.simTime - self.boxFirstEdgeTime ) / 2

            # after a time of rotationTime, call self.startBoxApproach
            self.schedule( rotationTime, self.startBoxApproach )


    def startBoxApproach(self):

        # stop the previous behaviour
        self.behaviour = lambda : None

        # open the claw ready to get the box
        self.clawMotor.setPosition( 1.8 )

        # start to move forward and start the goToBox behaviour after the robot speed has settled
        self.setWheelSpeeds(5.0, 5.0)
        self.schedule( 500, lambda : self.setBehaviour( self.goToBox ) )


    def goToBox(self):

        # when we detect light on one of the light sensors we have readched the box

        if self.greenLevel > 0.99 or self.redLevel > 0.99:

            self.schedule( 1000, lambda : self.setBehaviour( self.checkBox) )


    def checkBox(self):

        # stop the robot and close the claw
        self.setWheelSpeeds( 0.0, 0.0 )
        self.setClawMotor(0)

        colour = None

        # set colour to whichever colour was detected
        if self.greenSensor.getValue() > 0.99:

            print( "green box detected" )
            colour = Colour.GREEN
            
        if self.redSensor.getValue() > 0.99:
            
            print( "red box detected" )
            colour = Colour.RED
            
        # if we picked up the right colour then bring it back, otherwise continue the search
        if colour == self.colour:

            self.schedule( 1000, lambda : self.setBehaviour( self.carryBoxHome ) )

        else:

            self.behaviour = self.continueSearching
            

    def carryBoxHome(self):
        
        # get the direction of home
        homeDirection = norm( self.home - self.pos )
            
        # turnAmount is based on dot product of vel direction with home direction
        # it is 2 when the vectors are 180 degrees apart and 0 when they are the same
        turnAmount = 1 - np.dot( norm(self.vel), homeDirection )

        # drive home
        self.setWheelSpeeds(5.0 - turnAmount * 7.0, 5.0 + turnAmount * 7.0)

        # if we are home then stop
        if np.linalg.norm( self.home - self.pos ) < 0.1:

            self.behaviour = self.stop

         
    def continueSearching(self):

        # Note 0.1 chosen arbitrarily, needs to be made more accurate
        self.emitter.send(self.pos+self.forward*0.1)
        

        
        print("continueSearching not yet implemented")


    def mainLoop(self):

        # loop until simulation end
        while self.step( self.timestep ) != -1:

            self.simTime += self.timestep

            # update the time, distance sensor reading and scheduled tasks
            self.updateColourSensors()            
            self.updatePositionAndVelocity()
            self.updateDistance()
            self.updateSchedule()

            # call the current behaviour function
            self.behaviour()

            # can print position and velocity
            # print( "pos: " + " ".join( ["%.2f" % v for v in self.pos] ) )
            # print( "vel: " + " ".join( ["%.2f" % v for v in self.vel] ) )


Jasmine()