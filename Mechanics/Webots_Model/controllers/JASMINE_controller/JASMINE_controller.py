import math
import numpy as np
from controller import Robot, Motor, DistanceSensor, GPS
from enum import Enum
import struct
import sys

# mag(v) returns the magnitude of v
mag = lambda v : np.linalg.norm(v)

# norm(v) returns v scaled to unit length
norm = lambda v : v / mag(v)

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
        self.angvel           = 0
        self.forward          = np.zeros( 3 ) # unit vector in the direction we are facing
        self.posHist          = np.zeros( (10, 3) ) # stores last 10 positions of robot - maybe unneccesary
        self.distances        = np.zeros( (10, 2) ) # last 10 distances, 10 rows of [leftDistance, rightDistance]
        self.clawAngle        = 0.0
        self.simTime          = self.timestep
        self.scheduleTuples   = []
        self.behaviour        = self.locationsRoute
        self.greenLevel       = 0.0
        self.redLevel         = 0.0
        self.boxFirstEdgeTime = None
        self.receivedData     = np.array([])
        self.otherRobot       = np.array([[0, 0], [0, 0], [0, 0]])
        self.pointsSearched   = 0 # number of points in the grid that we have spun around completely and cleared
        self.directionCleared = np.array( [1,0,0] ) # direction that we have cleared up to on the current point - starting from 1, 0, 0
        self.ourColourBoxes   = []
        self.otherColourBoxes = []
 

        # Robot dependant variables
        if sys.argv[1] == "red":

            self.colour = Colour.RED
            self.home   = redCentre

        elif sys.argv[1] == "green":

            self.colour = Colour.GREEN
            self.home   = greenCentre

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
        self.emitter             = self.getDevice( "Emitter"             )
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

        # from the difference in gps readings, calculate the new angle we are facing and update the forward vector
        gpsDifference = self.gpsOffset.getValues() - self.pos
        newAngle      = np.arctan2( gpsDifference[2], gpsDifference[0] )
        self.forward  = norm( gpsDifference )

        # also calculate the angular velocity from the difference in angles and set self.angle
        self.angvel   = ( newAngle - self.angle ) / ( self.timestep / 1000 )
        self.angle    = newAngle


    def updateRecieverEmitter(self):

        # send other robot position and velocity for collision avoidance (ignoring y co-ordinate)
        posVelAndFor = struct.pack('ffffff', self.pos[0], self.pos[2], self.vel[0], self.vel[2], self.forward[0], self.forward[2])
        self.emitter.send(posVelAndFor)

        # update received data
        if self.receiver.getQueueLength() > 0:
            
            data = self.receiver.getData()

            if sys.getsizeof(data) == 57:

                recPosAndVel = np.asarray(struct.unpack("ffffff", data))
                self.otherRobot = np.array([[recPosAndVel[0], recPosAndVel[1]], [recPosAndVel[2], recPosAndVel[3]], [recPosAndVel[4], recPosAndVel[5]]])

            elif sys.getsizeof(data) == 45:

                boxLoc = np.asarray(struct.unpack("fff", data))
                self.ourColourBoxes.append( boxLoc )


    # function that tells you if a point lies in a region
    def intersect(self, point, poly):

        x = point[0]
        y = point[1]

        n = 4
        inside = False
        p2x = 0.0
        p2y = 0.0
        xints = 0.0
        p1x,p1y = poly[0]
        for i in range(n+1):
            p2x,p2y = poly[i % n]
            if y > min(p1y,p2y):
                if y <= max(p1y,p2y):
                    if x <= max(p1x,p2x):
                        if p1y != p2y:
                            xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x,p1y = p2x,p2y

        return inside

    # This function should be called by 1 robot only every frame
    def collisionAvoid(self):

        # produce coordinates for areas both robots will cover over the next 0.5 seconds, if these overlap, stop 1 robot

        pos2 = self.pos + self.vel*0.5
        pos22 = self.otherRobot[0] + self.otherRobot[1]*0.5

        # 90degree CW rotation
        rot = np.array([[0, -1], [1, 0]])

        front1 = norm(np.array([self.forward[0], self.forward[2]]))
        front2 = norm(self.otherRobot[2])

        gpsToFront = 0.22
        gpsToSide  = 0.075
        gpsToBack  = 0.1

        # coords of this robot
        p1 = [[self.pos + gpsToSide*(rot.dot(front1)) - front1*gpsToBack], [self.pos - gpsToSide*(rot.dot(front1)) - front1*gpsToBack], [pos2 + gpsToSide*(rot.dot(front1)) + front1*gpsToFront], [pos2 - gpsToSide*(rot.dot(front1)) + front1*gpsToFront]]
        # coords of other robot
        p2 = [[self.otherRobot[0] + gpsToSide*(rot.dot(front2)) - front2*gpsToBack], [self.otherRobot[0] - gpsToSide*(rot.dot(front2)) - front2*gpsToBack], [pos22 + gpsToSide*(rot.dot(front2)) + front2*gpsToFront], [pos22 - gpsToSide*(rot.dot(front2)) + front2*gpsToFront]]

        for i in p1:
            if self.intersect(i, p2) is True:
                self.setWheelSpeeds(0,0)
        
        for j in p2:
            if self.intersect(j, p1) is True:
                self.setWheelSpeeds(0, 0)


    def updateDistance(self):

        # add the distance sensors reading to the distances list
        self.distances     = np.roll( self.distances, -1, axis=0 )
        self.distances[-1] = self.getDistance()


    # --- Behaviours ---
    # functions that can be assigned to the self.behaviour variable to be called once per frame
    # and control what the robot does at different parts of the process


    def turnToDirection(self, direction, nextBehaviour = lambda : None):

        # get a value representing the amount we still need to turn
        turnAmount = np.clip( np.cross( norm(direction), norm( direction + self.forward ) ) @ np.array( [0,1,0] ) * 50, -7, 7 )

        # set the wheel speeds based on this value
        self.setWheelSpeeds( turnAmount, -turnAmount )

        # when we have turned, start the next behaviour
        if abs( turnAmount ) < 0.01:
            self.behaviour = nextBehaviour


    def goToPoint(self, destination, nextBehaviour = lambda : None, directionOnceArrived = None, tolerance = 0.02):
    
        # get the direction to the destination, making sure it's in the horizontal plane
        direction    = destination - self.pos
        direction[1] = 0

        turnAmount = np.clip( np.cross( norm(direction), norm( direction + self.forward ) ) @ np.array( [0,1,0] ) * 50, -7, 7 )

        # get a baseSpeed value - slow if we're close to the destination but not facing it and otherwise fast
        baseSpeed = 12.0 - min( abs(turnAmount) * 15 * (mag(direction) < 0.1),  12 )

        greenSquare = [[0.2, -0.6], [0.2, -0.2], [-0.2, -0.2], [-0.2, -0.6]]
        redSquare   = [[0.2,  0.6], [0.2,  0.2], [-0.2,  0.2], [-0.2,  0.6]]

        xzposNose = 0.3*self.forward + self.pos
        xzposNose = np.array([xzposNose[0], xzposNose[2]])        

        if False and self.intersect(xzposNose, greenSquare) is False and self.intersect(xzposNose, redSquare) is False:

            # get a value that is large when we are heading into the red square
            toRedSquare    = redCentre - self.pos
            avoidRedSquare = np.clip( ( 1.4 - mag( toRedSquare ) ) * ( norm(toRedSquare) @ self.forward ), 0, 0.2 ) * 10 * ( mag(destination - redCentre) > 0.1 )

            # get a value that is large when we are heading into the green square
            toGreenSquare    = greenCentre - self.pos
            avoidGreenSquare = np.clip( ( 1.4 - mag( toGreenSquare ) ) * ( norm(toGreenSquare) @ self.forward ), 0, 0.2 ) * 10 * ( mag(destination - greenCentre) > 0.1 )

            print( sys.argv[1], baseSpeed + turnAmount + avoidRedSquare + avoidGreenSquare, baseSpeed - turnAmount - avoidRedSquare - avoidGreenSquare )

            # set the wheel speeds based on these values
            self.setWheelSpeeds( baseSpeed + turnAmount + avoidRedSquare + avoidGreenSquare, baseSpeed - turnAmount - avoidRedSquare - avoidGreenSquare )
        
        else:
            self.setWheelSpeeds( baseSpeed + turnAmount, baseSpeed - turnAmount )


        # if we're within the tolerance distance to the destination then we have arrived
        arrived = np.linalg.norm( direction ) < tolerance

        # if we want to face a certain direction once we arrive then start the turnToDirection behaviour
        if arrived and directionOnceArrived is not None:
            self.behaviour = lambda : self.turnToDirection( directionOnceArrived, nextBehaviour )

        # when we have arrived, start the next behaviour
        elif arrived:
            self.behaviour = nextBehaviour


    def locationsRoute(self):

        # close the claw in case it's open
        self.clawMotor.setPosition( 0 )

        # all the positions to spin around at
        spinPositions = np.array( [[0, 0, -0.79], [ 0.8, 0, -0.8], [ 0.79, 0, 0], [ 0.8, 0,  0.8],
                                   [0, 0,  0.79], [-0.8, 0,  0.8], [-0.79, 0, 0], [-0.8, 0, -0.8]] )

        # if its the green robot use the same array rotated 4 spaces along
        if self.colour == Colour.GREEN:
            spinPositions = np.roll( spinPositions, 4, axis=0 )

        # figure out where we have to go now
        pointToGoTo        = spinPositions[self.pointsSearched]
        directionToStartAt = self.directionCleared


        # start going to the next point
        self.behaviour = lambda : self.goToPoint( pointToGoTo, self.startSpin, directionToStartAt )


    def startSpin(self):

        # set motors to spin
        self.setWheelSpeeds( 2.0, -2.0 )

        # start the spinning behaviour
        self.behaviour = self.spinAndFindBox


    def spinAndFindBox(self):

        # if in the next timestep we will cross 0 angle then move onto the next point to search
        nextAngle = self.angle + self.angvel * ( self.timestep / 1000 )

        if self.angle < 0 and nextAngle > 0:

            # increment the number of points we've searched, set the direction cleared back to its initial value and call locationsRoute
            self.directionCleared = np.array( [1,0,0] )
            self.pointsSearched += 1
            self.locationsRoute()


        # if the distances array is not yet initialised with data then return
        if self.distances[-2, 1] == 0:
            return

        # check if the right distance sensor detected a downwards step and we should ge the box
        if self.distances[-1, 1] - self.distances[-2,1] < -0.05 and self.shouldGetBox():

            # record the time that this happened
            self.boxFirstEdgeTime = self.simTime

            
        # check if the left distance sensor detected an upwards step and we already detected a step from the other sensor
        if self.distances[-1, 0] - self.distances[-2, 0] > 0.05 and self.boxFirstEdgeTime is not None:

            # we have now cleared the angle up to this direction
            self.directionCleared = self.forward

            # start spinning in the opposite direction
            self.setWheelSpeeds( -2.0, 2.0 )
            
            # calculate how far back to rotate
            rotationTime = ( self.simTime - self.boxFirstEdgeTime ) / 2

            # clear the boxFirstEdgeTime as we are done with it
            self.boxFirstEdgeTime = None

            # open the claw ready to get the box
            self.clawMotor.setPosition( 0.5 )

            # after a time of rotationTime, call self.startBoxApproach
            self.schedule( rotationTime, self.startBoxApproach )


    def shouldGetBox(self):

        # 90 degree CW matrix   
        rot = np.array([[0, -1], [1, 0]])
        straight = norm(np.array([self.forward[0], self.forward[2]]))
        side = rot.dot(straight)
        forwardDisp = straight*(self.distances[-1, 1]+0.25)
        objLoc = self.pos + np.array([forwardDisp[0], 0, forwardDisp[1]]) + np.array([side[0], 0, side[1]])*0.07

        # regions
        greenSquare = [[0.2, -0.6], [0.2, -0.2], [-0.2, -0.2], [-0.2, -0.6]]
        redSquare = [[0.2, 0.6], [0.2, 0.2], [-0.2, 0.2], [-0.2, 0.6]]
        arena = [[1.15, 1.15], [1.15, -1.15], [-1.15, -1.15], [-1.15, 1.15]]

        # other robots 4 corners
        centre = self.otherRobot[0]
        ahead = self.otherRobot[2]

        gpsToFront = 0.3
        gpsToSide  = 0.085
        gpsToBack  = 0.15

        p1 = np.ndarray.tolist(centre + gpsToSide*(rot.dot(ahead)) - ahead*gpsToBack)
        p2 = np.ndarray.tolist(centre - gpsToSide*(rot.dot(ahead)) - ahead*gpsToBack)
        p3 = np.ndarray.tolist(centre + gpsToSide*(rot.dot(ahead)) + ahead*gpsToFront)
        p4 = np.ndarray.tolist(centre - gpsToSide*(rot.dot(ahead)) + ahead*gpsToFront)

        fourCorners = [p1, p2, p3, p4] 
        # get the distances to all the other colour boxes
        closeToOtherColourBoxes = [ mag(objLoc - box) < 0.05 for box in self.otherColourBoxes ]
        # if the block is too close to any we've already seen then ignore it unless the simulation has been going for a long time
        # if True in closeToOtherColourBoxes and self.simTime < 12000:
        #     return False

        return self.intersect(np.array([objLoc[0], objLoc[2]]), fourCorners) == False and self.intersect(np.array([objLoc[0], objLoc[2]]), greenSquare) == False and self.intersect(np.array([objLoc[0], objLoc[2]]), redSquare) == False and self.intersect(np.array([objLoc[0], objLoc[2]]), arena) == True
            

    def startBoxApproach(self):

        # stop the previous behaviour
        self.behaviour = lambda : None

        # start to move forward and start the goToBox behaviour
        self.setWheelSpeeds( 8.0, 8.0 )
        self.behaviour = lambda x=self.simTime : self.goToBox(x)


    def goToBox(self, startTime):

        # when we detect light on one of the light sensors we have reached the box
        if self.greenLevel > 0.99 or self.redLevel > 0.99:

            self.behaviour = lambda : None
            self.schedule( 160, lambda : self.setBehaviour( self.checkBox ) )

        # if we have been driving for too long then give up
        if self.simTime - startTime > 2500:

            self.locationsRoute()

        # if we drive into the home squares by accident then reverse out
        if mag( self.pos - greenCentre ) < 0.2 or mag( self.pos - redCentre ) < 0.2:

            self.releaseBlock()


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
            self.behaviour = lambda : None
            self.schedule( 500, lambda : self.setBehaviour( lambda : self.goToPoint( self.home, self.releaseBlock, tolerance=0.2 ) ) )
            

        else:

            # tell the other robot the location of this block
            self.sendBoxLocation()

            # if its the wrong colour call releaseBlock 
            self.releaseBlock()


    def releaseBlock(self):

        # lift the claw and reverse the motors
        self.clawMotor.setPosition( 0.5 )
        self.setWheelSpeeds( -7.0, -7.0 )

        # schedule locationsRoute and set behaviour to nothing
        self.behaviour = lambda : None
        self.schedule( 1000, self.locationsRoute )

         
    def sendBoxLocation(self):

        # assume block is 0.18 in front of the gps
        boxLoc = self.pos + self.forward * 0.18

        # add the box location to our robots otherColourBoxes list
        self.otherColourBoxes.append( boxLoc )
        
        # turn data in array into c type data for transmission
        message = struct.pack('fff', boxLoc[0], boxLoc[1], boxLoc[2])
        self.emitter.send(message)


    def reverseFromWall(self):

        # re check the stuck conditions
        atEdgeOfArena = abs(self.x) > 0.8 or abs(self.z) > 0.8
        facingWall    = self.forward @ norm(self.pos) > 0
        stopped       = np.linalg.norm( self.vel ) < 0.01

        # if they arent still true then do nothing
        if not (atEdgeOfArena and facingWall and stopped):
            return

        # set the wheels to reverse
        self.setWheelSpeeds( -4.0, -4.0 )

        # set the behaviour to nothing and wait for a bit before resuming the behaviour
        self.schedule( 2000, lambda x=self.behaviour : self.setBehaviour(x) )
        self.behaviour = lambda : None


    def recoverFromStuck(self):

        # sometimes the robot get stuck where the claw is on top of a block and the wheels are off the ground
        if self.y > 0.055:

            # just call releaseBlock to recover
            self.releaseBlock()

        # sometimes we are facing the wall and trying to drive into it
        atEdgeOfArena = abs(self.x) > 0.8 or abs(self.z) > 0.8
        facingWall    = self.forward @ norm(self.pos) > 0
        stopped       = np.linalg.norm( self.vel ) < 0.03

        if atEdgeOfArena and facingWall and stopped:

            # reverse for a while to recover
            # call it after 2 seconds and only do it if we're still stuck
            self.schedule( 2000, self.reverseFromWall )


    def mainLoop(self):

        # loop until simulation end
        while self.step( self.timestep ) != -1:

            self.simTime += self.timestep

            # update the time, distance sensor reading and scheduled tasks
            self.updateColourSensors()            
            self.updatePositionAndVelocity()
            self.updateDistance()
            self.updateSchedule()
            self.updateRecieverEmitter()

            # call the current behaviour function
            self.behaviour()

            # recover from stuck in case we are
            self.recoverFromStuck()

            # can print position and velocity
            # print( "pos: " + " ".join( ["%.2f" % v for v in self.pos] ) )
            # print( "vel: " + " ".join( ["%.2f" % v for v in self.vel] ) )


Jasmine()