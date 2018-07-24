from gps import *
import time
import threading
import math
import wiringpi2 as pi
import random
import numpy as np
import RPi.GPIO as GPIO

class LatLon: 
        RADIUS = 20000 / math.pi # radius of the earth 
        def __init__( self, lat, lon ):
                self.latitude = lat;
                self.longitude = lon;
        def __str__( self ):
                return str(self.latitude) + ',' +str(self.longitude) + ','
        def distDirection(self, to):
                xFrom = self.longitude
                yFrom = self.latitude
                xTo = to.longitude
                yTo = to.latitude
                deltaX = xTo - xFrom
                sinX = math.sin(math.radians(deltaX))
                cosX = math.cos(math.radians(deltaX))
                sinY1 = math.sin(math.radians(yFrom))
                cosY1 = math.cos(math.radians(yFrom))
                sinY2 = math.sin(math.radians(yTo))
                cosY2 = math.cos(math.radians(yTo))
                tanY2 = math.tan(math.radians(yTo))
                direction = 90 - math.degrees(math.atan2(cosY1 * tanY2 - sinY1 * cosX, sinX)) # [deg]
                while(direction<0):
                        direction = direction + 360
                while(direction>=360):
                        direction = direction - 360
                dist = 1000 * self.RADIUS * math.acos(sinY1 * sinY2 + cosY1 * cosY2 * cosX) # [m]
                return dist,direction
#
class Maneuver(object): 
        def __init__(self, duration):
                self.duration = duration # [sec]
                self.name = "man"
        def __str__(self):
                return self.name
        def execute( self, sailCntl, rudderCntl, goalAngle):
                time.sleep(self.duration)
        
class ManeuverTurnSlow(Maneuver):
        def __init__(self, duration):
                super(ManeuverTurnSlow, self).__init__(duration)
                self.name = "TurnSlow"
        def execute( self, sailCntl, rudderCntl, goalAngle ):
                sailCntl.fullClose()
                time.sleep(0.1)
                rudderCntl.fullToGoal(goalAngle)
                time.sleep(self.duration*0.75)
                rudderCntl.center()
                time.sleep(self.duration*0.25)
                
class ManeuverTurnFast(Maneuver):
        def __init__(self, duration):
                super(ManeuverTurnFast, self).__init__(duration)
                self.name = "TurnFast"
        def execute( self, sailCntl, rudderCntl, goalAngle ):
                sailCntl.fullOpen()
                time.sleep(0.1)
                rudderCntl.fullToGoal(goalAngle)
                time.sleep(self.duration*0.75)
                rudderCntl.center()
                time.sleep(self.duration*0.25)
                
class ManeuverSlowDec(Maneuver):
        def __init__(self, duration):
                super(ManeuverSlowDec, self).__init__(duration)
                self.name = "SlowDec"
        def execute( self, sailCntl, rudderCntl, goalAngle ):
                sailCntl.openSail()
                time.sleep(0.1)
                rudderCntl.center()
                time.sleep(self.duration)
                
class ManeuverSlowAcc(Maneuver):
        def __init__(self, duration):
                super(ManeuverSlowAcc, self).__init__(duration)
                self.name = "SlowAcc"
        def execute( self, sailCntl, rudderCntl, goalAngle ):
                sailCntl.closeSail()
                time.sleep(0.1)
                rudderCntl.center()
                time.sleep(self.duration)
                
class ManeuverFastAcc(Maneuver):
        def __init__(self, duration):
                super(ManeuverFastAcc,self).__init__(duration)
                self.name = "FastAcc"
        def execute( self, sailCntl, rudderCntl, goalAngle ):
                sailCntl.closeSail()
                time.sleep(0.1)
                rudderCntl.toGoal(goalAngle)
                time.sleep(self.duration*0.25)
                rudderCntl.center()
                time.sleep(self.duration*0.75)
                
class ManeuverKeepGoing(Maneuver):
        def __init__(self, duration):
                super(ManeuverKeepGoing, self).__init__(duration)
                self.name = "KeepGoing"
        def execute( self, sailCntl, rudderCntl, goalAngle ):
                sailCntl.openSail()
                time.sleep(0.1)
                rudderCntl.toGoal(goalAngle)
                time.sleep(self.duration*0.25)
                rudderCntl.center()
                time.sleep(self.duration*0.75)
                
class ManeuverGoDownward(Maneuver):
        def __init__(self, duration):
                super(ManeuverGoDownward, self).__init__(duration)
                self.name = "GoDownward"
        def execute( self, sailCntl, rudderCntl, goalAngle ):
                sailCntl.openSail()
                rudderCntl.opposite(goalAngle)
                time.sleep(0.1)
                time.sleep(self.duration*0.25)
                rudderCntl.center()
                time.sleep(self.duration*0.75)
#
class SailController:
        def __init__(self, initValue):
                self.value = initValue
                pi.pwmWrite( SAIL_SERVO_PIN, self.value )
        def fullClose(self):
                self.value = SAIL_FULLCLOSE
                pi.pwmWrite( SAIL_SERVO_PIN, self.value)
        def closeSail(self):
                self.value = self.value - 4
                if(self.value < SAIL_FULLCLOSE):
                        self.value = SAIL_FULLCLOSE
                pi.pwmWrite( SAIL_SERVO_PIN, self.value)
        def openSail(self):
                self.value = self.value + 4
                if(self.value > SAIL_FULLOPEN):
                        self.value = SAIL_FULLOPEN
                pi.pwmWrite( SAIL_SERVO_PIN, self.value)
        def fullOpen(self):
                self.value = SAIL_FULLOPEN
                pi.pwmWrite( SAIL_SERVO_PIN, self.value)
#
class RudderController:
        def __init__(self, initValue):
                pi.pwmWrite( RUDDER_SERVO_PIN, initValue)
        def fullToGoal(self, goalAngle): 
                if(goalAngle < 180):
                        f.write(str(RUDDER_FULLRIGHT) + ", ")
                        pi.pwmWrite( RUDDER_SERVO_PIN, RUDDER_FULLRIGHT)
                else:
                        f.write(str(RUDDER_FULLLEFT) + ", ")
                        pi.pwmWrite( RUDDER_SERVO_PIN, RUDDER_FULLLEFT)
        def center(self): 
                f.write(str(RUDDER_FULLLEFT) + ", ")
                pi.pwmWrite( RUDDER_SERVO_PIN, RUDDER_CENTER)
        def toGoal(self, goalAngle):
                if(goalAngle<10 or goalAngle>350):
                        self.center()
                elif(goalAngle<180):
                        f.write(str(RUDDER_RIGHT) + ", ")
                        pi.pwmWrite( RUDDER_SERVO_PIN, RUDDER_RIGHT)
                else:
                        f.write(str(RUDDER_LEFT) + ", ")
                        pi.pwmWrite( RUDDER_SERVO_PIN, RUDDER_LEFT)
        def opposite(self, goalAngle): 
                if(goalAngle<180):
                        f.write(str(RUDDER_LEFT) + ", ")
                        pi.pwmWrite( RUDDER_SERVO_PIN, RUDDER_LEFT)
                else:
                        f.write(str(RUDDER_RIGHT) + ", ")
                        pi.pwmWrite( RUDDER_SERVO_PIN, RUDDER_RIGHT)

class LedController(threading.Thread):
    def __init__(self, PIN):
        super(LedController, self).__init__()
        self.flushing = False
        self.pin = PIN
        self.stateStr = "NONE"
    def __str__(self):
        return self.stateStr
    def turnOn(self):
        self.stateStr = "ON"
        self.flushing = False
        GPIO.output(self.pin,GPIO.HIGH)
    def turnOff(self):
        self.stateStr = "OFF"
        self.flushing = False
        GPIO.output(self.pin,GPIO.LOW)
    def flush(self):
        self.stateStr = "FLUSH"
        self.flushing = True
    def run(self):
        while True:
            if self.flushing == True:
                GPIO.output(self.pin, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(self.pin, GPIO.LOW)
                time.sleep(0.5)        
#
class GpsReceiver(threading.Thread):
        def __init__( self ):
            threading.Thread.__init__( self )
            self.dead = 0
            self.gpsd = gps(mode=WATCH_ENABLE)
            self.running = False

        def run( self ):
            self.running = True
            while self.running:
                # grab EACH set of gpsd info to clear the bufffer
                self.gpsd.next()

        def stopController( self ):
            self.running = False
        
        @property
        def fix(self):
             return self.gpsd.fix

        @property
        def utc(self):
            return self.gpsd.utc

        @property
        def satellites(self):
            return self.gpsd.satellites




if __name__ == '__main__':
        filename = 'GPSdata!_Saiko01.txt'
        #
        SAIL_SERVO_PIN = 19 # pin number
        SAIL_FULLCLOSE = 35 # pwm
        SAIL_FULLOPEN = 55  # pwm
        #
        RUDDER_SERVO_PIN = 12 # pin number
        RUDDER_FULLRIGHT = 41 # pwm
        RUDDER_RIGHT = 48 # pwm
        RUDDER_CENTER = 57 # pwm
        RUDDER_LEFT = 70 # pwm
        RUDDER_FULLLEFT = 75 # pwm
        #
        LED_RED = 25 # pin number
        LED_GREEN = 23 # pin number
        LED_BLUE = 24 # pin number
        #
        gpsc = GpsReceiver()
        #
        SPEED_LOW = 0.3 # [m/sec]
        #
        maneuverDuration = 2 # [sec]
        
        try:
            gpsc.start()
            deadCount = 0
            #
            startPosition = LatLon(35.7938358, 139.6929069)
            goal1Position = LatLon(35.7938690, 139.6934678)
            goal2Position = LatLon(35.7935399, 139.6933149)
            goal = goal1Position
            #
            f = open('GPSdata!_Saiko00.txt', 'w')
            f.write("\n")
            f.write(str(time.ctime())+"\n")
            check = 0
            #
            speedPrev = 0
            newTrack = 0
            goalComponentPrev = 0
            listPosition = [LatLon(0, 0)]
            #
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(LED_RED, GPIO.OUT)
            GPIO.setup(LED_GREEN, GPIO.OUT)
            GPIO.setup(LED_BLUE,GPIO.OUT)
            pi.wiringPiSetupGpio()
            pi.pinMode(SAIL_SERVO_PIN,2)
            pi.pinMode(RUDDER_SERVO_PIN,2)
            pi.pwmSetMode(0)
            pi.pwmSetRange(1024)
            pi.pwmSetClock(375)
            #
            
            sailValue = (SAIL_FULLCLOSE + SAIL_FULLOPEN)/2
            if(SAIL_FULLCLOSE < 35 or SAIL_FULLOPEN > 55 or SAIL_FULLCLOSE > SAIL_FULLOPEN): exit()
            sailCntl = SailController(sailValue)
            
            #
            time.sleep(30)
            #
            rudderValue = RUDDER_CENTER
            if(rudderValue < 40 or rudderValue > 76 or RUDDER_FULLRIGHT < 40 or RUDDER_FULLLEFT > 76 or RUDDER_FULLRIGHT > RUDDER_FULLLEFT): exit()
            
            rudderCntl = RudderController(rudderValue)
            
            #
            time.sleep(30)
            #
            maneuver = 0
            #
            lcRed = LedController(LED_RED)
            lcGreen = LedController(LED_GREEN)
            lcBlue = LedController(LED_BLUE)
            lcRed.start()
            lcGreen.start()
            lcBlue.start()
            #
            startTime = time.time()
            
            while True:
                    #print('check1')
                    presentPosition = LatLon(gpsc.fix.latitude, gpsc.fix.longitude) #gpsc.fix.latitude, gpsc.fix.longitude
                    speed = gpsc.fix.speed#random.uniform(0.1, 0.5) #speed gpsc.fix.speed
                    track = gpsc.fix.track#random.uniform(0, 360) #shinnkouhoukou #gpsc.fix.track
                    #print(presentPosition)
                    #print(speed)
                    #print(track)
                    #print('check2')
                    if(math.isnan(track) == False):
                            listPosition.append(presentPosition)
                            distance, goalDirection = presentPosition.distDirection(goal)
                            goalAngle = goalDirection - track
                            
                    if(math.isnan(track) == True):
                            reliablePosition = listPosition[-1]
                            distance, newTrack = reliablePosition.distDirection(presentPosition)
                            distance, goalDirection = presentPosition.distDirection(goal)
                            track = newTrack
                            goalAngle = goalDirection - track 
                    
                    while( goalAngle < 0 ):
                            goalAngle = goalAngle + 360
                    while( goalAngle >= 360):
                            goalAngle = goalAngle - 360
                    #
                    goalComponent = speed * math.cos(math.radians(goalAngle))
                    #  
                    if(goalComponent < 0):
                        lcRed.turnOn()
                    else:
                        lcRed.turnOff()
                    if (goalComponent > goalComponentPrev):
                        lcBlue.turnOn()
                    else:
                        lcBlue.turnOff()
                    if(goalAngle < 10 or goalAngle > 350):
                        lcGreen.flush()
                    elif(10 <= goalAngle < 180):
                        lcGreen.turnOn()
                    else:
                        lcGreen.turnOff()
                    #
                    if(goalComponent < 0):
                            if(speed < SPEED_LOW):
                                    maneuver = ManeuverTurnSlow(maneuverDuration)
                            else:
                                    maneuver = ManeuverTurnFast(maneuverDuration)
                    else:
                            if(speed < SPEED_LOW):
                                    if(speed < speedPrev):
                                            maneuver = ManeuverSlowDec(maneuverDuration)
                                    else:
                                            maneuver = ManeuverSlowAcc(maneuverDuration)
                            else:
                                    if(speed < speedPrev):
                                            if(goalComponent < goalComponentPrev):
                                                    maneuver = ManeuverKeepGoing(maneuverDuration)
                                            else:
                                                    maneuver = ManeuverGoDownward(maneuverDuration)
                                    else:
                                            maneuver = ManeuverFastAcc(maneuverDuration)
                    #
                    
                    f.write(str(time.time()-startTime)+", ")
                    f.write(str(presentPosition))
                    #print('latitude, longitude', presentPosition)
                    f.write(str(goal))
                    #print('goal_latitude, goal_longitude', goal)
                    f.write(str(speed)+", ")
                    
                    f.write(str(speedPrev)+", ")
                    f.write(str(goalComponent)+", ")
                    f.write(str(goalComponentPrev)+", ")
                    f.write(str(goalAngle)+", ")
                    f.write(str(goalDirection)+", ")
                    f.write(str(track)+", ")
                    f.write(str(newTrack)+", ")
                    f.write(str(distance)+", ")
                    f.write("RED, " + str(lcRed) + ", ")
                    f.write("GREEN, " + str(lcGreen) + ", ")
                    f.write("BLUE, " + str(lcBlue) + ", ")
                    f.write(str(sailCntl.value) + ", ")
                    f.write(str(maneuver) + ", ")
                    f.write(str(gpsc.utc))
                    f.write("\n")
                    maneuver.execute(sailCntl, rudderCntl, goalAngle)
                    speedPrev = speed
                    goalComponentPrev = goalComponent
                    #
                    if(time.time() > startTime + 1800):
                            goal = startPosition
                            check = 1
                    #
                    if(goal == goal1Position and distance <= 8):
                            goal = goal2Position
                    elif(goal == goal2Position and distance <= 6):
                            goal = startPosition
                    elif(goal == startPosition and distance <= 4):
                            break
                            #if check==1:
                                    #break
                            #else:
                                    #goal = goal1Position
        #Ctrl C
        except KeyboardInterrupt:
                print ("User cancelled")
        finally:
                f.close()
                GPIO.cleanup()
                print ('savefaile_change name !!, savefail_change_name!!')
                print ("Stoppin gps Receivet")
                gpsc.stopController()
                #wait for the tread to finish
                gpsc.join()
                lcRed.join()
                lcGreen.join()
                lcBlue.join()
                print ("Done")
