import RPi.GPIO as GPIO
import time
import csv
import sys, os
sys.path.append("/usr/local/lib/")
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, default_still_waiting_callback
from pymavlink import mavutil

#-- Connect to the vehicle
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = "/dev/ttyACM0"
baud_rate = 57600
print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

#ultrasonic connect
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

TRIGR = 23
ECHOR = 24
TRIGL = 20
ECHOL = 16

GPIO.setup(TRIGR, GPIO.OUT)
GPIO.setup(ECHOR, GPIO.IN)
GPIO.setup(TRIGL, GPIO.OUT)
GPIO.setup(ECHOL, GPIO.IN)

GPIO.output(TRIGR, True)
GPIO.output(TRIGR, False)
GPIO.output(TRIGL, True)
GPIO.output(TRIGL, False)

##take off##########3
def arm_and_takeoff(altitude):
    print("basic checkup")
    while not vehicle.is_armable:
        print("waiting to initialize")
        time.sleep(1)
    print("arming motors")
    vehicle.mode=VehicleMode("GUIDED")
    vehicle.armed=True

    while not vehicle.armed:
        print("waiting to be armed")
        time.sleep(1)
    print(vehicle.home_location)
    print("taking off")
    vehicle.simple_takeoff(altitude)

    while True:
        print("altitude: {val}".format(val=vehicle.location.global_relative_frame.alt))
        
        if vehicle.location.global_relative_frame.alt>=altitude*0.95:
            print("target altitude reached")
            break
        time.sleep(1)

##########################################################################ULTRASONIC#########################
def get_distanceR():
    GPIO.output(TRIGR, True)
    time.sleep(0.0001)
    GPIO.output(TRIGR, False)

    while GPIO.input(ECHOR) == False:
        start = time.time()

    while GPIO.input(ECHOR) == True:
        end = time.time()

    sig_time = end - start

    distanceR = sig_time * 17150
    distanceR = round(distanceR, 2)

    print('DistanceR: %.1f cm' % distanceR)
    time.sleep(0.0001)
    return distanceR

def get_distanceL():
    GPIO.output(TRIGL, True)
    time.sleep(0.0001)
    GPIO.output(TRIGL, False)

    while GPIO.input(ECHOL) == False:
        start = time.time()

    while GPIO.input(ECHOL) == True:
        end = time.time()

    sig_time = end - start

    distanceL = sig_time * 17150
    distanceL = round(distanceL, 2)

    print('DistanceL: %.1f cm' % distanceL)
    time.sleep(0.0001)
    return distanceL
##################################control#########################################
def send_ned_velocity(Vx, Vy, Vz, duration):
    print("Velocity command recieved")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,      
        0, 0,  
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
        0b0000111111000111, 
        0, 0, 0, 
        Vx, Vy, Vz, 
        0, 0, 0, 
        0, 0)    

    # send command to vehicle on 1 Hz cycle
    for i in range(0,duration):
        vehicle.send_mavlink(msg)
        
arm_and_takeoff(2)

        #condition
DR = get_distanceR 
DL = get_distanceL
print ("start can MODE")
#moveforward
for DR, DL in range(150, 350):
    if ((DR-DL) < 20) or ((DL-DR) < 20):
        print("Move forward DistanceR: %.1f cm" % DR "DistanceL: %.1f cm" % DL)
        send_ned_velocity(0.5,0,0,5)
        time.sleep(5)
    elif (DR or DL) == 150:
        print("Range for scan")
        break
time.sleep(5)
#scan 
for DR, DL in range(140, 160):
    print('SCan mode DistanceR: %.1f cm' % DR 'DistanceL: %.1f cm' % DL)
    send_ned_velocity(0,0.2,0,300)
    if DL > 200:
        print("end of right")
        time.sleep(5)
        send_ned_velocity(0,0,-0.2,10)
        send_ned_velocity(0,-0.5,0,300)
        break
        
    elif DR > 200:
        print("end of Left")
        time.sleep(5)
        send_ned_velocity(0,0,-0.2,10)
        send_ned_velocity(0,0.5,0,300)
        break
        
#aviod
while (DR or DL) < 140:
    print('TOO close object DistanceR: %.1f cm' % DR 'DistanceL: %.1f cm' % DL)
    send_ned_velocity(-0.5,0,0,300)