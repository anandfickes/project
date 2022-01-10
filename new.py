import RPi.GPIO as GPIO
import time
import csv
import sys, os
sys.path.append("/usr/local/lib/")
import math
import pyrealsense2.pyrealsense2 as rs 

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

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)
print("\nStart streaming")

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

#send vision positon
def send_vision_position_message(x,y,z,roll,pitch,yaw):
    msg = vehicle.message_factory.vision_position_estimate_encode(
        start_time,	#us	Timestamp (UNIX time or time since system boot)
	    x,	        #Global X position
	    y,          #Global Y position
        z,	        #Global Z position
        roll,	    #Roll angle
        pitch,	    #Pitch angle
        yaw	        #Yaw angle
        #0              #covariance :upper right triangle (states: x, y, z, roll, pitch, ya
        #0              #reset_counter:Estimate reset counter. 
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

#to use lacal position wihtout GPS
def set_default_global_origin():
    msg = vehicle.message_factory.set_gps_global_origin_encode(
        1
        home_lat, 
        home_lon,
        home_alt
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()
#allow to use position with out gps
def set_default_home_position():
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = vehicle.message_factory.set_home_position_encode(
        int(vehicle._master.source_system),
        home_lat, 
        home_lon,
        home_alt,
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

#update time
def update_timesync(ts=0, tc=0):
    if ts == 0:
        ts = int(round(time.time() * 1000))
    msg = vehicle.message_factory.timesync_encode(
        tc,     # tc1
        ts      # ts1
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()
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


try:
    while True:
        start_time=time.time()
        #microseconds = int(round(time.time())

        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()
        if pose:
            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            #print("Frame #{}".format(pose.frame_number))
            #print("T{}".format(data.translation)),
            #print("R{}".format(data.rotation)),
            #print("Velocity: {}".format(data.velocity))
            #print("Acceleration: {}\n".format(data.acceleration))
            #print ("Time", start_time , "FPS", round(.01 / (time.time()- start_time))) #,

            x =  data.translation.x
            y =  data.translation.y
            z =  data.translation.z
        
            Qw = data.rotation.w
            Qx = data.rotation.x
            Qy = data.rotation.z    # permutation of z and y
            Qz = data.rotation.y    # as Chris Anderson proposition in RealSense Github
        

            #LOOKING FORWARD
            roll = math.asin(2.0*(Qx*Qz - Qw*Qy))
            pitch = math.atan2(2.0*(Qy*Qz + Qw*Qx), Qw*Qw -Qx*Qx - Qy*Qy + Qz*Qz)              
            yaw = math.atan2(2.0*(Qx*Qy + Qw*Qz), Qw*Qw + Qx*Qx - Qy*Qy - Qz*Qz)

        
             # Send vectors to Mavlink according to distance and azimuth (above)
            send_vision_position_message(x,y,z,roll,pitch,yaw)
            print "R", ("%0.2f" %roll) , "  P", ("%0.2f" %pitch) ,"  Y", ("%0.2f" %yaw)
               

            #Convert in Degreee for visualisation
            roll *=180/math.pi
            pitch *=180/math.pi           
            yaw *=180/math.pi

            if roll < 0: roll +=360  #ensure it stays between 0 - 360
            if pitch < 0: pitch +=360  #ensure it stays between 0 - 360      
            if yaw < 0: yaw +=360  #ensure it stays between 0 - 360

            #print ("Roll", round(roll) , "Pitch", round(pitch) ,"Yaw", round(yaw))
            #print ("Roll", (roll) , "Pitch", (pitch) ,"Yaw", (yaw))

            time.sleep(.01)
        
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

except KeyboardInterrupt:
    print("INFO: KeyboardInterrupt has been caught. Cleaning up...")     

finally:
    pipe.stop()
    vehicle.close()
    print("INFO: Realsense pipeline and vehicle object closed.")
    sys.exit()




    