#####################################################
##          librealsense T265 streams test         ##
#####################################################
# This assumes .so file is found on the same directory
import pyrealsense2.pyrealsense2 as rs

# Prettier prints for reverse-engineering
from pprint import pprint
import numpy as np
import csv
import time
import math as m

csvfile = "t265pose.csv"


# Get realsense pipeline handle
pipe = rs.pipeline()

# Print all connected devices and find the T265
devices = rs.context().devices
for i in range(len(devices)):
    print('Found device:', devices[i].get_info(rs.camera_info.name), ', with serial number: ', devices[i].get_info(rs.camera_info.serial_number))

# Configure the pipeline
cfg = rs.config()

# Prints a list of available streams, not all are supported by each device
print('Available streams:')
pprint(dir(rs.stream))


# Enable streams you are interested in
cfg.enable_stream(rs.stream.pose) # Positional data (translation, rotation, velocity etc)
cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

# Start the configured pipeline
pipe.start(cfg)


try:
    while(1):
        frames = pipe.wait_for_frames()

        # Left fisheye camera frame
        left = frames.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())

        # Right fisheye camera frame
        right = frames.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())

        print('Left frame', left_data.shape)
        print('Right frame', right_data.shape)

        # Positional data frame
        pose = frames.get_pose_frame()
        if pose:
            pose_data = pose.get_pose_data()
            print("\nFrame number: %5.0f" % (pose.frame_number))
            print("Position xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.translation.x, pose_data.translation.y, pose_data.translation.z))
            
            posex=pose_data.translation.x
            posey=pose_data.translation.y
            posez=pose_data.translation.z
            #print("Velocity xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z))
            #print("Accelera xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.acceleration.x, pose_data.acceleration.y, pose_data.acceleration.z))
            w = pose_data.rotation.w
            x = -pose_data.rotation.z
            y = pose_data.rotation.x
            z = -pose_data.rotation.y
            
            pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
            roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
            yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;
            print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))
            time.sleep(0.1)
            timeD = time.strftime("%I") + ':' + time.strftime("%M") + ':' + time.strftime("%S")
            data = ["posex", posex, "posey", posey, "posez", posez, "roll", roll, "pitch", pitch, "yaw", yaw, "time", timeD]
            #create csv file
            with open(csvfile, "a") as output:
                writer = csv.writer(output, delimiter=",", lineterminator='\n')
                writer.writerow(data)
                time.sleep(2)

           
finally:
    pipe.stop()
    



