import time
import RPi.GPIO as GPIO
#GPIO Mode (board/bcm)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
#GPIO pin ultrasonic r=right l=left
TRIGR = 16
ECHOR = 18
TRIGL = 38
ECHOL = 36



while True:
    #set trig high
    print("distance measurement in progress")
    # set gpio in/out direction
    GPIO.setup(TRIGR, GPIO.OUT)
    GPIO.setup(ECHOR, GPIO.IN)
    GPIO.setup(TRIGL, GPIO.OUT)
    GPIO.setup(ECHOL, GPIO.IN)
    GPIO.output(TRIGR, False)
    GPIO.output(TRIGL, False)
    print("waiting fot sensor")
    # set trig high
    time.sleep(0.2)
    GPIO.output(TRIGR, True)
    GPIO.output(TRIGL, True)
    #set trig after 0.0005s low
    time.sleep(0.0005)
    GPIO.output(TRIGR, False)
    GPIO.output(TRIGL, False)

    StartTimeR = time.time()
    StopTimeR = time.time()
    StartTimeL = time.time()
    StopTimeL = time.time()
  # save StartTime
    while GPIO.input(ECHOR) == 0:
        StartTimeR = time.time()
    while GPIO.input(ECHOL) == 0:
        StartTimeL = time.time()

    # save time of arrival
    while GPIO.input(ECHOR) == 1:
        StopTimeR = time.time()
    while GPIO.input(ECHOL) == 1:
        StopTimeL = time.time()
    # time difference between start and arrival
    TimeR = StopTimeR - StartTimeR
    TimeL = StopTimeL - StartTimeL
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distanceR = (TimeR * 34300) / 2
    distanceL = (TimeL * 34300) / 2
    print("distanceR",distanceR,"cm  distanceL",distanceL,"cm")
    time.sleep(2)


