import RPi.GPIO as GPIO
import time
import csv
import sys

csvfile = "ultra_distance300.csv"

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

def get_distance():
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

    return [distanceR, distanceL]


while True:
    distance = get_distance()
    timeD = time.strftime("%I") + ':' + time.strftime("%M") + ':' + time.strftime("%S")
    data = [distance, timeD]

    with open(csvfile, "a") as output:
        writer = csv.writer(output, delimiter=",", lineterminator='\n')
        writer.writerow(data)
        time.sleep(5)
