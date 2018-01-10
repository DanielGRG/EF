# import the necessary packages
from imutils.video.pivideostream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
from PIL import Image, ImageFont, ImageDraw
from RC_control import RC_control
import os
os.system("sudo pigpiod")
import time
import cv2
import sys
import imutils
import serial
import RPi.GPIO as GPIO
import pigpio
import math
import datetime

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print("Camera Warm-UP...")
now = datetime.datetime.now()
vs = PiVideoStream().start()
rc = RC_control()
time.sleep(2.0)
show = False

open('logs/log.txt', 'w').close()
current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
print current_time, "Masina a pornit"
rc.scrie(current_time, " Masina a pornit\n")

def DistanceToCamera(v, h, x_shift, image):
    # camera params
    alpha = 8.0 * math.pi / 180
    v0 = 119.865631204
    ay = 332.262498472

    # compute and return the distance from the target point to the camera
    d = h / math.tan(alpha + math.atan((v - v0) / ay))
    if d > 0:
        cv2.putText(image, "%.1fcm" % d,(image.shape[1] - x_shift, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return d

# Create the haar cascade
faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
stopCascade = cv2.CascadeClassifier("stop_sign.xml")
crossCascade = cv2.CascadeClassifier("crossPedestrian.xml")
parkCascade = cv2.CascadeClassifier("cascade.xml")

# initialize distance
v = 0

# capture frames from the camera
while True:
    # start fps counter
    fps = FPS().start()

    # grab the frame from the threaded video stream and resiz it
    # to have a maximum width of 400 pixels
    frame = vs.read()
    # frame = imutils.resize(frame, width=320)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    font = cv2.FONT_HERSHEY_SIMPLEX
    h1 = 30.0 - 10  # cm

    # Manual control within Bluetooth
    rc.control()

    # Detect faces in the image
    # faces = faceCascade.detectMultiScale(gray, 1.3, 5)
    stops = stopCascade.detectMultiScale(
        gray,
        scaleFactor=1.3,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )
    crosses = crossCascade.detectMultiScale(
        gray,
        scaleFactor=1.3,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )

    parks = parkCascade.detectMultiScale(
        gray,
        scaleFactor=1.3,
        minNeighbors=5,
        minSize=(30, 30),
        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
    )

    
    # Draw a rectangle around the faces
    
    # for (x, y, w, h) in faces:
        # print "FATA a fost detectata!"
        # cv2.rectangle(image, (x, y), (x+w, y+h), (255, 0, 0), 1)
    for (x, y, w, h) in stops:
        current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
        print current_time, "Semnul de STOP a fost detectat!"
##        f.write(current_time + "Semnul de STOP a fost detectat!\n")
        rc.scrie(current_time, " Semnul de STOP a fost detectat!\n")
        cv2.rectangle(frame, (x+5, y+5), (x+w-5, y+h-5), (0, 0, 255), 2)
        cv2.putText(frame, "STOP", (x,y-5), font, 0.5, (0, 0, 255), 1, cv2.CV_AA)
        rc.ser.write("1.png")
        v = y + h - 5
        # print x,y,x+w-5, y+h-5, w, h
        distance = DistanceToCamera(v,h1,300,frame)
        print "Distanta pana la semn: %.1f" %distance, " cm"
##        f.write("Distanta pana la semn: %.1f" %distance + " cm\n")
        rc.scrie(current_time, " Distanta pana la semn: %.1f" %distance + " cm\n")
        
    for (x, y, w, h) in crosses:
        current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
        print current_time, "Semnul de TRECERE PENTRU PIETONI a fost detectat!"
##        f.write(current_time + "Semnul de TRECERE PENTRU PIETONI a fost detectat!\n")
        rc.scrie(current_time, " Semnul de TRECERE PENTRU PIETONI a fost detectat!\n")
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)
        cv2.putText(frame, "CROSS", (x,y-5), font, 0.5, (0, 255, 0), 1, cv2.CV_AA)
        rc.ser.write("2.png")
        v = y + h - 5
        # print x,y,x+w-5, y+h-5, w, h
        distance = DistanceToCamera(v,h1,300,frame)
        print "Distanta pana la semn: %.1f" %distance, " cm"
##        f.write("Distanta pana la semn: %.1f" %distance + " cm\n")
        rc.scrie(current_time, " Distanta pana la semn: %.1f" %distance + " cm\n")
            
    for (x, y, w, h) in parks:
        current_time = time.strftime("<%d/%b/%Y-%H:%M:%S>")
        print current_time, "Semnul de PARCARE a fost detectat!"
##        f.write(current_time + "Semnul de PARCARE a fost detectat!\n")
        rc.scrie(current_time, " Semnul de PARCARE a fost detectat!\n")
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 255), 1)
        cv2.putText(frame, "PARK", (x,y-5), font, 0.5, (255, 0, 255), 1, cv2.CV_AA)
        rc.ser.write("3.png")
        v = y + h - 5
        # print x,y,x+w-5, y+h-5, w, h
        distance = DistanceToCamera(v,h1,300,frame)
        print "Distanta pana la semn: %.1f" %distance, " cm"
##        f.write("Distanta pana la semn: %.1f" %distance + " cm\n")
        rc.scrie(current_time, " Distanta pana la semn: %.1f" %distance + " cm\n")
        
    # show the frame
    cv2.imshow("Frame", frame)

    # update the FPS counter
    fps.update()
    
    # stop the timer and display FPS information
    fps.stop()

    # show the timer and display FPS information
    # show = True
    if(show):
        print"[INFO] elasped time: {:.2f}".format(fps.elapsed())
        print"[INFO] approx. FPS: {:.2f}".format(fps.fps())

    
        # if the `q` key was pressed, break from the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# do a bit of cleanup
rc.inchide()
cv2.destroyAllWindows()
vs.stop()              
rc.p.stop()
rc.pi.set_servo_pulsewidth(4, 1500)
rc.pi.stop()
GPIO.cleanup()
