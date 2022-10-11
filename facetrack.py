import cv2
import numpy as np
import time
import busio
from board import SCL, SDA
from adafruit_servokit import ServoKit
import mediapipe as mp
from adafruit_pca9685 import PCA9685
from ctypes import cdll
from threading import Timer

def exitfunc():
    os._exit(0)
templib = cdll.LoadLibrary('/home/pi/dts.so')

ANGLE_STEP = 1
kit=ServoKit(channels=16)

kit.servo[0].angle=90
kit.servo[1].angle=90
kit.servo[0].actuation_range = 175
kit.servo[1].actuation_range = 140
kit.servo[0].set_pulse_width_range(750, 2250)
kit.servo[0].set_pulse_width_range(750, 2250)
i2c=busio.I2C(SCL, SDA)

mp_drawing = mp.solutions.drawing_utils
mp_face_detection = mp.solutions.face_detection

face_detection = mp_face_detection.FaceDetection(
    min_detection_confidence=0.7)

cap = cv2.VideoCapture(0)
angle = 90
while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break
    
    img = cv2.flip(img, 1) # mirror image

    results = face_detection.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    if results.detections:
        for detection in results.detections:
            Timer(20, exitfunc).start()
            angle = 0
            mp_drawing.draw_detection(img, detection)

            x1 = detection.location_data.relative_bounding_box.xmin         # left side of face bounding box
            x2 = x1 + detection.location_data.relative_bounding_box.width   # right side of face bounding box
            y1 = detection.location_data.relative_bounding_box.ymin         # down side of face bounding box
            y2 = y1 + detection.location_data.relative_bounding_box.height  # up side of face bounding box

            cx = (x1 + x2) / 2      # center of the face
            cy = (y1 + y2)/ 2       # center of the face

            if cx < 0.4: # left -> clockwise
                angle += ANGLE_STEP
                kit.servo[0].angle = angle
            elif cx > 0.6: # right -> counter clockwise
                angle -= ANGLE_STEP
                kit.servo[0].angle = angle
            if cy < 0.4: # up -> clockwise
                angle += ANGLE_STEP
                kit.servo[1].angle = angle
            elif cy > 0.6: # down -> counter clockwise
                angle -= ANGLE_STEP
                kit.servo[1].angle = angle

            cv2.putText(img, str(tmplib.gettemp())+'℃, '+'%d deg' % (angle), org=(13, 45), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, color=255, thickness=2) 
            # temperature notation
            break

    cv2.imshow('Face Cam', img)     #camera screen output
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()