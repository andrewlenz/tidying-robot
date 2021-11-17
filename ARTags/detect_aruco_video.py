#!/usr/bin/env python3

from imutils.video import VideoStream
import cv2
import argparse
import imutils
import time
import sys

marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
marker_params = cv2.aruco.DetectorParameters_create()

print("Starting video stream")
vs = VideoStream(src=0).start()
time.sleep(2.0)

# while True:
# 	print(vs)
# 	vs.read()

