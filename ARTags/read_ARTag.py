#!/usr/bin/env python3

import cv2
import argparse
import imutils
import sys


marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
AR_tag1 = np.zeros((300,300,1), dtype="uint8")
cv2.aruco.drawMarker(marker_dict, 1, 300, AR_tag1, 1)
cv2.imwrite("./", AR_tag1)
cv2.imshow("ARTag1", ARTag1)
cv2.waitKey(0)

