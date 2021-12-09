#!/usr/bin/env python3

import cv2
import argparse
import numpy as np
import sys

marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

for id in range(0, 7):
	# Add edge cases
	AR_tag = np.zeros((300,300,1), dtype="uint8")
	cv2.aruco.drawMarker(marker_dict, id, 300, AR_tag, 1)
	filename = ("tag_markers/ARtag{}.png").format(str(id))
	cv2.imwrite(filename, AR_tag)
	# cv2.imshow("ARTag1", AR_tag1)
	# cv2.waitKey(0)