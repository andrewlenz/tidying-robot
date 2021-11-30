#!/usr/bin/env python3

from imutils.video import VideoStream
import cv2
import argparse
import imutils
import time
import sys
import socket
import imagezmq
from math import acos, pi

# INITIALIZE GLOBAL VARIABLES
refID = 0
refMarkerSideLen = 6 #cm
distanceRatio = 0 #pixels per centimeter
numMarkers = 7
cornerDict = dict({})
objectOrder = [3, 4, 5, 6] # TODO: Make this a stack

# MARKERS
robot1 = 1
robot2 = 2
object1, object2, object3, object4 = objectOrder[0], objectOrder[1], objectOrder[2], objectOrder[3]

# HELPER FUNCTIONS
def euclidean_dist(point1, point2):
	return ((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)**(1/2)


def identify_ref_marker(markerID, markerCorner, frame, color=(0, 0, 255)):
	(topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint, center, frontCenter) = identify_marker(markerID, markerCorner, frame, color)
	dist = euclidean_dist(topLeftPoint, topRightPoint)
	return (refMarkerSideLen/dist, center)

def identify_marker(markerID, markerCorner, frame, color=(0, 255, 0)):
	corners = markerCorner.reshape((4, 2))
	(topLeft, topRight, bottomRight, bottomLeft) = corners

	topLeftPoint = (int(topLeft[0]), int(topLeft[1]))
	topRightPoint = (int(topRight[0]), int(topRight[1]))
	bottomLeftPoint = (int(bottomLeft[0]), int(bottomLeft[1]))
	bottomRightPoint = (int(bottomRight[0]), int(bottomRight[1]))

	frame = cv2.line(frame, topLeftPoint, topRightPoint, color, 2)
	frame = cv2.line(frame, topRightPoint, bottomRightPoint, color, 2)
	frame = cv2.line(frame, bottomRightPoint, bottomLeftPoint, color, 2)
	frame = cv2.line(frame, bottomLeftPoint, topLeftPoint, color, 2)

	cX = int((topLeft[0] + bottomRight[0])//2)
	cY = int((topLeft[1] + bottomRight[1])//2)
	frontX = int((topLeft[0] + topRight[0])//2)
	frontY = int((topLeft[1] + topRight[1])//2)
	frame = cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

	return (topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint, (cX, cY), (frontX, frontY))

def angle_between_markers(from_center, to_center, from_front):
	a = euclidean_dist(from_front, from_center)
	b = euclidean_dist(from_front, to_center)
	c = euclidean_dist(from_center, to_center)
	return acos((a**2 + b**2 - c**2) / (2*a*b)) * 180 / pi

def update_corners(frame, marker_dict, marker_params):
	global cornerDict
	global distanceRatio

	(corners, ids, rejected) = cv2.aruco.detectMarkers(frame, marker_dict, parameters=marker_params)

	if len(corners) > 0:
		# flatten the ArUco IDs list
		ids = ids.flatten()
		cornerDict = dict((idx, [corner]) for idx, corner in zip(ids, corners))
		distanceRatio, ref_center = identify_ref_marker(refID, cornerDict[refID][0], frame, distanceRatio)

		# loop over the detected ArUCo corners
		for idx in range(0, numMarkers):
			# extract the marker corners (which are always returned
			# in top-left, top-right, bottom-right, and bottom-left
			# order)
			if idx in ids:
				topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint, center, frontPoint = identify_marker(idx, cornerDict[idx][0], frame)
				cornerDict[idx].append(center)
				cornerDict[idx].append(frontPoint)
				# distance_idx_ref = euclidean_dist(ref_center, center)* distanceRatio #cm


def find_closest_object(frame):
	try:
		dist_11_22 = euclidean_dist(cornerDict[robot1][1], cornerDict[object1][1])+ euclidean_dist(cornerDict[robot2][1], cornerDict[object2][1])
		dist_12_21 = euclidean_dist(cornerDict[robot1][1], cornerDict[object2][1])+ euclidean_dist(cornerDict[robot2][1], cornerDict[object1][1])

		if dist_11_22 < dist_12_21:
			return {robot1: object1, robot2: object2}
		else:
			return {robot1: object2, robot2: object1}
	except KeyError:
		print(KeyError)

def find_quadrant(robot, object):
	

def main(cam_src):

	global distanceRatio
	count = 0

	marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
	marker_params = cv2.aruco.DetectorParameters_create()

	print("Starting video stream")
	# Maybe do a video capture to display the webcam video?
	vs = VideoStream(src=cam_src)
	sender = imagezmq.ImageSender(connect_to="tcp://localhost:5555")
	cam_id = socket.gethostname()

	vs.start()
	time.sleep(2.0)

	print("video started")
	start_time = time.time()
	while True:
		# Step 1 - find the distance between robots and each marker
		# Step 2 - find the block closest to each of the robots
		# Step 3 - Find the angle
		frame = vs.read()
		frame = imutils.resize(frame, width=1000)

		update_corners(frame, marker_dict, marker_params)

		if count == 0:
			object_allocate = find_closest_object(frame) #, robot1, robot2, object1, object2)
			print("robot 1 is closest to object", object_allocate[robot1])
			print("robot 2 is closest to object", object_allocate[robot2])

		# t = time.time()
		if count % 10 == 0:
			# print(count, t - start_time)
			# start_time = t
			if len(objectOrder) > 0:
				angle1 = angle_between_markers(cornerDict[robot1][1], cornerDict[6][1], cornerDict[robot1][2])
				print(angle1)
				if len(objectOrder) > 1:
					angle2 = angle_between_markers(cornerDict[robot2][1], cornerDict[object_allocate[robot2]][1], cornerDict[robot2][2])
					print(angle1)
		count+=1
		sender.send_image(cam_id, frame)


if __name__ == "__main__":
	ap = argparse.ArgumentParser()

	ap.add_argument("-s", "--cam_src", required=True,
	 help="Source camera ID")
	args = vars(ap.parse_args())
	main(int(args["cam_src"]))