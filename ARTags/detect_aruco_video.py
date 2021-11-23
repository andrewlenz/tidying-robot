#!/usr/bin/env python3

from imutils.video import VideoStream
import cv2
import argparse
import imutils
import time
import sys
import socket
import imagezmq

# INITIALIZE GLOBAL VARIABLES
ref_ID = 1
ref_marker_side_len = 4.6 #cm
# global distance_ratio is pixels per cm

# HELPER FUNCTIONS
def euclidean_dist(point1, point2):
	return ((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)**(1/2)


def identify_ref_marker(markerID, markerCorner, frame, distance_ratio, color=(0, 0, 255)):
	(topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint, center) = identify_marker(markerID, markerCorner, frame, color)
	dist = euclidean_dist(topLeftPoint, topRightPoint)
	return (ref_marker_side_len/dist, center)

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
	frame = cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

	return (topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint, (cX, cY))


def main(cam_src):

	distance_ratio = 0

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
	while True:
		frame = vs.read()
		frame = imutils.resize(frame, width=1000)

		(corners, ids, rejected) = cv2.aruco.detectMarkers(frame, marker_dict, parameters=marker_params)

		print("corners are ", corners)
		print("IDs are ", ids)

		num_markers = len(corners)

		if num_markers > 0:
			# flatten the ArUco IDs list
			ids = ids.flatten()
			id_dict = dict((idx, corner) for idx, corner in zip(ids, corners))
			distance_ratio, ref_center = identify_ref_marker(ref_ID, id_dict[ref_ID], frame, distance_ratio)

			# loop over the detected ArUCo corners
			for idx in range(2, num_markers + 1):
				# extract the marker corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
				if idx in ids:
					topLeftPoint, topRightPoint, bottomLeftPoint, bottomRightPoint, center = identify_marker(idx, id_dict[idx], frame)
					distance_idx_ref = euclidean_dist(ref_center, center)* distance_ratio #cm
					print("ID ", idx)
					print("Distance to ref marker ", distance_idx_ref)

		sender.send_image(cam_id, frame)


if __name__ == "__main__":
	ap = argparse.ArgumentParser()

	ap.add_argument("-s", "--cam_src", required=True,
	 help="Source camera ID")
	args = vars(ap.parse_args())
	main(int(args["cam_src"]))