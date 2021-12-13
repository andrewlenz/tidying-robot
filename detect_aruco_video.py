#!/usr/bin/env python3

from imutils.video import VideoStream
import cv2
import argparse
import imutils
import time
import sys
import socket
import imagezmq
from math import acos, pi, cos, asin, sin, sqrt

from threading import Thread
import display_writer
import robot_controller
import asyncio
import numpy as np

iter_count = 0
dist_1 = 0
dist_2 = 0

class ARTags:

    global robot1_angle
    global robot2_angle
    # global iter_count

    def __init__(self, object_order):
        self.ref_ID = 0
        self.robot1 = 1
        self.robot2 = 2
        self.object_order = object_order
        self.ref_marker_side_len = 6 #cm
        self.distance_ratio = 0 #centimeter per pixel
        self.num_markers = 7
        self.coordinate_dict = self.initialize_coord_dict()
        self.obstacle_ahead = 0
        self.frame = None
        self.object1, self.object2, self.object3, self.object4 = self.object_order[0], self.object_order[1], self.object_order[2], self.object_order[3]
        # iter_count = 0

    def initialize_coord_dict(self):
        coord_dict = dict({})
        for i in range(self.num_markers):
            coord_dict[i] = [[(0,0),(0,0),(0,0),(0,0)], (0,0), (0,0)]
        return coord_dict

    def euclidean_dist(self, point1, point2):
        '''
        Find the distance between POINT1 and POINT2, both of which are tuples (x,y) in unit pixels
        '''
        return ((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)**(1/2)


    def identify_ref_marker(self, color=(0, 0, 255)):
        '''
        Identifies the top_left_point, top_right_point, bottom_left_point, bottom_right_point, center, front_center of the REFERENCE MARKER
        denoted by int MARKER_ID. FRAME is the current frame recorded by the camera 
        Calculates the distance ratio, aka pixels to cm ratio
        Returns the CENTER of the REFERENCE MARKER
        '''
        # (center, front_center) = self.identify_marker(self.ref_ID, color)
        corners_list = self.coordinate_dict[self.ref_ID][0]
        corners = corners_list.reshape((4, 2))
        edge_len = self.euclidean_dist(corners[0], corners[1])
        self.distance_ratio = self.ref_marker_side_len/edge_len
        # return center

    def identify_marker(self, marker_ID, color=(0, 255, 0)):
        '''
        Find the corners and centers of each marker with MARKER_ID int.
        Make visual boxes around each marker 
        Return a tuple with CENTER and FRONT CENTRE coordinates
        '''
        corners_list = self.coordinate_dict[marker_ID][0]
        corners = corners_list.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners

        # Convert each x and y value into an int
        top_left_point = (int(top_left[0]), int(top_left[1]))
        top_right_point = (int(top_right[0]), int(top_right[1]))
        bottom_left_point = (int(bottom_left[0]), int(bottom_left[1]))
        bottom_right_point = (int(bottom_right[0]), int(bottom_right[1]))

        # Draw ounding box around each AR Tag for visual purposes
        self.frame = cv2.line(self.frame, top_left_point, top_right_point, color, 2)
        self.frame = cv2.line(self.frame, top_right_point, bottom_right_point, color, 2)
        self.frame = cv2.line(self.frame, bottom_right_point, bottom_left_point, color, 2)
        self.frame = cv2.line(self.frame, bottom_left_point, top_left_point, color, 2)

        # Find the center of the AR Tag
        cX = int((top_left[0] + bottom_right[0])//2)
        cY = int((top_left[1] + bottom_right[1])//2)

        # Find the center front of the AR Tag
        front_X = int((top_left[0] + top_right[0])//2)
        front_Y = int((top_left[1] + top_right[1])//2)
        self.frame = cv2.circle(self.frame, (cX, cY), 4, (0, 0, 255), -1)

        return ((cX, cY), (front_X, front_Y))

    def angle_between_markers1(self, from_center, to_front, from_front):
        '''
        Finds the ANGLE that the robot needs to turn to face in the direction of the object,
        using coorindate tuples FROM_CENTER, FROM_FRONT and TO_CENTER and the cosine rule
        '''
        a = self.euclidean_dist(from_front, from_center)
        b = self.euclidean_dist(from_front, to_front)
        c = self.euclidean_dist(from_center, to_front)
        angle = (acos((a**2 + b**2 - c**2) / (2*a*b)) * 180 / pi)
        # angle = 180 - (acos((a**2 + b**2 - c**2) / (2*a*b)) * 180 / pi)
        if ((from_front[0] - from_center[0]) * (to_front[1] - from_center[1])) - ((from_front[1] - from_center[1]) * (to_front[0] - from_center[0])) >= 0:
            return False, angle
        else:
            return True, angle

    def angle_between_markers2(self, negative, intermediate_angle, from_to_distance):

        c = sqrt(from_to_distance**2 + (15/self.distance_ratio)**2 - 2*from_to_distance*(15/self.distance_ratio)*cos(intermediate_angle*pi/180))
        desired_angle = (asin((from_to_distance * sin(intermediate_angle*pi/180))/c))*180/pi
        if negative:
            return -desired_angle
        else:
            return desired_angle


    def update_corners(self, marker_dict, marker_params):
        '''
        MARKER_DICT is a dictionary of all the markers provided by the Aruco library. 
        MARKER_PARAMS is provided by the Aruco library as well.
        Identifies all the markers, draws bounding boxes around them and updates the COORDINATE_DICT dictionary
        '''

        # Corners and IDs only shows the AR tags that are visible at that instant
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.frame, marker_dict, parameters=marker_params)
        # print("IDs in the frame right now are ", ids)
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # Therefore coordinate dict should also only have tags that are visible in the FRAME
            #self.coordinate_dict = dict((idx, [corner]) for idx, corner in zip(ids, corners))
            for idx, tag_corners in zip(ids, corners):
                idx_value = self.coordinate_dict[idx]
                idx_value[0] = tag_corners
                center, front_point = self.identify_marker(idx)
                idx_value[1] = center
                idx_value[2] = front_point
            self.identify_ref_marker()

            # loop over the detected ArUCo corners
            # for idx in range(0, self.num_markers):
            #   if idx in ids:
            #       center, front_point = self.identify_marker(idx)
            #       idx_value = self.coordinate_dict[idx]
            #       self.coordinate_dict[idx].append(center)
            #       self.coordinate_dict[idx].append(front_point)
                    # distance_idx_ref = euclidean_dist(ref_center, center)* distanceRatio #cm


    def find_closest_object(self):
        '''
        Determine which OBJECT ROBOT1 and ROBOT2 should pick up first respectively, 
        depending on the order of the blocks and the distance between the robots and the blocks
        '''
        global dist_1
        global dist_2

        # Calculate the distance between the robots and the objects
        dist_11 = self.euclidean_dist(self.coordinate_dict[self.robot1][1], self.coordinate_dict[self.object1][1])
        dist_22 = self.euclidean_dist(self.coordinate_dict[self.robot2][1], self.coordinate_dict[self.object2][1])
        dist_12 = self.euclidean_dist(self.coordinate_dict[self.robot1][1], self.coordinate_dict[self.object2][1])
        dist_21 = self.euclidean_dist(self.coordinate_dict[self.robot2][1], self.coordinate_dict[self.object1][1])

        # Calculate the total distance for each combination of robot-object assignments
        dist_11_22 = dist_11 + dist_22
        dist_12_21 = dist_12 + dist_21

        print("dist_11_22 is ", dist_11_22)
        print("dist_12_21 is ", dist_12_21)

        # Assigns robots to objects based on the shortest total distance traveled
        if dist_11_22 < dist_12_21:
            dist_1 = dist_11
            dist_2 = dist_22
            return {self.robot1: self.object1, self.robot2: self.object2}
        else:
            dist_1 = dist_12
            dist_2 = dist_21
            return {self.robot1: self.object2, self.robot2: self.object1}


    # given a robot, returns the angle to it's target
    def find_angle(self, robot):
        negative, inter_angle = self.angle_between_markers1(self.coordinate_dict[robot][1], self.coordinate_dict[robot.target][2], self.coordinate_dict[robot][2])
        angle = self.angle_between_markers2(negative, inter_angle, self.euclidean_dist(self.coordinate_dict[robot][2], self.coordinate_dict[robot.target][2]))
        return angle
        # except KeyError:
        #   print("Key not found")

# def find_quadrant(robot, object):

# def check_obstacle(frame, robot, object):
#   global obstacleAhead

def setup(cam_src):
    '''
    Sets up the camera to send footage to the server. 
    '''
    sender = imagezmq.ImageSender(connect_to="tcp://localhost:5555")
    cam_id = socket.gethostname()
    vs = VideoStream(src=cam_src, framerate=30).start()
    time.sleep(2.0)

    print("video started")
    start_time = time.time()
    return vs, sender, cam_id

async def update_frame(cam_src, vs, sender, cam_id, new_task, marker_dict, marker_params, robot):
    '''
    Updates the video frame based on the current position of the markers.
    '''
    global iter_count
    global dist_1, dist_2
    while True:

        new_vs = vs.read()
        if np.array_equal(new_vs, new_task.frame):
            print("camera not updating")
        new_task.frame = new_vs

        # If for some reason we cannot find the task frame, try again.
        if new_task.frame is None:
            print("reading video stream failed, trying again")
            vs.stop()
            vs, sender, cami_id = setup(cam_src)
            new_task.frame = vs.read()

        new_task.frame = imutils.resize(new_task.frame, width=1000)

        new_task.update_corners(marker_dict, marker_params)

        if iter_count == 0:
            object_allocate = new_task.find_closest_object()
            print(object_allocate)
            print("robot 1 is closest to object", object_allocate[new_task.robot1])
            print("robot 2 is closest to object", object_allocate[new_task.robot2])
            robot.target = object_allocate[new_task.robot1]
        else:
            dist_1 = new_task.euclidean_dist(new_task.coordinate_dict[new_task.robot1][1], new_task.coordinate_dict[robot.target][1])
            dist_2 = new_task.euclidean_dist(new_task.coordinate_dict[new_task.robot2][1], new_task.coordinate_dict[object_allocate[new_task.robot2]][1])

            

        # if iter_count % 10 == 0:
        if len(new_task.object_order) > 0:

            # robot has picked up object and is waiting
            if robot.pick and robot.wait:
                robot.target = new_task.ref_ID
                robot.wait = 0

            negative, robot1_angle = new_task.angle_between_markers1(new_task.coordinate_dict[new_task.robot1][1], new_task.coordinate_dict[object_allocate[new_task.robot1]][2], new_task.coordinate_dict[new_task.robot1][2])
            robot1_angle = new_task.angle_between_markers2(negative, robot1_angle, new_task.euclidean_dist(new_task.coordinate_dict[new_task.robot1][2], new_task.coordinate_dict[object_allocate[new_task.robot1]][2]))
            # print("Final angle is ",robot1_angle)
            await robot.edit_angle(robot1_angle)

            await robot.edit_ready()

            # at object
            if dist_1 < 50:
                print("At object")
                await robot.edit_arrived()

            # close enough to start driving cautiously
            else if dist_1 < 200:
                print("Close to the object")
                await robot.edit_drive_cautious()




            if len(new_task.object_order) > 1:
                robot2_angle = new_task.angle_between_markers1(new_task.coordinate_dict[new_task.robot2][1], new_task.coordinate_dict[object_allocate[new_task.robot2]][2], new_task.coordinate_dict[new_task.robot2][2])



        iter_count+=1
        sender.send_image(cam_id, new_vs)
        await asyncio.sleep(0)
    
async def main(cam_src, order=[3, 4, 5, 6]):
    marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    marker_params = cv2.aruco.DetectorParameters_create()
    vs, sender, cam_id = setup(cam_src)
    
    robot = display_writer.RobotController()
    # await robot.disconnect()
    await robot.client.connect()

    new_task = ARTags(order)
    task_1 = asyncio.create_task(update_frame(cam_src, vs, sender, cam_id, new_task, marker_dict, marker_params, robot))
    task_2 = asyncio.create_task(robot.send())
    print("Starting awaits")

    while True:
        await task_1
        await task_2

    print("exited loop for some reason")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()

    ap.add_argument("-s", "--cam_src", type=int, required=True, help="Source camera ID")
    ap.add_argument("-o", "--order", nargs='+', type=int, required=False, help="Desired order of the objects. This should be a subset of the numbers 3, 4, 5, and 6. For instance, '-o 3 5 4 6'.")
    args = vars(ap.parse_args())
    asyncio.run(main(args["cam_src"], [int(_) for _ in args["order"]]))
