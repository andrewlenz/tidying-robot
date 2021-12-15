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
# import display_writer
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
        self.object_order = object_order
        self.ref_marker_side_len = 7 #cm
        self.distance_ratio = 0 #pixels per centimeter
        self.num_markers = 7
        self.coord_dict = self.initialize_coord_dict()
        self.obstacle_ahead = 0
        self.frame = None
        self.object1, self.object2, self.object3, self.object4 = self.object_order[0], self.object_order[1], self.object_order[2], self.object_order[3]
        self.robot1 = None
        self.robot2 = None
        # iter_count = 0

    def initialize_coord_dict(self):
        coord_dict = dict({})
        for i in range(self.num_markers):
            coord_dict[i] = [[(0,0),(0,0),(0,0),(0,0)], (0,0), (0,0)]
        return coord_dict

    def euc_dist(self, point1, point2):
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
        corners_list = self.coord_dict[self.ref_ID][0]
        corners = np.reshape(corners_list, (4, 2))
        edge_len = self.euc_dist(corners[0], corners[1])
        self.distance_ratio = edge_len/self.ref_marker_side_len


    def identify_marker(self, marker_ID, color=(0, 255, 0)):
        '''
        Find the corners and centers of each marker with MARKER_ID int.
        Make visual boxes around each marker 
        Return a tuple with CENTER and FRONT CENTRE coordinates
        '''
        corners_list = self.coord_dict[marker_ID][0]
        corners = corners_list.reshape((4, 2))
        (top_left, top_right, bottom_right, bottom_left) = corners

        # Convert each x and y value into an int
        top_left_point = (int(top_left[0]), int(top_left[1]))
        top_right_point = (int(top_right[0]), int(top_right[1]))
        bottom_left_point = (int(bottom_left[0]), int(bottom_left[1]))
        bottom_right_point = (int(bottom_right[0]), int(bottom_right[1]))

        # Draw bounding box around each AR Tag for visual purposes
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

    def angle_between_markers1(self, from_center, to_center, from_front, robot):
        '''
        Finds the ANGLE that the robot needs to turn to face in the direction of the object,
        using coorindate tuples FROM_CENTER, FROM_FRONT and TO_CENTER and the cosine rule
        '''
        a = self.euc_dist(from_front, from_center)
        b = self.euc_dist(from_front, to_center)
        c = self.euc_dist(from_center, to_center)

        if a == 0 or b == 0 or c == 0 or (abs((a**2 + b**2 - c**2) / (2*a*b)) > 1):
            # print("error in angle calculation w arccos, using last")
            return robot.orient, robot.angle

        angle = (acos((a**2 + b**2 - c**2) / (2*a*b)) * 180 / pi)
        if ((from_front[0] - from_center[0]) * (to_center[1] - from_center[1])) - ((from_front[1] - from_center[1]) * (to_center[0] - from_center[0])) >= 0:
            robot.orient = False
            return False, angle
        else:
            robot.orient = True
            return True, angle

    def angle_between_markers2(self, negative, intermediate_angle, from_to_distance, robot):
        if intermediate_angle == 0:
            # print("angle is 0, skipping")
            return 4
        if intermediate_angle == robot.angle:
            if robot.orient:
                return -robot.angle - 1
            else:
                return robot.angle

        c = sqrt(from_to_distance**2 + (14*self.distance_ratio)**2 - 2*from_to_distance*(14*self.distance_ratio)*cos(intermediate_angle*pi/180))
        sanity = (from_to_distance * sin(intermediate_angle*pi/180))/c
        # print(sanity)
        if abs(sanity) > 1:
            print("error in angle calculation w arcsin, using last")
            if robot.orient:
                return -robot.angle - 1
            else:
                return robot.angle

        else:
            desired_angle = (asin(sanity)*180/pi)


        if negative:
            return -desired_angle - 1
        else:
            return desired_angle


    def update_corners(self, marker_dict, marker_params):
        '''
        MARKER_DICT is a dictionary of all the markers provided by the Aruco library. 
        MARKER_PARAMS is provided by the Aruco library as well.
        Identifies all the markers, draws bounding boxes around them and updates the coord_dict dictionary
        '''

        # Corners and IDs only shows the AR tags that are visible at that instant
        (corners, ids, rejected) = cv2.aruco.detectMarkers(self.frame, marker_dict, parameters=marker_params)
        # print("IDs in the frame right now are ", ids)
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # Therefore coordinate dict should also only have tags that are visible in the FRAME
            #self.coord_dict = dict((idx, [corner]) for idx, corner in zip(ids, corners))
            for idx, tag_corners in zip(ids, corners):
                if idx > 7:
                    return;
                idx_value = self.coord_dict[idx]
                idx_value[0] = tag_corners
                center, front_point = self.identify_marker(idx)
                idx_value[1] = center
                idx_value[2] = front_point
            self.identify_ref_marker()


    def assign_targets(self):
        '''
        Determine which OBJECT ROBOT1 and ROBOT2 should pick up first respectively, 
        depending on the order of the blocks and the distance between the robots and the blocks
        assigns targets to the robots
        '''
        # Calculate the distance between the robots and the objects
        dist_11 = self.euc_dist(self.coord_dict[self.robot1.id][1], self.coord_dict[self.object1][1])
        dist_22 = self.euc_dist(self.coord_dict[self.robot2.id][1], self.coord_dict[self.object2][1])
        dist_12 = self.euc_dist(self.coord_dict[self.robot1.id][1], self.coord_dict[self.object2][1])
        dist_21 = self.euc_dist(self.coord_dict[self.robot2.id][1], self.coord_dict[self.object1][1])

        # Calculate the total distance for each combination of robot-object assignments
        dist_11_22 = dist_11 + dist_22
        dist_12_21 = dist_12 + dist_21

        print("dist_11_22 is ", dist_11_22)
        print("dist_12_21 is ", dist_12_21)

        # Assigns robots to objects based on the shortest total distance traveled
        if dist_11_22 < dist_12_21:
            self.robot1.dist = dist_11
            self.robot2.dist = dist_22
            self.robot1.target = self.object1
            self.robot2.target = self.object2
        else:
            self.robot1.dist = dist_12
            self.robot2.dist = dist_21
            self.robot1.target = self.object2
            self.robot2.target = self.object1
        self.object_order.pop(0)
        self.object_order.pop(0)


    # given a robot, returns the angle to it's target
    def find_angle(self, robot):
        negative, inter_angle = self.angle_between_markers1(self.coord_dict[robot.id][1], self.coord_dict[robot.target][1], self.coord_dict[robot.id][2], robot)
        angle = self.angle_between_markers2(negative, inter_angle, self.euc_dist(self.coord_dict[robot.id][2], self.coord_dict[robot.target][2]), robot)
        return angle




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

async def update_frame(cam_src, vs, sender, cam_id, task, marker_dict, marker_params):
    '''
    Updates the video frame based on the current position of the markers.
    '''
    p = 0
    global iter_count
    while True:

        ## CAMERA STUFF ##
        new_vs = vs.read()
        if np.array_equal(new_vs, task.frame):
            print("camera not updating")
        task.frame = new_vs

        # If for some reason we cannot find the task frame, try again.
        if task.frame is None:
            print("reading video stream failed, trying again")
            vs.stop()
            vs, sender, cami_id = setup(cam_src)
            task.frame = vs.read()

        task.frame = imutils.resize(task.frame, width=1000)
        task.update_corners(marker_dict, marker_params)
        # a = ([((0,0) in i or (0,0) in i[0]) for i in task.coord_dict.values()])
        # print(a)

        # while any([any((0,0) in i or (0,0) in i[0]) for i in task.coord_dict.values()]):
            # print("had to rerun update corners")
            # task.update_corners(marker_dict, marker_params)

        ## CONTROL LOGIC ##

        # start of run, need to get targets
        if iter_count == 0:
            task.assign_targets()
            print("robot 1's target is", task.robot1.target)
            print("robot 2's target is", task.robot2.target)

        # update robots
        for robot in [task.robot1, task.robot2]:
        # for robot in [task.robot2]:
            if not robot.finished:

            # get new distance
                robot.dist = task.euc_dist(task.coord_dict[robot.id][1], task.coord_dict[robot.target][1])

                if robot.pick and not robot.depositing: # robot is holding object and waiting for drop off angle
                    robot.arrived = 0
                    robot.target = task.ref_ID
                    robot.drive_cautious = 0
                    robot.dist = task.euc_dist(task.coord_dict[robot.id][1], task.coord_dict[robot.target][1])
                    print(robot.id, "is is heading to drop off")

                # if robot that has an object has signaled a drop off
                if robot.depositing and not robot.pick and robot.arrived:
                    robot.depositing = 0
                    robot.arrived = 0
                    robot.drive_cautious = 0
                    if len(task.object_order) > 0:
                        robot.target = task.object_order.pop(0)
                        robot.dist = task.euc_dist(task.coord_dict[robot.id][1], task.coord_dict[robot.target][1])
                        print(robot.id, "'s new target is ", robot.target)
                    else:
                        robot.finished = 1
                        robot.ready = 0
                        await robot.disconnect()
                        print(robot.id, " is FINISHED")


                # update robot angles
                angle = task.find_angle(robot)
                # if robot.id == 2 and angle > 0:
                #     angle += 2
                robot.angle = angle

                if robot.depositing == 0: # robot is always ready when not holding something
                    if robot.finished:
                        robot.ready = 0
                    else:
                        robot.ready = 1

                if robot.pick and not robot.depositing and not robot.finished: # update ready and depositing state to start drop off
                    robot.ready = 1
                    robot.depositing = 1

                # at object
                if (robot.dist / task.distance_ratio) < 20 and robot.depositing and robot.id == 1: # signal arrive for drop off
                    if p == 0:
                        print(robot.id, "is at drop off point")
                        p += 1
                    robot.arrived = 1

                # at object
                if (robot.dist / task.distance_ratio) < 20 and robot.depositing and robot.id == 2: # signal arrive for drop off
                    if p == 0:
                        print(robot.id, "is at drop off point")
                        p += 1
                    robot.arrived = 1

                if (robot.dist / task.distance_ratio) < 9.5 and not robot.depositing and robot.id == 1: # signal arrive for pick up
                    if p == 0:
                        print(robot.id, "is at object")
                        p += 1
                    robot.arrived = 1

                if (robot.dist / task.distance_ratio) < 9.5 and not robot.depositing and robot.id == 2: # signal arrive for pick up
                    if p == 0:
                        print(robot.id, "is at object")
                        p += 1
                    robot.arrived = 1


                elif (robot.dist / task.distance_ratio) >= 14 and not robot.depositing: # need to set arrive correctly again in back up
                    robot.arrived = 0
                    robot.drive_cautious = 0

                # close enough to start driving cautiously
                if (robot.dist / task.distance_ratio) < 30 and robot.id == 2:
                    if p == 1:
                        print(robot.id, "is close to the object")
                        p += 1
                    robot.drive_cautious = 1

                if (robot.dist / task.distance_ratio) < 30 and robot.id == 1:
                    if p == 1:
                        print(robot.id, "is close to the object")
                        p += 1
                    robot.drive_cautious = 1

                if robot.target == 100:
                    robot.ready = 0

        iter_count+=1
        sender.send_image(cam_id, new_vs)
        await asyncio.sleep(0.05)
    
async def main(cam_src, order=[3, 4, 5, 6]):
    marker_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    marker_params = cv2.aruco.DetectorParameters_create()
    vs, sender, cam_id = setup(cam_src)
    
    address1 = "C0:98:E5:49:30:01"
    address2 = "C0:98:E5:49:30:02"

    robot1 = robot_controller.RobotController(address1)
    robot1.id = 1
    robot2 = robot_controller.RobotController(address2)
    robot2.id = 2

    while not robot1.client.is_connected:
        try:
            print(f"Trying to connect to {robot1.address}")
            await robot1.client.connect()
        except Exception as e:
            print(f"connection error on 1, address is", address1, "trying again")

    while not robot2.client.is_connected:
        try:
            print(f"Trying to connect to {robot2.address}")
            await robot2.client.connect()
        except Exception as e:
            print(f"connection error on 2, address is", address2, "trying again")

    task = ARTags(order)

    task.robot1 = robot1
    task.robot2 = robot2

    task_1 = asyncio.create_task(update_frame(cam_src, vs, sender, cam_id, task, marker_dict, marker_params))
    task_2 = asyncio.create_task(robot1.send())
    task_3 = asyncio.create_task(robot2.send())
    print("Starting awaits")

    while True:
        await task_1
        await task_2
        await task_3

    print("exited loop for some reason")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()

    ap.add_argument("-s", "--cam_src", type=int, required=True, help="Source camera ID")
    ap.add_argument("-o", "--order", nargs='+', type=int, required=False, help="Desired order of the objects. This should be a subset of the numbers 3, 4, 5, and 6. For instance, '-o 3 5 4 6'.")
    args = vars(ap.parse_args())
    asyncio.run(main(args["cam_src"], [int(_) for _ in args["order"]]))
