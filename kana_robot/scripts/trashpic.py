#!/usr/bin/python3

import pyrealsense2 as rs
import rospy
import numpy as np
import cv2
import yolov7
import math
import time
from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Image

def travel(depth):
    global pub_move, move, linvel, camheight, gap

    # using Pythogoras theorem to find the distance bertween robot and the trash
    distance = math.sqrt(depth**2 - camheight**2) - gap
    move.linear.x = linvel
    move.angular.z = 0
    print("Velocity: ", str(linvel))
    pub_move.publish(move)
    time.sleep(distance / linvel)
    move.linear.x = 0
    pub_move.publish(move)
    time.sleep(3)



def finddepth(boxes):
    global depth_frame, color_image

    points = []

    for box in boxes:
        # image = cv2.rectangle(color_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255, 0, 0), 1)
        # cv2.imshow("Frame", image)
        # cv2.waitKey(1)
        if box[0]>=470 and box[0]<=620:
            point_x = int((box[0] + box[2]) / 2)
            point_y = int((box[1] + box[3]) / 2)
            points.append((point_x, point_y))
    points = sorted(points, key=lambda x: x[1], reverse=True)

    for point in points:
        depth = depth_frame.get_distance(*point)
        print("Inside finddepth: ", str(depth))
        travel(depth)
    
    # xval = (np.sum(orderedboxes, where=[True, False, True, False], axis=1) / 2).astype(int)
    # yval = (np.sum(boxes, where=[False, True, False, True], axis=1) / 2).astype(int)


def start_trashpic():
    global pipeline, depth_frame, rate, move, pub_move, angvel, color_image

    try:
        while not rospy.is_shutdown():

            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            
            move.angular.z = angvel
            move.linear.x = 0
            pub_move.publish(move)
            # Convert images to numpy arrays
            # depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            results = model(color_image)

            # parse results
            predictions = results.pred[0]
            boxes = predictions[:, :4] # x1, y1, x2, y2
            # scores = predictions[:, 4]
            # categories = predictions[:, 5]

            if len(boxes) > 0:
                finddepth(boxes)
            
            rate.sleep()

            # show detection bounding boxes on image
            # results.show()

            # point = (int((boxes[0][0] + boxes[0][2]) / 2), int((boxes[0][1] + boxes[0][3]) / 2))
            # print("Point: ", point)
            # depth = depth_frame.get_distance(*point)
            # # print(depth)
            # display_img = cv2.circle(color_image, point, 20, (255,0,0), 2)

            # cv2.imshow("Depth of the Point", display_img)
            # cv2.waitKey(1)

    finally:
        pipeline.stop()

if __name__ == '__main__':

    rospy.init_node("trashpic")
    rate = rospy.Rate(10) # 10hz
    pub_move = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    move = Twist()
    linvel = 0.2    # linear velocity of the robot im m/s
    angvel = 0.4   # angular velocity of the robot in m/s
    camheight = 0.6    # camera height from the ground in metres
    gap = 0.35    # distance between camera and the centre of trash collector in metres

    model = yolov7.load('/home/ubuntu/ros_ws/src/kana/kana_robot/scripts/yolov7/best.pt')
    model.conf = 0.7  # NMS confidence threshold
    model.iou = 0.45  # NMS IoU threshold
    model.classes = None  # (optional list) filter by class

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipe_profile = pipeline.start(config)
    
    start_trashpic()
