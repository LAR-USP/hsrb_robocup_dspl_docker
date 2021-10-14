#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import rospy
import tf
from utils import *
import msg_listener
import numpy as np
import requests
import base64
import ast

rgbd = RGBD()

locations = {
    'start' : [0,0,0],
    'mesinha': [0.05, 1.11, 90],
    'mesonaA': [1.57, 0.2, -90],
    'mesonaB': [1.2, 0.98, 90],
    'prateleira': [2.25, 4.0, 90],
    'prateleira_pickup': [2.25, 4.15, 90],
    'delivery_area': [0.643, 3.527, 180],
    'person left': [0.547, 2.892, 180],
    'person right': [0.547, 3.851, 180],
    'door': [2.52, 1.17, 90]
}

def move_to_location(loc):
    move_arm_init()
#     move_head_tilt(0)
    move_head_tilt(-1.3)
    x, y, yaw = locations[loc]
    move_base_goal(x, y, yaw)

def request_classification(img):
    url = 'http://172.19.0.1:5000/detect_objs'
    myobj = {
        "img":str(base64.b64encode(img.ravel(), 'utf-8')),
        "h": img.shape[0],
        "w": img.shape[1],    
        "c": img.shape[2]
    }
    return ast.literal_eval(requests.post(url, json= myobj).text)

def imgcoords2world(x, y, rgbd_obj):
    img = rgbd_obj.get_image()
    points_data = rgbd_obj._points_data
    xw = points_data['x'][y, x]
    yw = points_data['y'][y, x]
    zw = points_data['z'][y, x]
    return (xw, yw, zw)

def get_centroid(top_left, bottom_right):
    c_x = top_left[0] + (bottom_right[0] - top_left[0])/2
    c_y = top_left[1] + (bottom_right[1] - top_left[1])/2
    return (int(c_x), int(c_y))

class ObjectTracker():
    def __init__(self):
        self._br = tf.TransformBroadcaster()

        self._cloud_sub = rospy.Subscriber(
            "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
            PointCloud2, self._cloud_cb)
        self._frame_name = "objeto"
        self.x = 0
        self.y = 0
        self.z = 0
        
    def set_obj_coords(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)            

        self._br.sendTransform(
            (self.x, self.y, self.z), tf.transformations.quaternion_from_euler(0, 0, 0),
            rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
            self._frame_name,
            msg.header.frame_id)

def main():
    rospy.init_node("recognition")    

    wp_estante_cima = [2.22, 4.3, 0.92]
    wp_estante_baixo = [2.22, 4.3, 0.62]

    whole_body.set_workspace([-1.0, 1.0, -0.3, 0.3])
    tilt_angles = [-0.4, -0.35, -0.3]
    curr_t_a = 0

    move_to_location('door')
    move_to_location('prateleira')
        
    objeto, pessoa = msg_listener.listener()
    print("Pegando objeto: %s" % (objeto))
    print("Levando para: %s" % (pessoa))

    if objeto == "pear":
        objeto = "i_cups"

    xs, ys, zs = [], [], []
    found_obj = False
    while not found_obj and (curr_t_a < len(tilt_angles)):     
        angle = tilt_angles[curr_t_a]
        move_head_tilt(angle)
        result = request_classification(rgbd.get_image())
        if objeto in result:
            obj = result[objeto][0]
            obj_img_center = get_centroid((obj[0], obj[1]), (obj[2], obj[3]))
            x, y, z = imgcoords2world(obj_img_center[0], obj_img_center[1], rgbd)
    #         xs.append(x)
    #         ys.append(y)
    #         zs.append(z)
            found_obj = True
        else:
            curr_t_a += 1

    if not found_obj:
        print("Object not found. Selecting random object")
        objeto = list(result.keys())[0]
        print("Object selected: " + objeto)
        obj = result[objeto][0]
        obj_img_center = get_centroid((obj[0], obj[1]), (obj[2], obj[3]))
        x, y, z = imgcoords2world(obj_img_center[0], obj_img_center[1], rgbd)
    

    objt = ObjectTracker()
    objt.set_obj_coords(x, y, z)

    trans = get_relative_coordinate("odom", "objeto")
    x = trans.translation.x
    y = trans.translation.y
    z = trans.translation.z

    target = np.array([x, y, z])
    if z > 0.7:
        print("Usando waypoint cima!")
        starting_wp = wp_estante_cima
    else:
        print("Usando waypoint baixo!")
        starting_wp = wp_estante_baixo
    wp1 = target + np.array([0, -0.1, 0.075])
    wp2 = target + np.array([0, 0, 0.075])    

    move_wholebody_ik(starting_wp[0], starting_wp[1], starting_wp[2], -90, 0, -90)    
    move_wholebody_ik(wp1[0], wp1[1], wp1[2], -90, 0, -90)
    move_hand(1)
    move_wholebody_ik(x, y, z, -90, 0, -90)
    move_hand(0)
    move_wholebody_ik(wp2[0], wp2[1], wp2[2], -90, 0, -90)
    move_wholebody_ik(wp1[0], wp1[1], wp1[2], -90, 0, -90)
    move_wholebody_ik(starting_wp[0], starting_wp[1], starting_wp[2], -90, 0, -90)

    move_arm_init()
    move_to_location(pessoa)
    move_arm_neutral()
    move_hand(1)
    move_hand(0)
    move_arm_init()

if __name__ == "__main__":
    main()