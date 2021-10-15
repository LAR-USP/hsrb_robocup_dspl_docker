#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import rospy
import tf2_ros
from utils import *
import msg_listener
import numpy as np
import requests
import base64
import ast
import threading
from geometry_msgs.msg import Pose
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped
from geometry_msgs.msg import Pose

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
    move_head_tilt(0)
    #move_head_tilt(-1.3)
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

class SceneObject():
    def __init__(self, position, base_frame="map", obj_name="objeto"):
        self.br = tf.TransformBroadcaster()

        self.obj_name = obj_name
        self.parent_frame = "map"
        self.set_position(position, base_frame)
        self.rate = rospy.Rate(5)        
        self.publish_position = True
        self.t = threading.Thread(target=self.publish_pos)
        self.t.start()

    def publish_pos(self):
        while self.publish_position:
            self.br.sendTransform(
                self.position,
                tf.transformations.quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                self.obj_name,
                self.parent_frame
            )
            self.rate.sleep()

    def set_position(self, position, base_frame="map"):
        if base_frame != self.parent_frame:
            base_pose = Pose()
            base_pose.position.x = position[0]
            base_pose.position.y = position[1]
            base_pose.position.z = position[2]
            new_pose = self.transform_pose(base_pose, base_frame, self.parent_frame)
            position = (new_pose.position.x, new_pose.position.y, new_pose.position.z)
        self.position = position

    def get_position(self):
        return self.position

    def destroy(self):
        self.publish_position = False
        self.t.join()
        
    def transform_pose(self, input_pose, from_frame, to_frame):

        # **Assuming /tf2 topic is being broadcasted
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
    #     pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.stamp = rospy.Time(0)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(5))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

def init_moveit_collisions():
    print("Adding moveit collision boxes...")
    rospy.sleep(5)
    for i in range(3):
        box_pose = tf2_geometry_msgs.PoseStamped()
        box_pose.header.frame_id = "map"
        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 2.35
        box_pose.pose.position.y = 4.65
        box_pose.pose.position.z = 0.5 + i*0.3
        box_name = "box"+str(i)
        scene.add_box(box_name, box_pose, size=(0.8, 0.30, 0.01))
        rospy.sleep(0.1)
    for i in range(2):
        box_pose = tf2_geometry_msgs.PoseStamped()
        box_pose.header.frame_id = "map"
        box_pose.pose.orientation.x = 0.0
        box_pose.pose.orientation.y = 0.0
        box_pose.pose.orientation.z = 0.0
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 2.35 - 0.4 + 0.8*i
        box_pose.pose.position.y = 4.65
        box_pose.pose.position.z = 0.8
        box_name = "box"+str(i+3)
        scene.add_box(box_name, box_pose, size=( 0.01, 0.30, 0.8))
        rospy.sleep(0.1)
    if len(scene.get_known_object_names()) == 5:
        print("All collision boxes added")
    else:
        print("Error adding collision boxes")
        print(scene.get_known_object_names())
        
def main():
    rospy.init_node("go_and_get_it_maestro")

    whole_body.set_workspace([-0.3, -1.0, 0.3, 1.0])
    tilt_angles = [-0.4, -0.35, -0.3]
    curr_t_a = 0

    

    # move_to_location('door')
    move_to_location('prateleira')

    # MUDAR DEPOIS DO TESTE!
    # objeto, pessoa = "pudding_box", "person right"
    objeto, pessoa = msg_listener.listener()
    print("Pegando objeto: %s" % (objeto))
    print("Levando para: %s" % (pessoa))

    found_obj = False
    while not found_obj and (curr_t_a < len(tilt_angles)):
        angle = tilt_angles[curr_t_a]
        move_head_tilt(angle)
        result = request_classification(rgbd.get_image())
        if objeto in result:
            obj = result[objeto][0]
            obj_img_center = get_centroid((obj[0], obj[1]), (obj[2], obj[3]))
            x, y, z = imgcoords2world(obj_img_center[0], obj_img_center[1], rgbd)
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

    
    objt = SceneObject(position=(x, y, z),
                       obj_name="objeto",
                       base_frame="head_rgbd_sensor_rgb_frame")

    x, y, z = objt.get_position()

    wp_estante_cima = [x, 4.4, 0.92]
    wp_estante_baixo = [x, 4.4, 0.65]
    # wp_estante_cima = [2.21, 4.4, 0.92]
    # wp_estante_baixo = [2.21, 4.4, 0.65]

    if objeto not in ["master_chef_can"]:
        y -= 0.025 # Tool offset for small objects
    target = np.array([x, y, z])
    if z > 0.7:
        print("Usando waypoint cima!")
        starting_wp = wp_estante_cima
    else:
        print("Usando waypoint baixo!")
        starting_wp = wp_estante_baixo

    wp1 = target + np.array([0, -0.1, 0.05])
    wp2 = target + np.array([0, 0, 0.05])
    
    init_moveit_collisions()

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
    objt.destroy()

if __name__ == "__main__":
    main()