#!/usr/bin/env python
import rospy
from std_msgs.msg import String

msg = None
   
def listener():
    msg = rospy.wait_for_message("message", String, timeout=5)
    utterance = msg.data    
    obj, person = [x.strip() for x in utterance.split(' to ')]
    return obj, person
   
if __name__ == '__main__':
    listener()
