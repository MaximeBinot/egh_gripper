#!/usr/bin/env python
import sys
import rospy
from service import SchunkEGHServer

rospy.init_node('gripper_service', log_level=rospy.DEBUG)
server = SchunkEGHServer(test = len(sys.argv) > 1 and sys.argv[1] == "--test")
print("GripperServer running")
while not rospy.is_shutdown():
	continue