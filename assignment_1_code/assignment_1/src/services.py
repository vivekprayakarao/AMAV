#!/usr/bin/env python 

import rospy
from wheeled_robot_kinematics.srv import *
from wheeled_robot_kinematics.msg import *
from assignment_1.kinematics import *

def forward_func(req):
	#print "I am in service.py file > forward_func"

	#forward takes 3 tuples > pose p, action a & robot dimensions rd.
	#p & a are taken from req.
	#rd has to be pulled from parameter server.

	rd = ( rospy.get_param("/axle_length") , rospy.get_param("/wheel_radius") , rospy.get_param("/max_speed") )
	p = ( req.pose.x , req.pose.y , req.pose.theta )
	a = ( req.action.left_velocity , req.action.right_velocity , req.action.time )

	print "I am in service.py > forward_func"

	(x,y,theta) = forward( p , a , rd ) 

	print "Input:" , p, a, rd
	print "Output:" , x, y, theta
	print "-------------------------------------------------------"

	resp = DiffDriveFKResponse()
	resp.end_pose.x = x
	resp.end_pose.y = y
	resp.end_pose.theta = theta

	return resp

def inverse_func(req):

	#inverse function takes 3 tuples: p0, p1 & rd

	rd = ( rospy.get_param("/axle_length") , rospy.get_param("/wheel_radius") , rospy.get_param("/max_speed") )
	p0 = ( req.pose.x , req.pose.y , req.pose.theta )
	p1 = ( req.end_pose.x , req.end_pose.y , req.end_pose.theta )

	print "I am in service.py > inverse_func"

	resp = DiffDriveIKResponse()
	resp = inverse(p0,p1,rd)

	print "Input:" , p0, p1, rd
	print "Output:" , resp
	print "-------------------------------------------------------\n\n"

	return resp

def forward_inverse_server():
	rospy.init_node('kinematics')
	#print "I am in service.py file > forward_inverse_server"
	f = rospy.Service('kinematics/forward', DiffDriveFK, forward_func)
	i = rospy.Service('kinematics/inverse', DiffDriveIK, inverse_func)
	print "Ready"
	print "-------------------------------------------------------"
	print "-------------------------------------------------------"
	rospy.spin()

if __name__ == '__main__':
	#print "I am in service.py file"
	forward_inverse_server()
	#inverse_server()