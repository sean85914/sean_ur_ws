#!/usr/bin/env python

# Purpose: Know the transform from base_link to camera_link

## TF we have:
#  base_link -> tag
#  camera    -> tag (callback)
## TF we want to
#  base_link -> camera = (base_link -> tag) * (camera -> tag)^-1

import rospy
import tf
import numpy as np
from apriltags_ros import AprilTagDetectionArray

tf_baselink2tag = np.array([[],[],[],[]]) # TBD

def cb_tag(msg):
	# Make sure there is a tag
	if(msg.detections.size() == 0):
		return
	listener = tf.TransformListener()
	try:
		listener.waitForTransform("camera_link", "tag_??", rospy.Time(0), rospy.Duration(3.0)) # tag id TBD
		(trans, rot) = listener.lookupTransform("camera_link", "tag_??", rospy.Time(0))
	except (tf.LookuoException, tf.ConnectivityException, tf.ExtrapolationException):
		print "TF Exception"
		return 
	tf_cameralink2tag = tf.transformations.quaternion_matrix(rot)
	tf_cameralink2tag[0][3] = trans[0]
	tf_cameralink2tag[1][3] = trans[1]
	tf_cameralink2tag[1][3] = trans[2]
	tf_tag2cameralink = tf.transformations.inverse_matrix(tf_cameralink2tag)
	tf_baselink2cameralink = np.matmul(tf_baselink2tag, tf.tag2cameralink)
	# print matrix
	print tf_baselink2cameralink
	# print quaternion
	print tf.transformations.quaternion_from_matrix(tf_baselink2cameralink)
	
if __name__ == "__main__":
	rospy.init_node("calibration_node")
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, cb_tag)
	rospy.spin()
