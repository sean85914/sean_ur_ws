#!/usr/bin/env python
import rospy
import numpy as np
import tf
from math import cos, sin

def forward_kinematic(pos):
	d1 = 0.089159
	a2 = -0.42500
	a3 = -0.39225
	d4 = 0.10915
	d5 = 0.09465
	d6 = 0.0823
	T = np.zeros(16)
	T = np.resize(T, (4,4))
	s1 = sin(pos[0])
	c1 = cos(pos[0])
	s2 = sin(pos[1])
	c2 = cos(pos[1])
	s3 = sin(pos[2])
	c3 = cos(pos[2])
	s5 = sin(pos[4])
	c5 = cos(pos[4])
	s6 = sin(pos[5])
	c6 = cos(pos[5])
	s234 = sin(pos[1]+pos[2]+pos[3])
	c234 = cos(pos[1]+pos[2]+pos[3])
	T[0][0] = ((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0
	T[0][1] = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0)
	T[0][2] = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0))
	T[0][3] = ((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3)
	T[1][0] = c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0
	T[1][1] = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0))
	T[1][2] = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0))
	T[1][3] = ((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3)
	T[2][0] = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0)
	T[2][1] = ((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6)
	T[2][2] = (s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0)
	T[2][3] = (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234)
	T[3][0] = 0.0
	T[3][1] = 0.0
	T[3][2] = 0.0
	T[3][3] = 1.0
	print "Homogeneous transform matrix: "
	print T
	roll, pitch, yaw = tf.transformations.euler_from_matrix(T)
	print "RPY: "
	print roll, pitch, yaw
	q = tf.transformations.quaternion_from_matrix(T)
	print q

if __name__ == "__main__":
	joint_1 = float(raw_input("joint 1:"))
	joint_2 = float(raw_input("joint 2:"))
	joint_3 = float(raw_input("joint 3:"))
	joint_4 = float(raw_input("joint 4:"))
	joint_5 = float(raw_input("joint 5:"))
	joint_6 = float(raw_input("joint 6:"))
	joint_list = [joint_1, joint_2,joint_3, joint_4, joint_5, joint_6]
forward_kinematic(joint_list)
