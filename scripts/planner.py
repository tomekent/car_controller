#!/usr/bin/python


import rospy
import std_msgs.msg

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class pathPublisher():
	def __init__(self):
		self.update_rate = 60

		self.scanData = []
		self.pioneer_travel_dist = 0.0
		self.rccar_travel_dist = 0.0
		self.rccar_speed = 1.0 # speed in m/s


		self.pioneer_pub = rospy.Publisher("/cmd_vel", Twist)
		self.rccar_pub = rospy.Publisher("/rcctrl", Twist, queue_size=1)
		self.sick_data = rospy.Subscriber("/scan", LaserScan, self.scan_data_callback)


	def scan_data_callback(self, scanData):
		self.scanData = scanData.ranges
		# print len(self.scanData)

		return True

	def get_nearest_reading(self):
		if len(self.scanData)>0:
			min_reading = min(self.scanData[30:-30])
		else:
			min_reading = 1e9
		return min_reading


def set_cmd_ctrl(cmd, speed, steer):
	cmd.linear.x = speed
	cmd.linear.y = 0.
	cmd.linear.z = 0.
	cmd.angular.x = steer
	cmd.angular.y = 0.
	cmd.angular.z = 0.
	return cmd

def main():
	path_publisher = pathPublisher()
	rospy.init_node('pioneer_path_publisher')
	rospy.loginfo("path publisher up and running!")

	r = rospy.Rate(path_publisher.update_rate)
	dt = 1/ path_publisher.update_rate
	pioneer_max_dist = 6.0 # m
	rccar_max_dist = 2 # m
	
	rccar_nom_speed = 1.0 # m/s

	pioneer_cmd = Twist()
	rccar_cmd = Twist()

	while not rospy.is_shutdown():


		#### Do the pioneer Logic ####
		nearest_reading = path_publisher.get_nearest_reading()

		if nearest_reading < 1:
			# STOP
			print("STOP! as min_reading is %2.2f" %nearest_reading)
			pioneer_speed = 0.0
			pioneer_steer = 0.0
			pioneer_cmd = set_cmd_ctrl(pioneer_cmd, pioneer_speed, pioneer_steer)
		else:
			# GO
			print("GO! as min_reading is %2.2f" %nearest_reading)
			pioneer_speed = 0.5
			pioneer_steer = 0.0
			pioneer_cmd = set_cmd_ctrl(pioneer_cmd, pioneer_speed, pioneer_steer)


		if path_publisher.pioneer_travel_dist < pioneer_max_dist:
			pioneer_speed = 0.0
			pioneer_steer = 0.0
			pioneer_cmd = set_cmd_ctrl(pioneer_cmd, pioneer_speed, pioneer_steer)			
			rospy.loginfo("pioneer has travelled too far (%2.2f meters) stopping." %path_publisher.pioneer_travel_dist)



		#### Do the rccar Logic ####

		rccar_speed = rccar_nom_speed
		rccar_steer = 0.0
		rccar_cmd = set_cmd_ctrl(rccar_cmd, rccar_speed, rccar_steer)

		if path_publisher.rccar_travel_dist < rccar_max_dist:
			rccar_speed = 0.0
			rccar_steer = 0.0
			rccar_cmd = set_cmd_ctrl(rccar_cmd, rccar_speed, rccar_steer)
			rospy.loginfo("rccar has travelled too far (%2.2f meters) stopping." %path_publisher.rccar_travel_dist)

			
		rospy.loginfo("publishing a command to Pioneer")
		path_publisher.pioneer_pub.publish(pioneer_cmd)
		
		rospy.loginfo("publishing a command to rccar")
		path_publisher.pioneer_pub.publish(rccar_cmd)

		# update the distances based on the speeds
		path_publisher.rccar_travel_dist += dt*rccar_speed
		path_publisher.pioneer_travel_dist += dt*pioneer_speed






		r.sleep()

	rospy.spin()

if __name__ == "__main__":
	main()