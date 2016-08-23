#!/usr/bin/python


import rospy
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math


class pathPlanner():
	def __init__(self):
		self.update_rate = 60

		self.scanData = []
		self.pioneer_nom_v = 0.5
		self.pioneer_travel_dist = 0.0

		self.current_vel = Twist()

		self.obs_buffer = 0.5 # avoidance distance m.
		self.rccar_travel_dist = 0.0
		self.rccar_speed = 1.0 # speed in m/s


		self.pioneer_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		self.rccar_pub = rospy.Publisher("/rcctrl", Twist, queue_size=1)
		self.sick_data = rospy.Subscriber("/scan", LaserScan, self.scan_data_callback)
		self.odometry_data = rospy.Subscriber("/pose", Odometry, self.odometry_callback)

	def scan_data_callback(self, scanData):
		self.scanData = scanData.ranges
		# print len(self.scanData)

		return True

	def odometry_callback(self, data):
		self.current_pose.position = data.pose.pose.position
		self.current_pose.orientation = data.pose.pose.orientation
		self.current_vel.linear = data.twist.twist.linear
		self.current_vel.angular = data.twist.twist.angular
		# print self.current_vel.linear.x

	def get_nearest_reading(self):
		if len(self.scanData)>0:
			min_reading = min(self.scanData[30:-30])
		else:
			min_reading = 1e9
		return min_reading

	def speed_controller(self, dist):
		dt = 1./self.update_rate
		ob_dist_max = 1.
		ob_dist_min = self.obs_buffer

		nom_vel = self.pioneer_nom_v
		current_vel = self.current_vel.linear.x
		acc = 4. #m/s/s
		decc = -2. #m/s/s
		max_decc = -4.

		if dist >=  ob_dist_max:
			if current_vel < nom_vel:
				new_acc = acc
				new_vel = min(current_vel + dt * new_acc, nom_vel)
				rospy.loginfo("distance OK (%2.2f), too slow (%2.2f). Speeding up to %2.2f (a:%2.2f)" %(dist, current_vel, new_vel, new_acc))

			elif current_vel > nom_vel:
				new_acc = decc
				new_vel = max(current_vel + dt * new_acc, nom_vel)
				rospy.loginfo("distance OK (%2.2f), too fast (%2.2f). Slowing down to %2.2f (a:%2.2f)" %(dist, current_vel, new_vel, new_acc))
			else:
				new_vel = current_vel
				rospy.loginfo("distance OK (%2.2f), Speed OK (%2.2f)" %(dist, new_vel))


		else:
			dist_remaining = max(dist - ob_dist_min, 0.)
			emergency_dist = (- current_vel * current_vel) / (2 * max_decc)

			if dist_remaining <= emergency_dist:
				new_vel =  0.0
				rospy.loginfo("EMERGENCY BRAKE, too close at %2.2fm to buffer (%2.2fm) slowing down to %2.2f" %(dist_remaining, ob_dist_min,new_vel))
			else:
				min_acc = (- current_vel*current_vel )/ (2 * dist_remaining)
				new_acc = min_acc
				new_vel = min(current_vel + dt * new_acc, 0.0)
				rospy.loginfo("distance BAD, getting close at %2.2fm (to buffer of %2.2fm) slowing down to %2.2f (a:%2.2f)" %(dist_remaining, ob_dist_min,new_vel, new_acc))

		return new_vel


def set_cmd_ctrl(cmd, speed, steer):
	cmd.linear.x = speed
	cmd.linear.y = 0.
	cmd.linear.z = 0.
	cmd.angular.x = steer
	cmd.angular.y = 0.
	cmd.angular.z = 0.
	return cmd

def main():
	planner = pathPlanner()
	rospy.init_node('pioneer_path_planner')
	rospy.loginfo("path publisher up and running!")

	r = rospy.Rate(planner.update_rate)
	dt = 1./ planner.update_rate
	pioneer_max_dist = 6.0 # m
	rccar_max_dist = 5.0  # m
	
	rccar_nom_speed = .25 # m/s

	pioneer_cmd = Twist()
	rccar_cmd = Twist()

	while not rospy.is_shutdown():


		#### Do the pioneer Logic ####
		nearest_reading = planner.get_nearest_reading()

		pioneer_steer = 0.0
		pioneer_speed = planner.speed_controller(nearest_reading)
		pioneer_cmd = set_cmd_ctrl(pioneer_cmd, pioneer_speed, pioneer_steer)
		# if nearest_reading < 1:
		# 	# STOP
		# 	print("STOP! as min_reading is %2.2f" %nearest_reading)
		# 	pioneer_speed = 0.0
		# 	pioneer_steer = 0.0
		# 	pioneer_cmd = set_cmd_ctrl(pioneer_cmd, pioneer_speed, pioneer_steer)
		# else:
		# 	# GO
		# 	print("GO! as min_reading is %2.2f" %nearest_reading)
		# 	pioneer_speed = 0.5
		# 	pioneer_steer = 0.0
		# 	pioneer_cmd = set_cmd_ctrl(pioneer_cmd, pioneer_speed, pioneer_steer)


		# if planner.pioneer_travel_dist > pioneer_max_dist:
		# 	pioneer_speed = 0.0
		# 	pioneer_steer = 0.0
		# 	pioneer_cmd = set_cmd_ctrl(pioneer_cmd, pioneer_speed, pioneer_steer)			
		# 	rospy.loginfo("pioneer has travelled too far (%2.2f meters) stopping." %planner.pioneer_travel_dist)



		#### Do the rccar Logic ####

		# rccar_speed = rccar_nom_speed
		# rccar_steer = 0.0
		# rccar_cmd = set_cmd_ctrl(rccar_cmd, rccar_speed, rccar_steer)

		# if planner.rccar_travel_dist > rccar_max_dist:
		# 	rccar_speed = 0.0
		# 	rccar_steer = 0.0
		# 	rccar_cmd = set_cmd_ctrl(rccar_cmd, rccar_speed, rccar_steer)
		# 	rospy.loginfo("rccar has travelled too far (%2.2f meters) stopping." %planner.rccar_travel_dist)

			
		# rospy.loginfo("publishing a command to Pioneer")
		planner.pioneer_pub.publish(pioneer_cmd)
		
		# rospy.loginfo("publishing a command to rccar: {v: %2.2f, w: %2.2f}" %(rccar_cmd.linear.x, rccar_cmd.angular.x))
		# planner.rccar_pub.publish(rccar_cmd)

		# update the distances based on the speeds
		
		planner.pioneer_travel_dist += dt*planner.current_vel.linear.x
		# rospy.loginfo("pioneer has travelled: %2.4f meters" %planner.pioneer_travel_dist)		

		# planner.rccar_travel_dist += dt*rccar_speed
		# rospy.loginfo("rccar has travelled: %2.4f meters" %planner.rccar_travel_dist)	





		r.sleep()

	rospy.spin()

if __name__ == "__main__":
	main()