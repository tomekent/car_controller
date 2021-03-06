#!/usr/bin/python


import rospy
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, TwistStamped, PoseStamped
from sensor_msgs.msg import LaserScan
import math
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class pathPlanner():
	def __init__(self):
		self.update_rate = 60

		self.origin_x = 5.5
		self.origin_y = 0.5
		self.origin_yaw = math.pi/2
		self.scanData = []
		self.pioneer_nom_v = 0.5
		self.pioneer_travel_dist = 0.0

		self.tf_listener = tf.TransformListener()
		self.br = tf.TransformBroadcaster()
		self.brglobal = tf.TransformBroadcaster()
		self.current_pose_odom = PoseStamped()
		self.current_vel_odom = TwistStamped()

		self.init_pose = PoseStamped()

		self.init_flag = True
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
		if self.init_flag: # set the initial odometry location
			self.init_pose.header.frame_id  = 'odom_initialised'
			self.init_pose.pose.position = data.pose.pose.position
			self.init_pose.pose.orientation = data.pose.pose.orientation
			self.init_flag = False
			euler = euler_from_quaternion([self.init_pose.pose.orientation.x, self.init_pose.pose.orientation.y, self.init_pose.pose.orientation.z, self.init_pose.pose.orientation.w])
			yaw = euler[2]
			print("Initial pose: x: %2.2f, y: %2.2f, theta: %2.2f" %(self.init_pose.pose.position.x, self.init_pose.pose.position.y, yaw))

		q = quaternion_from_euler(self.origin_yaw, 0.,0.,'rzyx')
		self.br.sendTransform((self.origin_x, self.origin_y, 0.),
						 (q[0], q[1], q[2], q[3]),
		# br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
  						# (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
  						rospy.Time.now(),
  						"odom_initialised",
  						"global")

		self.brglobal.sendTransform((self.init_pose.pose.position.x, self.init_pose.pose.position.y, self.init_pose.pose.position.z),
						 (self.init_pose.pose.orientation.x, self.init_pose.pose.orientation.y, self.init_pose.pose.orientation.z, self.init_pose.pose.orientation.w),
		# br.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z),
  						# (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w),
  						rospy.Time.now(),
  						"odom",
  						"odom_initialised")
		self.current_pose_odom.header.frame_id  = 'odom'
		self.current_pose_odom.header.stamp = rospy.Time();
		self.current_pose_odom.pose.position = data.pose.pose.position
		self.current_pose_odom.pose.orientation = data.pose.pose.orientation
		self.current_vel_odom.twist.linear = data.twist.twist.linear
		self.current_vel_odom.twist.angular = data.twist.twist.angular
		
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
		current_vel = self.current_vel_odom.twist.linear.x
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

	def odom_to_global(self, odom):
		try:
			(self.trans, rot) = self.tf_listener.lookupTransform('/odom', '/global', rospy.Time(0))
		except:
			print "odom_to_global exception"
			pass

		object_pose = self.tf_listener.transformPose("/global", self.current_pose_odom)
		print object_pose



def set_cmd_ctrl(cmd, speed, steer):
	cmd.linear.x = speed
	cmd.linear.y = 0.
	cmd.linear.z = 0.
	cmd.angular.x = steer
	cmd.angular.y = 0.
	cmd.angular.z = 0.
	return cmd

def main():
	rospy.init_node('pioneer_path_planner')
	planner = pathPlanner()
	rospy.loginfo("path publisher up and running!")

	r = rospy.Rate(planner.update_rate)
	dt = 1./ planner.update_rate
	pioneer_max_dist = 6.0 # m
	rccar_max_dist = 5.0  # m
	
	rccar_nom_speed = .25 # m/s

	pioneer_cmd = Twist()
	rccar_cmd = Twist()

	while not rospy.is_shutdown():

		
		current_pose_global = PoseStamped()
		# print planner.current_pose_odom
		now = rospy.Time.now()
		# print (planner.current_pose_odom.header.stamp)
		# planner.tf_listener.waitForTransform("/global", "/odom", now, rospy.Duration(4.0))
		# print (planner.current_pose_odom.header.stamp)
		try:
			(trans, rot)=planner.tf_listener.lookupTransform('/odom', '/global',rospy.Time(0))
			current_pose_odom = planner.current_pose_odom
			current_pose_init = planner.init_pose
			current_pose_init = planner.tf_listener.transformPose('/odom_initialised', current_pose_odom)
			current_pose_global = planner.tf_listener.transformPose('/global', current_pose_odom)
			# print trans
			# print rot
			euler_odom = euler_from_quaternion([current_pose_odom.pose.orientation.x, current_pose_odom.pose.orientation.y, current_pose_odom.pose.orientation.z, current_pose_odom.pose.orientation.w])
			euler_init = euler_from_quaternion([current_pose_init.pose.orientation.x, current_pose_init.pose.orientation.y, current_pose_init.pose.orientation.z, current_pose_init.pose.orientation.w])
			euler_global = euler_from_quaternion([current_pose_global.pose.orientation.x, current_pose_global.pose.orientation.y, current_pose_global.pose.orientation.z, current_pose_global.pose.orientation.w])
			print("current odom pose: x: %2.2f, y: %2.2f, heading: %2.2f" %(current_pose_odom.pose.position.x, current_pose_odom.pose.position.y , euler_odom[2]))
			print("current init pose: x: %2.2f, y: %2.2f, heading: %2.2f" %(current_pose_init.pose.position.x, current_pose_init.pose.position.y , euler_init[2]))
			print("current global pose: x: %2.2f, y: %2.2f, heading: %2.2f" %(current_pose_global.pose.position.x, current_pose_global.pose.position.y , euler_global[2]))
		except:
			print "pass"
			pass

	# (trans,rot) = planner.tf_listener.lookupTransform("odom", "global", rospy.Time(0))
		# print trans
		# print rot
		# object_pose = planner.tf_listener.transformPose("odom", planner.current_pose_odom)
		# print object_pose

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
		# planner.pioneer_pub.publish(pioneer_cmd)
		
		# rospy.loginfo("publishing a command to rccar: {v: %2.2f, w: %2.2f}" %(rccar_cmd.linear.x, rccar_cmd.angular.x))
		# planner.rccar_pub.publish(rccar_cmd)

		# update the distances based on the speeds
		
		planner.pioneer_travel_dist += dt*planner.current_vel_odom.twist.linear.x
		# rospy.loginfo("pioneer has travelled: %2.4f meters" %planner.pioneer_travel_dist)		

		# planner.rccar_travel_dist += dt*rccar_speed
		# rospy.loginfo("rccar has travelled: %2.4f meters" %planner.rccar_travel_dist)	





		r.sleep()

	rospy.spin()

if __name__ == "__main__":
	main()