#!/usr/bin/env python
import roslib
import rospy
import tf
import math
from std_msgs.msg import UInt16MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class EKFOdometry:
    def __init__(self):
        self.odom_data_pub = rospy.Publisher('odom', Odometry, queue_size=100)
        self.odom_data_pub_quat = rospy.Publisher('odom_data_quat', Odometry, queue_size=100)
        self.odom = Odometry()
        self.odom_old = Odometry()

        # Initial Pose
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_theta = 0.00000000001
        self.pi = 3.141592

        # Robot Physical constant
        self.w = 0.38
        self.rad_wh = 0.16
        self.enc_rev_pls = 1000
        self.gear_ratio = 25

        # Distance both wheels have traveled
        self.distance_left = 0.0
        self.distance_right = 0.0

        # Flag to see if the initial pose has been received
        self.initial_pose_received = False
        # Flag to start count
        self.initial_l_count = False
        self.initial_r_count = False

        # Initialize ROS node
        rospy.init_node('ekf_odom_pub')

        # Set the data fields of the odometry message
        self.odom.header.frame_id = 'odom'
        self.odom.pose.pose.position.z = 0
        self.odom.pose.pose.orientation.x = 0
        self.odom.pose.pose.orientation.y = 0
        self.odom.twist.twist.linear.x = 0
        self.odom.twist.twist.linear.y = 0
        self.odom.twist.twist.linear.z = 0
        self.odom.twist.twist.angular.x = 0
        self.odom.twist.twist.angular.y = 0
        self.odom.twist.twist.angular.z = 0
        self.odom_old.pose.pose.position.x = self.initial_x
        self.odom_old.pose.pose.position.y = self.initial_y
        self.odom_old.pose.pose.orientation.z = self.initial_theta

        # Initialize odom tf
        self.br = tf.TransformBroadcaster()
        self.odom_trans = TransformStamped()
        self.odom = Odometry()

        # Subscribe to ROS topics
        rospy.Subscriber('modbus/regs_read', UInt16MultiArray, self.calc_leftandright, queue_size=100)
        rospy.Subscriber('initial_2d', PoseStamped, self.set_initial_2d, queue_size=1)

        # Rate for the ROS node
        self.rate = rospy.Rate(10)

        self.TICKS_PER_METER = self.ticks_per_meter()

        # Initialize last counts
        self.initial_last_count_l = 0
        self.initial_last_count_r = 0

    def ticks_per_meter(self):
        TPM = (self.gear_ratio * self.enc_rev_pls * 1000) / (2 * self.pi * self.rad_wh)
        return TPM

    # Callback to set the initial 2D pose from RViz or a manual pose publisher
    def set_initial_2d(self, rviz_click):
        self.odom_old.pose.pose.position.x = rviz_click.pose.position.x
        self.odom_old.pose.pose.position.y = rviz_click.pose.position.y
        self.odom_old.pose.pose.orientation.z = rviz_click.pose.orientation.z
        self.initial_pose_received = True

    # Calculate the distance that the left and right wheel has traveled since the last cycle
    def calc_leftandright(self, lt_rt_count):
        if not self.initial_l_count:
            self.distance_left = 0.0
            self.initial_l_count = True
        else:
            if lt_rt_count.data[0] != 0 and self.initial_last_count_l != 0:
                left_ticks = lt_rt_count.data[0] - self.initial_last_count_l
                self.distance_left = left_ticks / self.TICKS_PER_METER
        if not self.initial_r_count:
            self.distance_right = 0.0
            self.initial_r_count = True
        else:
            if lt_rt_count.data[1] != 0 and self.initial_last_count_r != 0:
                right_ticks = lt_rt_count.data[1] - self.initial_last_count_r
                self.distance_right = right_ticks / self.TICKS_PER_METER   
        rospy.loginfo("left and right distance LEFT = %d  RIGHT = %d", lt_rt_count.data[0], lt_rt_count.data[1])
        self.initial_last_count_l = lt_rt_count.data[0]
        self.initial_last_count_r = lt_rt_count.data[1]
        
        

    # Publish a nav_msgs::Odometry message in quaternion format
    def publish_quat(self):
        odom_q = tf.transformations.quaternion_from_euler(0, 0, self.odom.pose.pose.orientation.z)
        odom_q = Quaternion(*odom_q)
        self.odom_trans.header.stamp = self.odom.header.stamp
        self.odom_trans.transform.translation.x = self.odom.pose.pose.position.x
        self.odom_trans.transform.translation.y = self.odom.pose.pose.position.y
        self.odom_trans.transform.translation.z = 0.0
        self.odom_trans.transform.rotation = odom_q
        quat_odom = Odometry()
        quat_odom.header.stamp = self.odom.header.stamp
        quat_odom.header.frame_id = 'odom'
        quat_odom.child_frame_id = 'base_link'
        quat_odom.pose.pose.position.x = self.odom.pose.pose.position.x
        quat_odom.pose.pose.position.y = self.odom.pose.pose.position.y
        quat_odom.pose.pose.position.z = self.odom.pose.pose.position.z
        quat_odom.pose.pose.orientation = odom_q
        quat_odom.twist.twist.linear.x = self.odom.twist.twist.linear.x
        quat_odom.twist.twist.linear.y = self.odom.twist.twist.linear.y
        quat_odom.twist.twist.linear.z = self.odom.twist.twist.linear.z
        quat_odom.twist.twist.angular.x = self.odom.twist.twist.angular.x
        quat_odom.twist.twist.angular.y = self.odom.twist.twist.angular.y
        quat_odom.twist.twist.angular.z = self.odom.twist.twist.angular.z
        for i in range(36):
            if i == 0 or i == 7 or i == 14:
                quat_odom.pose.covariance[i] = 0.01
            elif i == 21 or i == 28 or i == 35:
                quat_odom.pose.covariance[i] += 0.1
            else:
                quat_odom.pose.covariance[i] = 0
        #publish transforms
        self.br.sendTransform((self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, 0.0), tf.transformations.quaternion_from_euler(0.0, 0.0,self.odom.pose.pose.orientation.z), self.odom.header.stamp, "base_link", "odom")
        self.odom_data_pub_quat.publish(quat_odom)

    # Update odometry information
    def update_odom(self):
        # Calculate the average distance
        cycle_distance = (self.distance_right + self.distance_left) / 2

        # Calculate the number of radians the robot has turned since the last cycle
        cycle_angle = math.asin((self.distance_right - self.distance_left) / self.w)

        # Average angle during the last cycle
        avg_angle = cycle_angle / 2 + self.odom_old.pose.pose.orientation.z

        if avg_angle > self.pi:
            avg_angle -= 2 * self.pi
        elif avg_angle < -self.pi:
            avg_angle += 2 * self.pi

        # Calculate the new pose (x, y, and theta)
        self.odom.pose.pose.position.x = self.odom_old.pose.pose.position.x + math.cos(avg_angle) * cycle_distance
        self.odom.pose.pose.position.y = self.odom_old.pose.pose.position.y + math.sin(avg_angle) * cycle_distance
        self.odom.pose.pose.orientation.z = cycle_angle + self.odom_old.pose.pose.orientation.z

        # Prevent lockup from a single bad cycle
        if math.isnan(self.odom.pose.pose.position.x) or math.isnan(self.odom.pose.pose.position.y) or math.isnan(
                self.odom.pose.pose.position.z):
            self.odom.pose.pose.position.x = self.odom_old.pose.pose.position.x
            self.odom.pose.pose.position.y = self.odom_old.pose.pose.position.y
            self.odom.pose.pose.orientation.z = self.odom_old.pose.pose.orientation.z

        # Make sure theta stays in the correct range
        if self.odom.pose.pose.orientation.z > self.pi:
            self.odom.pose.pose.orientation.z -= 2 * self.pi
        elif self.odom.pose.pose.orientation.z < -self.pi:
            self.odom.pose.pose.orientation.z += 2 * self.pi

        # Compute the velocity
        self.odom.header.stamp = rospy.Time.now()
        time_diff = self.odom.header.stamp.to_sec() - self.odom_old.header.stamp.to_sec()
        self.odom.twist.twist.linear.x = cycle_distance / time_diff
        self.odom.twist.twist.angular.z = cycle_angle / time_diff

        # Save the pose data for the next cycle
        self.odom_old.pose.pose.position.x = self.odom.pose.pose.position.x
        self.odom_old.pose.pose.position.y = self.odom.pose.pose.position.y
        self.odom_old.pose.pose.orientation.z = self.odom.pose.pose.orientation.z
        self.odom_old.header.stamp = self.odom.header.stamp

        # odom_data_pub.publish(odomNew);
        self.odom_data_pub.publish(self.odom)

    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.initial_pose_received:
                self.update_odom()
                self.publish_quat()
            rate.sleep()

if __name__ == "__main__":
    ekf_odom_pub = EKFOdometry()
    ekf_odom_pub.run()

