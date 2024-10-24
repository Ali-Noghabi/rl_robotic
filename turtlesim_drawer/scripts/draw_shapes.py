#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class ShapeDrawer:
    def __init__(self):
        rospy.init_node('shape_drawer', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)  # 10 Hz

        # Parameters (can be loaded from ROS parameters or YAML)
        self.square_side = rospy.get_param('~square_side', 2.0)
        self.circle_radius = rospy.get_param('~circle_radius', 1.0)
        self.triangle_side = rospy.get_param('~triangle_side', 2.0)

    def update_pose(self, data):
        self.pose = data

    def move_forward(self, speed, distance):
        vel_msg = Twist()
        vel_msg.linear.x = speed
        vel_msg.angular.z = 0

        x0 = self.pose.x
        y0 = self.pose.y
        distance_moved = 0

        while distance_moved < distance and not rospy.is_shutdown():
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            distance_moved = math.sqrt((self.pose.x - x0)**2 + (self.pose.y - y0)**2)

        vel_msg.linear.x = 0
        self.velocity_publisher.publish(vel_msg)

    def rotate(self, angular_speed, angle_deg, clockwise=True):
        vel_msg = Twist()
        if clockwise:
            vel_msg.angular.z = -abs(angular_speed)
        else:
            vel_msg.angular.z = abs(angular_speed)
        
        angle_rad = math.radians(angle_deg)
        current_angle = 0
        start_angle = self.pose.theta

        while current_angle < angle_rad and not rospy.is_shutdown():
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            current_angle = abs(self.pose.theta - start_angle)

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def draw_square(self, side_length):
        rospy.loginfo("Drawing a square...")
        for _ in range(4):
            self.move_forward(2.0, side_length)
            self.rotate(45, 90, clockwise=True)
        rospy.loginfo("Square completed.")

    def draw_circle(self, radius, angular_speed):
        rospy.loginfo("Drawing a circle...")
        vel_msg = Twist()
        vel_msg.linear.x = 2.0
        vel_msg.angular.z = angular_speed

        circumference = 2 * math.pi * radius
        time_to_move = circumference / vel_msg.linear.x

        rospy.sleep(1)  # Wait for 1 second
        rospy.loginfo("Circle completed.")

    def draw_triangle(self, side_length):
        rospy.loginfo("Drawing a triangle...")
        for _ in range(3):
            self.move_forward(2.0, side_length)
            self.rotate(60, 120, clockwise=True)
        rospy.loginfo("Triangle completed.")

    def run(self):
        # Allow some time for connections to establish
        rospy.sleep(2)

        # Draw shapes
        self.draw_square(self.square_side)
        rospy.sleep(2)
        self.draw_circle(self.circle_radius, angular_speed=1.0)
        rospy.sleep(2)
        self.draw_triangle(self.triangle_side)
        rospy.loginfo("All shapes drawn.")

if __name__ == '__main__':
    try:
        drawer = ShapeDrawer()
        drawer.run()
    except rospy.ROSInterruptException:
        pass
