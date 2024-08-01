#!/usr/bin/env python

import rospy
import math
from ev3_control_msg.srv import WheelSpeed, WheelSpeedRequest
from geometry_msgs import PoseStamped

class RobotDrive:
    def __init__(self):
        # Initialize the node
        rospy.init_node('ev3_drive_node', anonymous=True)

        # Read parameters from the parameter server
        self.self.kp = rospy.get_param('~self.kp', 500)
        self.self.kd = rospy.get_param('~self.kd', 100)
        self.self.kp_turn = rospy.get_param('~self.kp_turn', 500)

        # Create service client
        self.wheel_srv = rospy.ServiceProxy('ev3_control/wheel_speed', WheelSpeed)

        # Subscribe to topics
        self.robot_pose_sub = rospy.Subscriber('robot_pose', PoseStamped, self.robot_pose_callback)
        self.target_pose_sub = rospy.Subscriber('target_pose', PoseStamped, self.target_pose_callback)

        self.robot_pose[0] = 0
        self.robot_pose[1] = 0
        self.previousLinDist = 0

        # Define a rate for the loop
        self.rate = rospy.Rate(10)  # 10 Hz

    def robot_pose_callback(self, msg):
        rospy.loginfo(f"Received message from robot_pose: {msg.data}")
        self.robot_pose[0] = msg.pose.position.x
        self.robot_pose[1] = msg.pose.position.y
        self.wheel_speed_service()

    def target_pose_sub(self, msg):
        rospy.loginfo(f"Received message from target_pose: {msg.data}")
        self.target_pose[0] = msg.pose.position.x
        self.target_pose[1] = msg.pose.position.y
        self.wheel_speed_service()


    def wheel_speed_service(self):
        error_x = self.target_pose[0] - self.robot_pose[0]
        error_y = self.target_pose[1] - self.robot_pose[1]
        linDist = math.sqrt(error_x**2 + error_y**2)
        # TODO use this
        # angCurrent = self.robot_pose[2]
        angCurrent = 0 

        angToTarget = math.atan(error_x/error_y)
        angError = angToTarget - angCurrent

        derivative = linDist - self.previousLinDist
        self.previousLinDist = derivative

        # self.kp = 500
        # self.kd = 100
        # self.kp_turn = 500

        speed = (self.kp * linDist) + (self.kd * derivative)
        # speed = -speed

        left_motor_speed = (speed + (self.kp_turn * angError)) #speeds up the left wheel when angError is positive
        right_motor_speed = (speed - (self.kp_turn * angError)) #slows the right wheel when angError is positive

        try:
            rospy.wait_for_service('ev3_control/wheel_speed')
            # req = AddTwoIntsRequest()
            req = WheelSpeedRequest()
            req.left = left_motor_speed
            req.right = right_motor_speed
            response = self.wheel_srv(req)
            rospy.loginfo(f"Service response: {response.result}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    my_node = RobotDrive()
    my_node.spin()
