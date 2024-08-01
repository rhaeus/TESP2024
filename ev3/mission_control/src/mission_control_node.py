import rospy

from mission_msg.msg import Rock, Rocks
from aStar2 import aStar, run
from ev3_control_msg.srv import WheelSpeed, WheelSpeedRequest
import math
from geometry_msgs.msg import PoseStamped


class MissonControl:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('mission_control_node', anonymous=True)

        self.Kp = 500
        self.Kd = 100
        self.Kp_turn = 500
        
        self.obstacle_positions = [[]]
        self.robot_pose = [0,0,0]
        

        self.rocks_sub = rospy.Subscriber('rocks', Rocks, self.rocks_callback)
        # self.aruco_sub = rospy.Subscriber('aruco', Rocks, self.aruco_callback)
        self.aruco_sub = rospy.Subscriber('fiducial_transforms', PoseStamped, self.aruco_callback)


        self.wheel_srv = rospy.ServiceProxy('ev3_control/wheel_speed', WheelSpeed)
        self.previousLinDist = 0
        

        # testing
        self.got_rocks = False
        self.got_aruco = False
        self.current_waypoint_index = -1

    def rocks_callback(self, rocks):
        print("received rock positions")
        self.got_rocks = True
        self.obstacle_positions = [[]]
        for rock in rocks:
            self.obstacle_positions.append([[rock.top_x, rock.top_y], [rock.bottom_x, rock.bottom_y]])

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z

    def aruco_callback(self, msg):
        print("received aruco positions")

        # call controller
        #TODO use msg data
         # Extract position
        x = msg.pose.position.x
        y = msg.pose.position.y

        self.robot_pose[0] = x
        self.robot_pose[1] = y
        
        # Extract orientation and convert to Euler angles
        orientation = msg.pose.orientation
        _, _, theta = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)
        
        rospy.loginfo("x: %f", x)
        rospy.loginfo("y: %f", y)
        rospy.loginfo("theta: %f", theta)

        # self.robot_pose = [0,0,0] # x,y, theta 
        currentPose = [[self.robot_pose[0]], [self.robot_pose[1]], theta]
        targetPose = [[self.path[self.current_waypoint_index][0]], [self.path[self.current_waypoint_index][1]]]
        self.controlSystem(currentPose, targetPose)


    def plan_path(self):
        # start_point = [0, 0] # xy meters
        # end_point = [2.0, 1.0] # xy meters
        start_point = [self.robot_pose[0], self.robot_pose[1]]
        end_point = [2.0, 1.0]

        x_Max = 2.06 # meters
        y_Max = 1.00 # meters

        # obs_pos = [((0.5, 0.4), (0.3, 0.3)),
        #             ((1.3, 0.7), (1.5, 0.5)),
        #             ((0.6, 1.0), (0.7, 0.7)),
        #             ((1.3, 0.25), (1.5, 0.2))]
    
        # astar = aStar()
        # run astar
        self.path = run(start_point, end_point, x_Max, y_Max, self.obstacle_positions)

        self.current_waypoint_index = 0
        


    # currentPose = [[0.1], [0.2]]
    # targetPose = [[0.5], [0.3]]
    def controlSystem(self, currentPose, targetPose):
        posErrorX = targetPose[0] - currentPose[0]
        posErrorY = targetPose[1] - currentPose[1]
        linDist = math.sqrt(posErrorX**2 + posErrorY**2)
        angCurrent = currentPose[2]

        angToTarget = math.atan2(posErrorY, posErrorX)
        angError = angToTarget - angCurrent

        derivative = linDist - previousLinDist
        previousLinDist = derivative



        speed = (self.Kp * linDist) + (self.Kd * derivative)
        # speed = -speed

        if angError < 0:
            angCurrentNew += 2 * math.pi
            angErrorNew = angToTarget - angCurrentNew
            if abs(angErrorNew) < abs(angError):
                angError = angErrorNew
        else:
            angCurrentNew -= 2 * math.pi
            angErrorNew = angToTarget - angCurrentNew
            if abs(angErrorNew) < abs(angError):
                angError = angErrorNew

        # left_motor.dc(speed + (Kp_turn * angError)) #speeds up the left wheel when angError is positive
        # right_motor.dc(speed - (Kp_turn * angError)) #slows the right wheel when angError is positive

        left_motor_speed = (speed + (self.Kp_turn * angError)) #speeds up the left wheel when angError is positive
        right_motor_speed = (speed - (self.Kp_turn * angError)) #slows the right wheel when angError is positive

        middle_motor_speed = abs((left_motor_speed + right_motor_speed) /2.0)


        print("left speed: ", left_motor_speed)
        print("right speed: ", right_motor_speed)
        print("middle motor speed: ", middle_motor_speed)

        try:
            print("send wheel commands")
            rospy.wait_for_service('ev3_control/wheel_speed')
            req = WheelSpeedRequest()
            req.left = left_motor_speed
            req.right = right_motor_speed
            response = self.wheel_srv(req)
            rospy.loginfo(f"Service response: {response.success}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        

    def spin(self):
        # NOTE: this is *very* hacky
        if self.mission_done:
            print("Mission done!")
        else:
            if self.current_waypoint_index == -1 and self.got_aruco and self.got_rocks:
                # generate the path
                self.plan_path()

            elif self.current_waypoint_index >=0:
                # follow the path
                # if robot position close to current way point, take next waypoint
                x_diff = self.robot_pose[0]-self.path[self.current_waypoint_index][0]
                y_diff = self.robot_pose[1]-self.path[self.current_waypoint_index][1]
                dist = math.sqrt(x_diff**2 + y_diff**2)
                if dist < 0.2:
                    self.current_waypoint_index = self.current_waypoint_index + 1
                    if self.current_waypoint_index >= len(self.path):
                        self.mission_done = True
                

            rospy.spin()

if __name__ == '__main__':


    mission_control = MissonControl()
    mission_control.spin()

    # publish wheel stuff
    

    # while(not(rospy.is_shutdown())):
    #     rospy.spin()