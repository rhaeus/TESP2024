#ifndef WHEEL_CONTROLLER_HPP
#define WHEEL_CONTROLLER_HPP

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ev3_control_msg/WheelSpeed.h"
#include <std_msgs/Float32.h>

#include "ev3dev.h"

//using namespace ev3dev;

class WheelController {
public:
  WheelController(ros::NodeHandle nh, ros::NodeHandle nh_priv)
      : nh_(nh), nh_priv_(nh_priv) {

    left_wheel_pub_ = nh_priv.advertise<std_msgs::Float32>("speed_left", 10);	 
    right_wheel_pub_ = nh_priv.advertise<std_msgs::Float32>("speed_right", 10);	 
    middle_wheel_pub_ = nh_priv.advertise<std_msgs::Float32>("speed_middle",10);	 

    stop_srv_ =
        nh_priv.advertiseService("stop", &WheelController::stopCallback, this);
    wheel_speed_srv_ =
        nh_priv.advertiseService("wheel_speed", &WheelController::wheelSpeedCallback, this);

    ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
    ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
    ev3dev::large_motor middle_motor(ev3dev::OUTPUT_D);
    if (!left_motor.connected()) {
      ROS_ERROR("Left motor not connected to port B");
    }
    if (!right_motor.connected()) {
      ROS_ERROR("Right motor not connected to port B");
    }
    if (!middle_motor.connected()) {
      ROS_ERROR("Middle motor not connected to port B");
    }
    ROS_INFO("Left Motor max speed: %d", left_motor.max_speed());
    ROS_INFO("Right Motor max speed: %d", right_motor.max_speed());
    ROS_INFO("Middle Motor max speed: %d", middle_motor.max_speed());
  }
  void publishWheelSpeeds() {
	  publishWheelSpeed(0);
	  publishWheelSpeed(1);
	  publishWheelSpeed(2);
  }

  void publishWheelSpeed(int wheel) {
	  std_msgs::Float32 msg;
	  switch(wheel){
		  case 0: // left
			  {
    			ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
		        msg.data = left_motor.speed();
			left_wheel_pub_.publish(msg);
			  }
			break;
		  case 1: // right
			{
    			ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
		        msg.data = right_motor.speed();
			right_wheel_pub_.publish(msg);
			}
			break;
		  case 2: // middle
			{
    			ev3dev::large_motor middle_motor(ev3dev::OUTPUT_D);
		        msg.data = middle_motor.speed();
			middle_wheel_pub_.publish(msg);
			}
			break;
	  }
  }

  bool stopCallback(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res) {
    ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
    ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
    ev3dev::large_motor middle_motor(ev3dev::OUTPUT_D);

    if (!left_motor.connected()) {
      ROS_ERROR("Left motor not connected to port B");
    }
    if (!right_motor.connected()) {
      ROS_ERROR("Right motor not connected to port B");
    }
    if (!middle_motor.connected()) {
      ROS_ERROR("Middle motor not connected to port B");
    }
    left_motor.stop();
    right_motor.stop();
    middle_motor.stop();

    res.success = true;
    res.message = "Stop wheels service has been called successfully.";
    ROS_INFO("Stop called");
    return true;
  }

  bool wheelSpeedCallback(ev3_control_msg::WheelSpeed::Request &req,
                       ev3_control_msg::WheelSpeed::Response &res) {
    ROS_INFO("WheelSpeed service was called.");

    ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
    ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
    ev3dev::large_motor middle_motor(ev3dev::OUTPUT_D);
    if (!left_motor.connected()) {
      ROS_ERROR("Left motor not connected to port B");
      res.success = false;
      res.message = "Left motor not connected to port B";
      return false;
    }
    if (!right_motor.connected()) {
      ROS_ERROR("Right motor not connected to port B");
      res.success = false;
      res.message = "Right motor not connected to port C";
      return false;
    }
    if (!middle_motor.connected()) {
      ROS_ERROR("Middle motor not connected to port B");
      res.success = false;
      res.message = "Middle motor not connected to port C";
      return false;
    }
    float left_speed = req.left;
    float right_speed = req.right;
    float middle_speed = req.middle;
    

    left_motor.set_speed_sp(left_speed);
    right_motor.set_speed_sp(right_speed);
    middle_motor.set_speed_sp(middle_speed);
    left_motor.run_forever();
    right_motor.run_forever();
    middle_motor.run_forever();

    res.success = true;
    res.message = "Wheel speed service has been called successfully.";
    return true;
  }




private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  ros::ServiceServer stop_srv_;
  ros::ServiceServer wheel_speed_srv_;

  ros::Publisher left_wheel_pub_;  
  ros::Publisher right_wheel_pub_;
  ros::Publisher middle_wheel_pub_; 
};

#endif
