#ifndef WHEEL_CONTROLLER_HPP
#define WHEEL_CONTROLLER_HPP

#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "ev3_control_msg/WheelSpeed.h"

#include "ev3dev.h"

//using namespace ev3dev;

class WheelController {
public:
  WheelController(ros::NodeHandle nh, ros::NodeHandle nh_priv)
      : nh_(nh), nh_priv_(nh_priv) {
    //forwards_srv_ = nh.advertiseService(
    //    "forwards", &WheelController::forwardCallback, this);
    //backwards_srv_ = nh.advertiseService(
    //    "backwards", &WheelController::backwardCallback, this);

    //left_srv_ =
    //    nh.advertiseService("left", &WheelController::leftCallback, this);

    //right_srv_ =
    //    nh.advertiseService("right", &WheelController::rightCallback, this);
    stop_srv_ =
        nh_priv.advertiseService("stop", &WheelController::stopCallback, this);
    wheel_speed_srv_ =
        nh_priv.advertiseService("wheel_speed", &WheelController::wheelSpeedCallback, this);
  }

  bool stopCallback(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res) {
    // TODO send wheel command
    ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
    ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
    if (!left_motor.connected()) {
      ROS_ERROR("Left motor not connected to port B");
    }
    if (!left_motor.connected()) {
      ROS_ERROR("Left motor not connected to port B");
    }
    left_motor.stop();
    right_motor.stop();

    res.success = true;
    res.message = "Trigger service has been called successfully.";
    ROS_INFO("Stop called");
    return true;
  }

  bool wheelSpeedCallback(ev3_control_msg::WheelSpeed::Request &req,
                       ev3_control_msg::WheelSpeed::Response &res) {
    ROS_INFO("WheelSpeed service was called.");

    ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
    ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
    if (!left_motor.connected()) {
      ROS_ERROR("Left motor not connected to port B");
      res.success = false;
      res.message = "Left motor not connected to port B";
      return false;
    }
    if (!left_motor.connected()) {
      ROS_ERROR("Left motor not connected to port B");
      res.success = false;
      res.message = "Right motor not connected to port C";
      return false;
    }
    float left_speed = req.left;
    float right_speed = req.right;

    left_motor.set_speed_sp(left_speed);
    right_motor.set_speed_sp(right_speed);
    left_motor.run_forever();
    right_motor.run_forever();

    res.success = true;
    res.message = "Trigger service has been called successfully.";
    return true;
  }

  //bool forwardCallback(std_srvs::Trigger::Request &req,
  //                     std_srvs::Trigger::Response &res) {
  //  // TODO send wheel command
  //  ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
  //  ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
  //  if (!left_motor.connected()) {
  //    ROS_ERROR("Left motor not connected to port B");
  //  }
  //  if (!left_motor.connected()) {
  //    ROS_ERROR("Left motor not connected to port B");
  //  }
  //  left_motor.set_speed_sp(motor_speed);
  //  right_motor.set_speed_sp(motor_speed);
  //  left_motor.run_forever();
  //  right_motor.run_forever();

  //  res.success = true;
  //  res.message = "Trigger service has been called successfully.";
  //  ROS_INFO("Forward called");
  //  return true;
  //}

  //bool backwardCallback(std_srvs::Trigger::Request &req,
  //                      std_srvs::Trigger::Response &res) {
  //  // TODO send wheel command
  //  ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
  //  ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
  //  if (!left_motor.connected()) {
  //    ROS_ERROR("Left motor not connected to port B");
  //  }
  //  if (!left_motor.connected()) {
  //    ROS_ERROR("Left motor not connected to port B");
  //  }
  //  left_motor.set_speed_sp(-motor_speed);
  //  right_motor.set_speed_sp(-motor_speed);
  //  left_motor.run_forever();
  //  right_motor.run_forever();

  //  res.success = true;
  //  res.message = "Trigger service has been called successfully.";
  //  ROS_INFO("Backward called");
  //  return true;
  //}

  //bool leftCallback(std_srvs::Trigger::Request &req,
  //                  std_srvs::Trigger::Response &res) {
  //  // TODO send wheel command
  //  ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
  //  ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
  //  if (!left_motor.connected()) {
  //    ROS_ERROR("Left motor not connected to port B");
  //  }
  //  if (!left_motor.connected()) {
  //    ROS_ERROR("Left motor not connected to port B");
  //  }
  //  left_motor.set_speed_sp(-motor_speed);
  //  right_motor.set_speed_sp(motor_speed);
  //  left_motor.run_forever();
  //  right_motor.run_forever();

  //  res.success = true;
  //  res.message = "Trigger service has been called successfully.";
  //  ROS_INFO("TurnLeft called");
  //  return true;
  //}

  //bool rightCallback(std_srvs::Trigger::Request &req,
  //                   std_srvs::Trigger::Response &res) {
  //  // TODO send wheel command
  //  ev3dev::large_motor left_motor(ev3dev::OUTPUT_B);
  //  ev3dev::large_motor right_motor(ev3dev::OUTPUT_C);
  //  if (!left_motor.connected()) {
  //    ROS_ERROR("Left motor not connected to port B");
  //  }
  //  if (!left_motor.connected()) {
  //    ROS_ERROR("Left motor not connected to port B");
  //  }
  //  left_motor.set_speed_sp(motor_speed);
  //  right_motor.set_speed_sp(-motor_speed);
  //  left_motor.run_forever();
  //  right_motor.run_forever();

  //  res.success = true;
  //  res.message = "Trigger service has been called successfully.";
  //  ROS_INFO("TurnLeft called");
  //  return true;
  //}


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  //ros::ServiceServer forwards_srv_;
  //ros::ServiceServer backwards_srv_;
  //ros::ServiceServer left_srv_;
  //ros::ServiceServer right_srv_;

  ros::ServiceServer stop_srv_;
  ros::ServiceServer wheel_speed_srv_;

  // motors
  int motor_speed = 500;
};

#endif
