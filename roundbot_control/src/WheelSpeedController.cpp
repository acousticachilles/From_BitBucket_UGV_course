#include <roundbot_control/WheelSpeedController.h>

namespace roundbot_control
{

bool WheelSpeedController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
{
  sub_left_command_ = n.subscribe("left_command", 1, &WheelSpeedController::recvLeftCommand, this);
  sub_right_command_ = n.subscribe("right_command", 1, &WheelSpeedController::recvRightCommand, this);
  loadParams(n);
  left_target_speed_ = 0.0;
  right_target_speed_ = 0.0;

  left_joint_ = hw->getHandle(left_joint_name_);
  right_joint_ = hw->getHandle(right_joint_name_);
  return true;
}

void WheelSpeedController::loadParams(ros::NodeHandle& n)
{
  n.param("left_joint", left_joint_name_, std::string("left_wheel"));
  n.param("right_joint", right_joint_name_, std::string("right_wheel"));
  n.param("control_timeout", timeout_, 0.25);

  double max_speed, max_accel, max_decel;
  n.param("max_speed", max_speed, 10.0);
  n.param("max_accel", max_accel, 1.0);
  n.param("max_decel", max_decel, 1.0);

  left_motor_ = new MotorSim(max_speed, max_accel, max_decel);
  right_motor_ = new MotorSim(max_speed, max_accel, max_decel);
}

void WheelSpeedController::recvLeftCommand(const std_msgs::Float64::ConstPtr& msg)
{
  left_command_stamp_ = ros::Time::now();
  left_target_speed_ = msg->data;
}

void WheelSpeedController::recvRightCommand(const std_msgs::Float64::ConstPtr& msg)
{
  right_command_stamp_ = ros::Time::now();
  right_target_speed_ = msg->data;
}

void WheelSpeedController::update(const ros::Time& time, const ros::Duration& period)
{
  double left_actual;
  double right_actual;

  if (((time - left_command_stamp_).toSec() >= timeout_) || ((time - right_command_stamp_).toSec() >= timeout_)) {
    left_actual = left_motor_->iterate(0.0, period.toSec());
    right_actual = right_motor_->iterate(0.0, period.toSec());
  }else{
    left_actual = left_motor_->iterate(left_target_speed_, period.toSec());
    right_actual = right_motor_->iterate(right_target_speed_, period.toSec());
  }

  left_joint_.setCommand(left_actual);
  right_joint_.setCommand(right_actual);
}

void WheelSpeedController::starting(const ros::Time& time)
{

}

void WheelSpeedController::stopping(const ros::Time& time)
{

}

}
