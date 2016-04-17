#include <roundbot_control/DiffController.h>

namespace roundbot_control
{

bool DiffController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n)
{
  sub_twist_ = n.subscribe("/roundbot/cmd_vel", 1, &DiffController::recvTwist, this);
  loadParams(n);
  left_target_speed_ = 0.0;
  right_target_speed_ = 0.0;

  left_joint_ = hw->getHandle(left_joint_name_);
  right_joint_ = hw->getHandle(right_joint_name_);
  return true;
}

void DiffController::loadParams(ros::NodeHandle& n)
{
  n.param("left_joint", left_joint_name_, std::string("left_wheel"));
  n.param("right_joint", right_joint_name_, std::string("right_wheel"));
  n.param("control_timeout", timeout_, 0.25);
  n.param("wheel_radius", wheel_radius_, 0.2);
  n.param("wheel_separation", wheel_separation_, 0.9);

  double max_speed, max_accel, max_decel;
  n.param("max_speed", max_speed, 10.0);
  n.param("max_accel", max_accel, 1.0);
  n.param("max_decel", max_decel, 1.0);

  left_motor_ = new MotorSim(max_speed, max_accel, max_decel);
  right_motor_ = new MotorSim(max_speed, max_accel, max_decel);
}

void DiffController::recvTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
  command_stamp_ = ros::Time::now();
  cmd_ = *msg;
}

void DiffController::update(const ros::Time& time, const ros::Duration& period)
{
  if ((time - command_stamp_).toSec() >= timeout_) {
    left_target_speed_ = 0.0;
    right_target_speed_ = 0.0;
  }else{
    left_target_speed_ = (cmd_.linear.x - 0.5 * wheel_separation_ * cmd_.angular.z) / wheel_radius_;
    right_target_speed_ = (cmd_.linear.x + 0.5 * wheel_separation_ * cmd_.angular.z) / wheel_radius_;
  }

  double left_actual = left_motor_->iterate(left_target_speed_, period.toSec());
  double right_actual = right_motor_->iterate(right_target_speed_, period.toSec());
  left_joint_.setCommand(left_actual);
  right_joint_.setCommand(right_actual);
}

void DiffController::starting(const ros::Time& time)
{

}

void DiffController::stopping(const ros::Time& time)
{

}

}
