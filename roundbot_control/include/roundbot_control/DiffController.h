
#ifndef DIFFCONTROLLER_H_
#define DIFFCONTROLLER_H_

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Twist.h>
#include <roundbot_control/MotorSim.h>

namespace roundbot_control
{

class DiffController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
{
public:
  bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle &n);
  void update(const ros::Time& time, const ros::Duration& period);
  void starting(const ros::Time& time);
  void stopping(const ros::Time& time);
private:
  void recvTwist(const geometry_msgs::Twist::ConstPtr& msg);
  void loadParams(ros::NodeHandle& n);

  ros::Subscriber sub_twist_;

  hardware_interface::JointHandle left_joint_;
  hardware_interface::JointHandle right_joint_;

  MotorSim* left_motor_;
  MotorSim* right_motor_;
  geometry_msgs::Twist cmd_;
  double left_target_speed_;
  double right_target_speed_;
  ros::Time left_command_stamp_;
  ros::Time command_stamp_;

  // Parameters
  std::string left_joint_name_;
  std::string right_joint_name_;
  double timeout_;
  double wheel_radius_;
  double wheel_separation_;
};

PLUGINLIB_EXPORT_CLASS(roundbot_control::DiffController, controller_interface::ControllerBase);

}



#endif /* DIFFCONTROLLER_H_ */
