#include <roundbot_control/MotorSim.h>

namespace roundbot_control{

MotorSim::MotorSim(double max_speed, double max_accel, double max_decel)
{
  current_speed_ = 0;
  max_speed_ = max_speed;
  max_accel_ = max_accel;
  max_decel_ = max_decel;
}

double MotorSim::iterate(double target_speed, double dt)
{
  double target_accel = (target_speed - current_speed_) / dt;

  if (target_accel > max_accel_){
    target_accel = max_accel_;
  }else if (target_accel < -max_decel_){
    target_accel = -max_decel_;
  }

  current_speed_ += (dt * target_accel);
  if (current_speed_ > max_speed_){
    current_speed_ = max_speed_;
  }else if (current_speed_ < -max_speed_){
    current_speed_ = -max_speed_;
  }

  return current_speed_;
}

}
