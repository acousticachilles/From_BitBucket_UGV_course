
#ifndef MOTORSIM_H_
#define MOTORSIM_H_

namespace roundbot_control{

class MotorSim{
public:
  MotorSim(double max_speed, double max_accel, double max_decel);

  double iterate(double target_speed, double dt);
private:

  double current_speed_;

  double max_speed_;
  double max_accel_;
  double max_decel_;
};

}



#endif /* MOTORSIM_H_ */
