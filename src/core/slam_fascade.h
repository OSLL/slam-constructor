#ifndef __SLAM_FASCADE_H
#define __SLAM_FASCADE_H

template<typename InputType>
class SlamFascade {
public:
  virtual void handle_sensor_data(InputType &) = 0;
};

#endif // guard
