#include <sys/_stdint.h>
#ifndef SMORPHI_ODOMETRY_H
#define SMORPHI_ODOMETRY_H
#include "Arduino.h"
#include <ESP32Encoder.h>
#include <math.h>

namespace smorphi_odometry {
class MotorProperty_t {
public:
  MotorProperty_t(ESP32Encoder *encoder, double radius, double total_pulse)
    : encoder_(encoder),
      radius_(radius),
      totalPulse_(total_pulse) {
    lasttime = micros();
    rps = 0;
    mps = 0;
    last_pulse = encoder_->getCount();
  }

  void update() {
    unsigned long now = micros();
    // int64_t newPulse = encoder_->getCount();
    // encoder_->clearCount();
    double delta_time = (now - lasttime) / 1e6;
    lasttime = now;
    // rps = ((float)newPulse / totalPulse_) / (delta_time / 1e6);
    int64_t currentPulse = encoder_->getCount();
    int64_t deltaPulse = currentPulse - last_pulse;
    last_pulse = currentPulse;
    rps = (deltaPulse / totalPulse_) / delta_time;
    mps = rps * 2 * M_PI * radius_;
  }
  float getVelocityMPS() {
    return mps;
  }
  float getVelocityRPS() {
    return rps;
  }
  int64_t get_pulse(){
    return encoder_->getCount();
  }
private:
  ESP32Encoder *encoder_;
  double totalPulse_;
  float radius_;

  double rps;
  double mps;
  unsigned long lasttime;

  int64_t last_pulse;
};
class Odometry_t {
public:
  Odometry_t(MotorProperty_t *motor_fl, MotorProperty_t *motor_fr, MotorProperty_t *motor_rl, MotorProperty_t *motor_rr, double LX, double LY)
    : motor_fl_(motor_fl),
      motor_fr_(motor_fr),
      motor_rl_(motor_rl),
      motor_rr_(motor_rr),
      lx_(LX),
      ly_(LY) {
    vel_x_ = 0;
    vel_y_ = 0;
    vel_theta_ = 0;
    //O shape Forward kinematic
    k = lx_ + ly_;
    a = 0.17;

    f1 = (a + 2 * k) / (8 * k);
    f2 = (-a + 2 * k) / (8 * k);
    f3 = (-a - 2 * k) / (8 * k);
    f4 = (a - 2 * k) / (8 * k);
    inv4k = 1.0f / (4 * k);
  }

  void update() {
    static long last_time = micros();
    long now = micros();
    auto dt = (now - last_time) / 1e6;
    last_time = now;

    // //Default module
    vel_x_ = (motor_fl_->getVelocityMPS() + motor_rl_->getVelocityMPS() + motor_fr_->getVelocityMPS() + motor_rr_->getVelocityMPS()) / 4.0;
    vel_y_ = (motor_fl_->getVelocityMPS() - motor_rl_->getVelocityMPS() - motor_fr_->getVelocityMPS() + motor_rr_->getVelocityMPS()) / 4.0;
    vel_theta_ = (motor_fl_->getVelocityMPS() + motor_rl_->getVelocityMPS() - motor_fr_->getVelocityMPS() - motor_rr_->getVelocityMPS()) / (2 * (lx_ + ly_));

    // Serial.printf("fl: %.3f | rl: %.3f | rr: %.3f | rl : %.3f\n", motor_fl_->getVelocityMPS(), motor_fr_->getVelocityMPS(), motor_rr_->getVelocityMPS(), motor_rl_->getVelocityMPS());
    // vel_x_ = f1 * motor_fl_->getVelocityMPS() + f2 * motor_fr_->getVelocityMPS()
    //          + f1 * motor_rr_->getVelocityMPS() + f2 * motor_rl_->getVelocityMPS();

    // vel_y_ = f3 * motor_fl_->getVelocityMPS() + f1 * motor_fr_->getVelocityMPS()
    //          + f2 * motor_rr_->getVelocityMPS() + f4 * motor_rl_->getVelocityMPS();

    // vel_theta_ = -inv4k * motor_fl_->getVelocityMPS()
    //              + inv4k * motor_fr_->getVelocityMPS()
    //              - inv4k * motor_rl_->getVelocityMPS()
    //              + inv4k * motor_rr_->getVelocityMPS();


    auto vel_x_raw = vel_x_ * dt;
    auto vel_y_raw = vel_y_ * dt;
    auto vel_theta_raw = vel_theta_ * dt;

    // double radian = fmod(pos_theta_, M_PI * 2);
    // if (radian < 0)
    //   radian += M_PI * 2;
    // pos_theta_ = radian;

    auto vel_glob_x = (std::cos(pos_theta_) * vel_x_raw) - (std::sin(pos_theta_) * vel_y_raw);
    auto vel_glob_y = (std::cos(pos_theta_) * vel_y_raw) + (std::sin(pos_theta_) * vel_x_raw);
    pos_x_ += vel_glob_x;
    pos_y_ += vel_glob_y;
    // pos_x_ += vel_x_raw;
    // pos_y_ += vel_y_raw;
    // pos_theta_ += vel_theta_raw;

    // pos_theta_ = fmod(pos_theta_, 2*M_PI);
    // if (pos_theta_ < 0) pos_theta_ += 2*M_PI;
  }

  double vel_x() {
    return vel_x_;
  }
  double vel_y() {
    return vel_y_;
  }
  double vel_theta() {
    return vel_theta_;
  }

  double pos_x() {
    return pos_x_;
  }
  double pos_y() {
    return pos_y_;
  }
  double pos_theta() {
    return pos_theta_;
  }

  void setOrientation(double theta) {
    pos_theta_ = theta;
  }

  void setResetPose() {
    pos_theta_ = 0;
    pos_x_ = 0;
    pos_y_ = 0;
  }

private:
  MotorProperty_t *motor_fl_;
  MotorProperty_t *motor_fr_;
  MotorProperty_t *motor_rl_;
  MotorProperty_t *motor_rr_;

  double vel_x_ = 0;
  double vel_y_ = 0;
  double vel_theta_ = 0;

  double pos_x_ = 0;
  double pos_y_ = 0;
  double pos_theta_ = 0;

  double lx_ = 0;
  double ly_ = 0;


  float k, a, f1, f2, f3, f4, inv4k;
};
}

#endif