#include "sensors.h"

Sensors my_sensors;

void Sensors::updateSensors() {
  base_left_pos = deg2rad(MotorL1.position(deg)+MotorL2.position(deg)+MotorL3.position(deg)+MotorL4.position(deg)) / 4 * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4) / 2.0 * BASE_FORWARD_COEFF;
  base_right_pos = deg2rad(MotorR1.position(deg)+MotorR2.position(deg)+MotorR3.position(deg)+MotorR4.position(deg)) / 4 * CHASSIS_GEAR_RATIO * (WHEEL_DIAMETER * 25.4) / 2.0 * BASE_FORWARD_COEFF;
  base_left_vel = (MotorL1.velocity(pct) + MotorL2.velocity(pct) + MotorL3.velocity(pct) + MotorL4.velocity(pct)) / 4;
  base_right_vel = (MotorR1.velocity(pct) + MotorR2.velocity(pct) + MotorR3.velocity(pct) + MotorR4.velocity(pct)) / 4;
  base_heading = IMU.rotation() * 180 / IMU_HEADING_5;
}

float Sensors::getBaseLeftPos() {
  return base_left_pos;
}

float Sensors::getBaseRightPos() {
  return base_right_pos;
}

float Sensors::getBaseForwardPos() {
  return (base_left_pos + base_right_pos) * 2;
}

float Sensors::getBaseLeftVel() {
  return base_left_vel;
}

float Sensors::getBaseRightVel() {
  return base_right_vel;
}

/**
 * return base forward velocity from -100 to 100
*/
float Sensors::getBaseForwardVel() {
  return (base_left_vel + base_right_vel) / 2;
}

float Sensors::getBaseHeading() {
  return base_heading;
}

void Sensors::setBaseHeading(float _heading) {
  IMU.setRotation(_heading, degrees);
}

void Sensors::resetBaseLeftPos() {
  MotorL1.resetPosition();
  MotorL2.resetPosition();
  MotorL3.resetPosition();
  MotorL4.resetPosition();
}

void Sensors::resetBaseRightPos() {
  MotorR1.resetPosition();
  MotorR2.resetPosition();
  MotorR3.resetPosition();
  MotorR4.resetPosition();
}

void Sensors::resetBasePos() {
  resetBaseLeftPos();
  resetBaseRightPos();
}

void Sensors::resetBaseHeading() {
  IMU.resetRotation();
}

void autonSensors() {
  while(1) {
    my_sensors.updateSensors();
    this_thread::sleep_for(5);
  }
}
