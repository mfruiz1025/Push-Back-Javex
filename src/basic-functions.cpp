#include "vex.h"
#include "basic-functions.h"
#include "parameters.h"
#include <queue>
#include "math-tools.h"
#include "sensors.h"

static float left_pos_last = 0, right_pos_last = 0;

/**
 * powers all motors on left side of base with duty cycle _input%
 * @param _input ranges from -100 : 100
*/
void moveLeft(float _input) {
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  _input = _input * BASE_LEFT_POWER;
  Left.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}

void moveLeftVel(float _input) {
  // _input ranges from -100 : 100
  // powers all motors on left side of base with duty cycle _input%
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Left.spin(directionType::fwd, (int)_input, velocityUnits::pct);
}

/**
 * locks all motors on left side of base
*/
void lockLeft(void) {
  Left.stop(vex::brakeType::hold);
}

/**
 * unlocks all motors on left side of base
*/
void unlockLeft(void) {
  Left.stop(vex::brakeType::coast);
}

/**
 * powers all motors on right side of base with duty cycle _input%
 * @param _input ranges from -100 : 100
*/
void moveRight(float _input) {
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  _input = _input * BASE_RIGHT_POWER;
  Right.spin(directionType::fwd, (int)127 * _input, voltageUnits::mV);
}

/**
 * powers right motors on right side of base with duty cycle _input%
 * @param _input ranges from -100 : 100
*/
void moveRightVel(float _input) {
  if (fabs(_input) > 100) _input = sign(_input) * 100;
  Right.spin(directionType::fwd, (int) _input, velocityUnits::pct);
}

/**
 * locks all motors on right side of base
*/
void lockRight(void) {
  Right.stop(vex::brakeType::hold);
}

/**
 * unlocks all motors on right side of base
*/
void unlockRight(void) {
  Right.stop(vex::brakeType::coast);
}

/**
 * move forward with _input% power
 * @param _input ranges from -100 : 100
*/
void moveForward(float _input) {
  moveLeft(_input);
  moveRight(_input);
}

/**
 * rotate clockwise with _input% power
 * @param _input ranges from -100 : 100
*/
void moveClockwise(float _input) {
  moveLeft(_input);
  moveRight(-_input);
}

/**
 * lock the base
*/
void lockBase(void) {
  lockLeft();
  lockRight();
}

/**
 * unlock the base
*/
void unlockBase(void) {
  unlockLeft();
  unlockRight();
}

///////////////////////////////////////////////////////
// your special basic functions here
// we don't recommand you to write complex functions here
// you should call these functions in your advanced functions
///////////////////////////////////////////////////////

/**
 * return the position of left side of base (mm from last reset position) according to encoder value
 * @return return mm
*/
float getLeftPos() {
  return my_sensors.getBaseLeftPos() - left_pos_last;
}

/**
 * return the position of right side of base (mm from last reset position) according to encoder value
 * @return return mm
*/
float getRightPos() {
  return my_sensors.getBaseRightPos() - right_pos_last;
}

/**
 * return the vertical position of base (mm from last reset position) according to encoder value
 * @return return mm
*/
float getForwardPos() {
  return (getLeftPos() + getRightPos()) / 2;
}

/**
 * reset encoder on the left side of the base
*/
void resetLeftPos() {
  left_pos_last = my_sensors.getBaseLeftPos();
}

/**
 * reset encoder on the right side of the base
*/
void resetRightPos() {
  right_pos_last = my_sensors.getBaseRightPos();
}

/**
 * reset encoders on both side of the base
*/
void resetForwardPos() {
  resetLeftPos();
  resetRightPos();
}

float getCrtVel(){
  return (my_sensors.getBaseLeftVel() + my_sensors.getBaseRightVel()) / 2;
}

/**
 * return the heading angle of robot in deg (+360 after a full clockwise turn, -360 after a counter-clockwise turn) 
 * @return float with 1 decimal point
 */
float getHeading() {
  return my_sensors.getBaseHeading();
}

///////////////////////////////////////////////////////
// your special basic functions here
// which get robot current states/conditions
///////////////////////////////////////////////////////