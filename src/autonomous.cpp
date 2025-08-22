#include "autonomous.h"
#include "iostream"

static auto auton_timer = MyTimer();

/**
 * initialize the timer & gps and so on
*/
void auton_init(void) {
  // init
  my_gps.initGPS();
  // your initialization here
  auton_timer.reset();
}


/**
 * initialize pre user control and stop the timer
*/
void auton_pre_usercontrol(void) {
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("AutonTimer: %2.2fsec", auton_timer.getTime()/1000.0);
  // your pre user control here
}

/**
 * this auton program do nothing
*/
void auton_sb(void) {
  auton_init();
  auton_pre_usercontrol();
}

///////////////////////////////////////////////////////
// Your Autonomous here
// use void left/right_<logcal max score>_<pre load>(void){}
// don't forget to declare in the autonomous.h file
// with void left/right_<logcal max score>_<pre load>(void);
///////////////////////////////////////////////////////

/*
 * =====================================================================================
 *      Movement Function Reference
 * =====================================================================================
 *  Function                                  |   Type                |   Example
 * -------------------------------------------|-----------------------|-------------------------------------------------
 *  moveLeft(power)                           |   Basic motor control |   moveLeft(50) - 50% power
 *  my_base.timerForward(power, time)         |   Timed movement      |   my_base.timerForward(50, 1000) - 1 second
 *  my_base.posForwardRel(power, distance)    |   Distance-based      |   my_base.posForwardRel(50, 500) - 500mm
 *  my_gps.gpsPIDMove(x, y, dir, power, mode) |   GPS precision       |   my_gps.gpsPIDMove(600, 0, 1, 50, "PID")
 * =====================================================================================
 */

/**
 * this auton program do skill programing challenge
*/
void auton_skill(void){
  auton_init();
  // skill challenge auton code here
  auton_pre_usercontrol();
}

/**
 * GPS-based precise autonomous sequence
 * Replaces old encoder-based commands with GPS + sensor precision
 */
void auton_gps_precision(void) {
  auton_init();
  
  // Step 1: Begin with 1 preload ring in gripper, take ring in front and score both
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("Step 1: Collection");
  
  // Precise movement forward 320mm

  // gpsPIDMove(x_target, y_target, direction, max_power, mode)
  my_gps.gpsPIDMove(my_gps.getGpsX(), my_gps.getGpsY() + 320, 1, 50, "PID");
  
  Pinza.close();
  wait(200, msec);
  
  Pinza.open(); // Open gripper to release rings
  wait(500, msec); // Wait for rings to fall onto stake
  
  // Extended collection to ensure rings are secured (equivalent to old: recoleccion(100, 3);)
  recoleccion_precise(100, 3000);
  
  // Activate pneumatic collector
  RecolectorNeumatica.close();
  
  auton_pre_usercontrol();
}

/**
 * Precise collection mechanism function
 * @param power - power percentage for collection motors (0-100)
 * @param duration_ms - duration in milliseconds
 */
void recoleccion_precise(int power, int duration_ms) {
  Recolector.spin(directionType::fwd, power, velocityUnits::pct);
  wait(duration_ms, msec);
  Recolector.stop(brakeType::coast);
}

/**
 * this auton program will run a 'z' route to test the gps
 * 
 * gpsPIDMove(x_target, y_target, direction, max_power, mode="PID")
 * @param x_target - target X coordinate in mm (positive = right, negative = left)
 * @param y_target - target Y coordinate in mm (positive = forward, negative = backward)
 * @param direction - movement direction: 1 = forward, -1 = backward
 * @param max_power - maximum power percentage (0-100)
 * @param mode - "PID" for precise stop, "continue" for smooth transition, "heading" for heading correction
 * 
 * gpsAim(x_target, y_target, heading_offset)
 * @param x_target - X coordinate of target to aim at in mm
 * @param y_target - Y coordinate of target to aim at in mm  
 * @param heading_offset - additional angle offset in degrees (0 = point directly at target)
*/
void auton_gps_test(void) {
  auton_init();
  
  // Move to (600, 0) - 600mm to the right, forward direction, 60% power
  my_gps.gpsPIDMove(600, 0, 1, 60);
  
  // Move to (0, 600) - back to center X, 600mm forward, forward direction, 60% power
  my_gps.gpsPIDMove(0, 600, 1, 60);
  
  // Move to (600, 600) - 600mm right and 600mm forward, forward direction, 60% power
  my_gps.gpsPIDMove(600, 600, 1, 60);
  
  // Return to origin (0, 0) - back to starting position, forward direction, 60% power
  my_gps.gpsPIDMove(0, 0, 1, 60);
  
  // Aim at point (0, -1200) - point towards 1200mm behind starting position, no offset
  my_gps.gpsAim(0, -1200, 0);
  
  auton_pre_usercontrol();
}

/**
 * this function is used to trigger the autonomous manually without a field control with 'DOWN' key
*/
void runAuton(void) {
  // auton_skill();
  // auton_gps_test();
}