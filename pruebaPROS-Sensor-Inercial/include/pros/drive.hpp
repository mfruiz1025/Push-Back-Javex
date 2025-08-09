#ifndef DRIVE_HPP
#define DRIVE_HPP

void setMotorPower(int left, int right);
void driveForward(int speed, int time_ms);
void turnLeft(int speed, int time_ms);
void turnRight(int speed, int time_ms);
void avanzarGrados(int grados);
void girarGrados(double grados);
void testGiro();
#endif
