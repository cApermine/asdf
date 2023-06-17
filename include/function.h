#ifndef _FUNCTION_HPP_
#define _FUNCTION_HPP_

void Powerdrive(int powerforward, int powerturning);
void resetSens();
void thetime(int time, int direction);
void timeturning(int time, int direction);
double distance(double inches);
int speedlimit(int motorrunning);
void PIDdrive(int inches, double kP, double kI, double kD);
void PIDturn(int target, double kP, double kI, double kD);
void DriveStraight(int inches, double kP, double kI, double kD);
void brake();
void coast();

#endif