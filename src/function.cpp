#include "main.h"

 void resetMotors(){
	left_mtr1.tare_postion();
	left_mtr2.tare_postion();
	left_mtr3.tare_postion(); 
	left_mtr4.tare_postion();
	right_mtr1.tare_postion(); 
	right_mtr2.tare_postion(); 
	right_mtr3.tare_postion(); 
	right_mtr4.tare_postion(); 	
 }

//shows how motors will drive forward and turn
void Powerdrive(int powerforward, int powerturning){
left_mtr1 = powerforward + powerturning;
left_mtr2 = powerforward + powerturning;
left_mtr3 = powerforward + powerturning;
left_mtr4 = powerforward + powerturning;
right_mtr1 = powerforward - powerturning;
right_mtr2 = powerforward - powerturning;
right_mtr3 = powerforward - powerturning;
right_mtr4 = powerforward - powerturning;
}
/*
void resetSens()
left_mtr1
left_mtr2
left_mtr3
left_mtr4
right_mtr1
right_mtr2
right_mtr3
right_mtr4
*/

void thetime(int time, int direction)	{

	int starttime = millis();

	while(time > (millis() - starttime)){
		Powerdrive(127*direction ,0);

	}
	Powerdrive(0,0);

}

void timeturning(int time, int direction)	{

	int starttime = millis();

	while(time > (millis() - starttime)){
		Powerdrive(0,50*direction);

	}
	Powerdrive(0,0);

}
   
double distance(double inches){

	double internal, external, diameter, PI;

		//measures drive for PID
		internal = (double)300;
		external = (double)3/5;
		diameter = 3.25;
		PI = 3.141;

return ((inches*internal)/external/diameter*PI);

}
int speedlimit(int motorrunning){
	int cap = 60;
//limits speed to a max value
//if and else statement returns the speed in different possible scenarios
	if(abs(motorrunning)<= cap){
		return motorrunning;
	}
	else{
		if(motorrunning > cap){
			return cap;
	}
		else{
			return -cap;

	}
}

}


//varibles all control PID
//proportional multiplies to get closer to the target value
void PIDdrive(int inches, double kP, double kI, double kD){
	//resets known position of the robot
	left_mtr1.tare_position();

	int power, intergral, past_error;
	double derivative, error, target;
	target = distance(inches);
	//determines how much it can overshoot or undershoot by 2 encoder ticks of the target
	while	(fabs(target-left_mtr1.get_position()) > 2 )	{
		//updates past_error with error value
		past_error = error;
		//error is the difference between target and position of robot
		error = target - left_mtr1.get_position();
		//calculates derivative
		derivative = error - past_error;
		//10 is the number when we start using intergral
		//intergral is increases relative to proportional
		if(fabs(target - left_mtr1.get_position()) < 10 ){
			intergral += error;

			//derivative is the rate of change 
		}

	power = error*kP + intergral*kI + derivative*kD;
	Powerdrive(speedlimit(power), 0);

	}

	Powerdrive(0, 0);

}




void PIDturn(int target, double kP, double kI, double kD){
//controls turning of robot accurately
	
	int power, intergral, past_error;
	double derivative, error;
	//determines how much it can overshoot or undershoot by .2 encoder ticks of the target
	while	(fabs(target-imu.get_rotation()) > 0.2 )	{
		//updates past_error value with error
		past_error = error;
		//error is the difference between the target and the imu location
		error = target - imu.get_rotation();
		//calculates derivative
		derivative = error - past_error;
		//
		if(fabs(target - imu.get_rotation()) < 3 ){
			intergral += error;
		}

	power = error*kP + intergral*kI + derivative*kD;
	Powerdrive(0, speedlimit(power));
	}

	Powerdrive(0, 0);

}


//varibles all control PID
//proportional multiplies to get closer to the target value
void DriveStraight(int inches, double kP, double kI, double kD){
	//variables for driving
	int power, intergral, past_error;
	double derivative, error, target;
	//variables for turning
	int r_power, r_intergral, r_past_error;
	double r_derivative, r_error, r_target;
	double r_kP, r_kI, r_kD;
	r_kP = 0;
	r_kI = 0;
	r_kD = 0;

		//resets known position of the robot
	left_mtr1.tare_position();
	target = inches;
	//target = distance(inches);
	r_target = imu.get_rotation();
	//determines how much it can overshoot or undershoot by 2 encoder ticks of the target
	while	((fabs(target-left_mtr1.get_position()) > 2 )||(fabs(target-imu.get_rotation()) > 0.2))	{
		//updates past_error with error value
		past_error = error;
		//error is the difference between target and position of robot
		error = target - left_mtr1.get_position();
		//calculates derivative
		derivative = error - past_error;
		//10 is the number when we start using intergral
		//intergral is increases relative to proportional
		if(fabs(target - left_mtr1.get_position()) < 10 ){
			intergral += error;

			//derivative is the rate of change 
		}
	//determines how much it can overshoot or undershoot by .2 encoder ticks of the target

		//updates past_error value with error
		r_past_error = r_error;
		//error is the difference between the target and the imu location
		r_error = r_target - imu.get_rotation();
		//calculates derivative
		r_derivative = r_error - r_past_error;
		//
		if(fabs(r_target - imu.get_rotation()) < 3 ){
			r_intergral += r_error;
		}
	power = error*kP + intergral*kI + derivative*kD;
	r_power = r_error*r_kP + r_intergral*r_kI + r_derivative*r_kD;
	Powerdrive(speedlimit(power), r_power);

	}

	Powerdrive(0, 0);

}


// //reduces drift whtn driving robot
void brake(){
left_mtr1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
left_mtr2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
left_mtr3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
left_mtr4.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
right_mtr1.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
right_mtr2.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
right_mtr3.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
right_mtr4.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
}
//further reduces drift whtn driving robot
void coast(){
left_mtr1.set_brake_mode(E_MOTOR_BRAKE_COAST);
left_mtr2.set_brake_mode(E_MOTOR_BRAKE_COAST);
left_mtr3.set_brake_mode(E_MOTOR_BRAKE_COAST);
left_mtr4.set_brake_mode(E_MOTOR_BRAKE_COAST);
right_mtr1.set_brake_mode(E_MOTOR_BRAKE_COAST);
right_mtr2.set_brake_mode(E_MOTOR_BRAKE_COAST);
right_mtr3.set_brake_mode(E_MOTOR_BRAKE_COAST);
right_mtr4.set_brake_mode(E_MOTOR_BRAKE_COAST);
}
