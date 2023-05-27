#include "main.h"

	Motor left_mtr1(11,1);
	Motor left_mtr2(12,1);
	Motor left_mtr3(13,1);
	Motor left_mtr4(14,1);
	Motor right_mtr1(17);	
	Motor right_mtr2(18);	
	Motor right_mtr3(19);	
	Motor right_mtr4(20);	

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
   
double distance(int inches){

	double internal, external, diameter, PI;

		internal = 300;
		external = (double)3/5;
		diameter = 3.25;
		PI = 3.141;

return (inches*internal*external/diameter*PI);

}

void PIDdrive(int inches, double kP, double kI, double kD){

	int power, intergral, past_error;
	double derivative, error, target;
	// target = distance(inches);
	target = inches;
		while(abs(target-left_mtr1.tare_position()) > 2 ){

		past_error = error;
		error = target-left_mtr1.tare_position();
		derivative = error - past_error;

		if(abs(target - left_mtr1.tare_position()) < 10 ){
		intergral += error;
	}
	power = error*kP + intergral*kI + derivative*kD;
	Powerdrive(power, 0);

	}

	Powerdrive(0, 0);

}




/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	Controller master(pros::E_CONTROLLER_MASTER);

	PIDdrive(1000, 0.5, 0, 0);



	while (true) {
		int yaxis = master.get_analog(ANALOG_LEFT_Y);
		int xaxis = master.get_analog(ANALOG_LEFT_X);

		Powerdrive(yaxis, xaxis);



		pros::delay(20);
	}
}
