#include "main.h"
pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Motor Left1 (7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Left2 (8, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Left3 (9, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Left4 (10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Right1 (6, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Right2 (4, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Right3 (12, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Right4 (3, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor shootR (18, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor shootL (19, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor roll (11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor knife (20, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor slip (2, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIDigitalIn limit ('E');
pros::ADIAnalogIn  slip_potentiometer('G');
pros::ADIPotentiometer  knife_potentiometer('H');
pros::ADIDigitalIn  line('A');
pros::ADIDigitalIn  infrared('F');

int autonomous_para=0;
int starting_speed=5;
float precision=0.2 ;
float VelocityMax=95;
double accel=0.01;
int count=0;
int empty=1;
int knife_para=1;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
	Left1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
Right1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
Left2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Right2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Left3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Right3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Left4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
Right4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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
