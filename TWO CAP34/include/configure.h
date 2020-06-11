#include "api.h"
#include "math.h"

//configure:
extern pros::Controller master;
extern pros::Motor Left1;
extern pros::Motor Left2;
extern pros::Motor Left3;
extern pros::Motor Left4;
extern pros::Motor Right1;
extern pros::Motor Right2;
extern pros::Motor Right3;
extern pros::Motor Right4;
extern pros::Motor shootR;
extern pros::Motor shootL;
extern pros::Motor roll;
extern pros::Motor knife;
extern pros::Motor slip;
extern pros::ADIDigitalIn limit;
extern pros::ADIAnalogIn  slip_potentiometer;
extern pros::ADIPotentiometer  knife_potentiometer;
extern pros::ADIDigitalIn  line;
extern pros::ADIDigitalIn  infrared;

//vari:
extern int autonomous_para;
extern int starting_speed;
extern float precision;
extern float VelocityMax;
extern double accel;
extern int count;
extern int empty;
extern int knife_para;

#define A master.get_digital(pros::E_CONTROLLER_DIGITAL_A)
#define B master.get_digital(pros::E_CONTROLLER_DIGITAL_B)
#define X master.get_digital(pros::E_CONTROLLER_DIGITAL_X)
#define Y master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)
#define UP master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)
#define DOWN master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)
#define LEFT master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)
#define RIGHT master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)
#define L1 master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)
#define L2 master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)
#define R1 master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)
#define R2 master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)
#define axis_x master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)
#define axis_y master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)
#define Left_encoder Left1.get_position()
#define Right_encoder Right1.get_position()
#define represent Left3
#define turnRatio 0.83
#define inputS 1
#define inputM 2
#define inputB 5
#define outputS 1
#define outputM 2
#define outputB 5
#define outputZO 0
#define inputZO 0
#define delay_time 130
#define outputNB -outputB
#define outputPB outputB
#define outputNM -outputM
#define outputPM outputM
#define outputNS -outputS
#define outputPS outputS
#define inputNS -inputS
#define inputPS inputS
#define inputNM -inputM
#define inputPM inputM
#define inputNB -inputB
#define inputPB inputB





//autonomous function:
extern float *AnalyzeInput(float err);
extern float test_calculate(float *subjection1,float *subjection2);
extern void sideLeft(int voltage);
extern void sideRight(int voltage);
extern void position();
extern void resetChassis();
extern void chassis_auto_brake();
extern void chassis_auto_turn_brake();
extern void mission(int mission_parameter);
extern void move(float distance,int mission,int time);
extern void turn(double target,int time);
extern void control_move_debug(bool dir,float target,int mission_parameter);
extern void control_move_back(bool dir,float target,int mission_parameter,int time);
extern void control_move(bool dir,float target,int mission_parameter,int time);
extern void control_move_final(bool dir,float target,int time);
extern void control_move_turn(double target,int time);
extern void control_move_turn_small(double target,int time);
extern void control_snake(float target_left,float target_right,int mission_parameter);
extern void single_shoot_high();
extern void single_shoot_low();
extern void double_shoot();
extern void final_shoot(int para);
extern void trick_move(bool dir,float target,int mission_parameter,int time,int knife_value);
extern void trick_move2(bool dir,float target,int mission_parameter,int time,int knife_value);
//operator function:

extern void dead_knife();
extern void slipping();
extern void chassis_brake();
extern void shoot();
extern void moving();
extern void rolling();
