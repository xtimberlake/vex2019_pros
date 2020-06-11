#include "main.h"
#include "configure.h"


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
 void dead_knife()
  {
    static int before;
    if(R2)
    {
      knife.move_velocity(100);
      before=pros::millis();
    }
    else
    { if(pros::millis()-before>1000)
      knife.tare_position();

      if(knife.get_position()<12)
      {
        knife.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        knife.move_velocity(0);
      }
      else if(knife.get_position()>=50)
      {
        knife.move_velocity(-100);
      }
      else if(knife.get_position()<50&&knife.get_position()>=12)
      {
        knife.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        knife.move_velocity(0);
      }
      else if(knife.get_position()>=12&&knife.get_position()<20)
       knife.move_velocity(-5);
     }

  }
	void slipping()
{
		if(B)
		slip.move_velocity(200);
		else if(X)
		slip.move_velocity(-200);
		else
		{
				slip.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
				slip.move_velocity(0);

		}
}
void chassis_brake()
  {
    Left1.move_velocity(0);
    Right1.move_velocity(0);
    Left2.move_velocity(0);
    Right2.move_velocity(0);
    Left3.move_velocity(0);
    Right3.move_velocity(0);
    Left4.move_velocity(0);
    Right4.move_velocity(0);
  }

	void shoot()
  {
    if(R1)
    {
      shootL.move_velocity(200);
      shootR.move_velocity(200);
    }
    else
    {
      shootL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      shootR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      shootL.move_velocity(0);
      shootR.move_velocity(0);
    }
  }
	void moving()
  {
    static float radius;
    radius=(pow( master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X),2)+pow( master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),2));
    if(radius>4)
    {
      Left1.move(axis_y+axis_x*turnRatio);   Right1.move(axis_y-axis_x*turnRatio);
      Left2.move(axis_y+axis_x*turnRatio);   Right2.move(axis_y-axis_x*turnRatio);
      Left3.move(axis_y+axis_x*turnRatio);   Right3.move(axis_y-axis_x*turnRatio);
      Left4.move(axis_y+axis_x*turnRatio);   Right4.move(axis_y-axis_x*turnRatio);
    }
    else
    chassis_brake();

  }
	void rolling()
{
  if(L1)
  roll.move_velocity(200);
  else if(L2) roll.move_velocity(-200);
  else roll.move_velocity(0);
}

void opcontrol()
{
  Left1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  Right1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  Left2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  Right2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  Left3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  Right3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  Left4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  Right4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	while (true)
	{
		dead_knife();
		slipping();
		rolling();
		moving();
		shoot();
		if(UP)
		{
			autonomous_para=1;
			master.clear(); pros::delay(50);
			if(autonomous_para==1)
			{master.print(0,0,"Let'sGo RED!"); pros::delay(50);}
			else
			{master.print(0,0,"SwitchError"); pros::delay(50);}
		}
		if(DOWN)
		{
			autonomous_para=2;
			master.clear(); pros::delay(50);
			if(autonomous_para==2)
			{master.print(0,0,"Let'sGo BULE!"); pros::delay(50);}
			else
			{master.print(0,0,"SwitError"); pros::delay(50);}
		}
		if(A) autonomous();
    if(RIGHT)
    {
      move(-141,40,1100);
      turn(142.5,1700);
    Left1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Right1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Left2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Right2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Left3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Right3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    Left4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    Right4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
    if(LEFT)
    {
      move(-128,40,1000);
      turn(-147,1600);
      Left1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      Right1.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      Left2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      Right2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      Left3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      Right3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      Left4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      Right4.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
	}
}
