#include "main.h"
#include "configure.h"

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
 float *AnalyzeInput(float err);
 float test_calculate(float *subjection1,float *subjection2);
 void sideLeft(int voltage);
 void sideRight(int voltage);
 void position();
 void resetChassis();
 void chassis_auto_brake();
 void chassis_auto_turn_brake();
 void mission(int mission_parameter);
 void move(float distance,int mission,int time);
 void turn(double target,int time);
 void control_move_debug(bool dir,float target,int mission_parameter);
 void control_move_back(bool dir,float target,int mission_parameter,int time);
 void control_move(bool dir,float target,int mission_parameter,int time);
 void control_move_final(bool dir,float target,int time);
 void control_move_turn(double target,int time);
 void control_move_turn_small(double target,int time);
 void control_snake(float target_left,float target_right,int mission_parameter);
 void single_shoot_high();
 void single_shoot_low();
 void double_shoot();
 void final_shoot(int para);
 void trick_move(bool dir,float target,int mission_parameter,int time,int knife_value);
 void trick_move2(bool dir,float target,int mission_parameter,int time,int knife_value);
 float *AnalyzeInput(float err)
 {
   float *M=new float[4];

   if(err==inputNS||err==inputNM||err==inputNB||err==inputPS||err==inputPM||err==inputPB||err==inputZO)
   {
     if(err==inputNS) {M[0]=3; M[2]=4;}
     else if(err==inputNM) {M[0]=2; M[2]=3;}
     else if(err==inputNB) {M[0]=1; M[2]=2;}
     else if(err==inputPS) {M[0]=5; M[2]=6;}
     else if(err==inputPM) {M[0]=6; M[2]=7;}
     else if(err==inputPB) {M[0]=7; M[2]=7;}
     else if(err==inputZO) {M[0]=4;M[2]=5;}
     M[1]=1;
     M[3]=0;
   }
   else
   {
     if(err<inputNM&&err>inputNB)
                      {M[0]=1;M[1]=(inputNM-err)/(inputNM-inputNB);
                       M[2]=2;M[3]=(err-inputNB)/(inputNM-inputNB);}
     else if(err>inputNM&&err<inputNS)
                      {M[0]=2;M[1]=(inputNS-err)/(inputNS-inputNM);
                       M[2]=3;M[3]=(err-inputNM)/(inputNS-inputNM);}
     else if(err>inputNS&&err<inputZO)
                      {M[0]=3;M[1]=(inputZO-err)/(inputZO-inputNS);
                       M[2]=4;M[3]=(err-inputNS)/(inputZO-inputNS);}
     else if(err>inputZO&&err<inputPS)
                      {M[0]=4;M[1]=(inputPS-err)/(inputPS-inputZO);
                       M[2]=5;M[3]=(err-inputZO)/(inputPS-inputZO);}
     else if(err>inputPS&&err<inputPM)
                      {M[0]=5;M[1]=(inputPM-err)/(inputPM-inputPS);
                       M[2]=6;M[3]=(err-inputPS)/(inputPM-inputPS);}
     else if(err>inputPM&&err<inputPB)
                      {M[0]=6;M[1]=(inputPB-err)/(inputPB-inputPM);
                       M[2]=7;M[3]=(err-inputPM)/(inputPB-inputPM);}
     else if(err>inputPB)
                     {M[0]=7;M[1]=1;
                      M[2]=7;M[3]=0;}
     else if(err<inputNB)
                     {M[0]=1;M[1]=1;
                      M[2]=2;M[3]=0;}
   }
   return M;
 }



   float test_calculate(float *subjection1,float *subjection2)
   {
     float output1,output2,output3,output4;
     float result;

     switch((int)subjection1[0]*10+(int)subjection2[0])
     {
       case 11:
       case 12:
       case 13:
       case 14:
       case 21:
       case 22:
       case 23:
       case 31:  output1=outputPB*subjection1[1]*subjection2[1]; break;

       case 15:
       case 24:
       case 25:
       case 32:
       case 33:
       case 41:
       case 42:  output1=outputPM*subjection1[1]*subjection2[1]; break;

       case 34:
       case 43:
       case 51:
       case 52:  output1=outputPS*subjection1[1]*subjection2[1]; break;

       case 16:
       case 17:
       case 26:
       case 27:
       case 35:
       case 44:
       case 53:
       case 61:
       case 62:
       case 63:
       case 71: output1=0; break;

       case 36:
       case 45:
       case 72: output1=outputNS*subjection1[1]*subjection2[1]; break;

       case 37:
       case 46:
       case 47:
       case 54:
       case 55:
       case 56:
       case 64: output1=outputNM*subjection1[1]*subjection2[1]; break;

       case 57:
       case 65:
       case 66:
       case 67:
       case 73:
       case 74:
       case 75:
       case 76:
       case 77: output1=outputNB*subjection1[1]*subjection2[1]; break;

     }
     switch((int)subjection1[0]*10+(int)subjection2[2])
     {
       case 11:
       case 12:
       case 13:
       case 14:
       case 21:
       case 22:
       case 23:
       case 31:  output2=outputPB*subjection1[1]*subjection2[3]; break;

       case 15:
       case 24:
       case 25:
       case 32:
       case 33:
       case 41:
       case 42:  output2=outputPM*subjection1[1]*subjection2[3]; break;

       case 34:
       case 43:
       case 51:
       case 52:  output2=outputPS*subjection1[1]*subjection2[3]; break;

       case 16:
       case 17:
       case 26:
       case 27:
       case 35:
       case 44:
       case 53:
       case 61:
       case 62:
       case 63:
       case 71:  output2=0; break;

       case 36:
       case 45:
       case 72: output2=outputNS*subjection1[1]*subjection2[3]; break;

       case 37:
       case 46:
       case 47:
       case 54:
       case 55:
       case 56:
       case 64: output2=outputNM*subjection1[1]*subjection2[3]; break;

       case 57:
       case 65:
       case 66:
       case 67:
       case 73:
       case 74:
       case 75:
       case 76:
       case 77: output2=outputNB*subjection1[1]*subjection2[3]; break;

     }
     switch((int)subjection1[2]*10+(int)subjection2[0])
     {
       case 11:
       case 12:
       case 13:
       case 14:
       case 21:
       case 22:
       case 23:
       case 31:  output3=outputPB*subjection1[3]*subjection2[1]; break;

       case 15:
       case 24:
       case 25:
       case 32:
       case 33:
       case 41:
       case 42:  output3=outputPM*subjection1[3]*subjection2[1]; break;

       case 34:
       case 43:
       case 51:
       case 52:  output3=outputPS*subjection1[3]*subjection2[1]; break;

       case 16:
       case 17:
       case 26:
       case 27:
       case 35:
       case 44:
       case 53:
       case 61:
       case 62:
       case 63:
       case 71:  output3=0; break;

       case 36:
       case 45:
       case 72: output3=outputNS*subjection1[3]*subjection2[1]; break;

       case 37:
       case 46:
       case 47:
       case 54:
       case 55:
       case 56:
       case 64: output3=outputNM*subjection1[3]*subjection2[1]; break;

       case 57:
       case 65:
       case 66:
       case 67:
       case 73:
       case 74:
       case 75:
       case 76:
       case 77: output3=outputNB*subjection1[3]*subjection2[1]; break;

     }
     switch((int)subjection1[2]*10+(int)subjection2[2])
     {

       case 11:
       case 12:
       case 13:
       case 14:
       case 21:
       case 22:
       case 23:
       case 31:  output4=outputPB*subjection1[3]*subjection2[3]; break;

       case 15:
       case 24:
       case 25:
       case 32:
       case 33:
       case 41:
       case 42:  output4=outputPM*subjection1[3]*subjection2[3]; break;

       case 34:
       case 43:
       case 51:
       case 52:  output4=outputPS*subjection1[3]*subjection2[3]; break;

       case 16:
       case 17:
       case 26:
       case 27:
       case 35:
       case 44:
       case 53:
       case 61:
       case 62:
       case 63:
       case 71:  output4=0; break;

       case 36:
       case 45:
       case 72: output4=outputNS*subjection1[3]*subjection2[3]; break;

       case 37:
       case 46:
       case 47:
       case 54:
       case 55:
       case 56:
       case 64: output4=outputNM*subjection1[3]*subjection2[3]; break;

       case 57:
       case 65:
       case 66:
       case 67:
       case 73:
       case 74:
       case 75:
       case 76:
       case 77: output4=outputNB*subjection1[3]*subjection2[3]; break;

     }
     result=output1+output2+output3+output4;
     return result;
   }
   void chassis_auto_brake()
{
Left1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
Left1.move_velocity(0);
Right1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
Right1.move_velocity(0);
Left2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Right2.move_velocity(0);
Right2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Right2.move_velocity(0);
Left3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Left3.move_velocity(0);
Right3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Right3.move_velocity(0);
Left4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Left4.move_velocity(0);
Right4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
Right4.move_velocity(0);
}
   void sideLeft(int voltage)
{
  Left1.move(voltage);
  Left2.move(voltage);
  Left3.move(voltage);
  Left4.move(voltage);
}
void sideRight(int voltage)
{
  Right1.move(voltage);
  Right2.move(voltage);
  Right3.move(voltage);
  Right4.move(voltage);
}
 void chassis_auto_turn_brake()
 {
 Left1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 Left1.move_velocity(0);
 Right1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
 Right1.move_velocity(0);
 Left2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 Right2.move_velocity(0);
 Right2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 Right2.move_velocity(0);
 Left3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 Left3.move_velocity(0);
 Right3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 Right3.move_velocity(0);
 Left4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 Left4.move_velocity(0);
 Right4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
 Right4.move_velocity(0);
 }
 void position()
{
  master.clear_line(0); pros::delay(50);
  master.clear_line(1); pros::delay(50);
  master.clear_line(2); pros::delay(50);
  master.print(1, 0,"Left=%lf",Left_encoder); pros::delay(50);
  master.print(2, 0,"Right=%lf",Right_encoder); pros::delay(50);
  resetChassis();
}
void resetChassis()
{
  Left1.tare_position();
  Left2.tare_position();
  Left3.tare_position();
  Left4.tare_position();
  Right1.tare_position();
  Right2.tare_position();
  Right3.tare_position();
  Right4.tare_position();
}
void control_move_debug(bool dir,float target,int mission_parameter)
{
  static double portion=0.35;
  static double integral=0.00001;
  static double derivative=0.06;
  float data[7][3];
  int print=4;
  bool Velocity=0;
  double temp=0;
  float speed=starting_speed;

  double err_left,err_right;
  double err_last_left,err_last_right;
  double err,err_last,ec,output_E,output_EC,adjust_output;
  float *subjection_input1; //1 stands for left
  float *subjection_input2; //2 stands for right
  float speed_average;
  double a_left=0,a_right=0,b_left=0,b_right=0,c_left=0,c_right=0;
  float index_left,index_right;
  float add_left_positive=0,add_left_negative=0,add_right_positive=0,add_right_negative=0;
  int i,left_ready=0,right_ready=0;
  int reslut_left,reslut_right;
  float final_left,final_right;
  float test_adjust_output;
  resetChassis();
  err=Left_encoder-Right_encoder;
  err_last=err;
  while(Left_encoder<target*0.85)
  {
    err_last=err;
    err=Left_encoder-Right_encoder;
    if(speed<=VelocityMax)   { speed+=accel;} //  -12000-12000value

    ec=err-err_last;

    subjection_input1=AnalyzeInput(err);
    subjection_input2=AnalyzeInput(ec);
    test_adjust_output=test_calculate(subjection_input1,subjection_input2);



   /* if(print==4&&Left1.get_position()>target*0.5&&Left1.get_position()<target*0.6)
    {
      print=5;
      data[4][0]=err;
      data[4][1]=ec;
      data[4][2]=test_adjust_output;
    }
*/
    sideLeft(speed+test_adjust_output);
    sideRight(speed-test_adjust_output);


} /*
final_right=Right_encoder;
final_left=Left_encoder;
 while(Left1.get_position()<target&&speed>0)
 {
   err_last=err;
   err=Left_encoder-Right_encoder;
   speed-=0.002;
   ec=err-err_last;

   subjection_input1=AnalyzeInput(err);
   subjection_input2=AnalyzeInput(ec);
   test_adjust_output=test_calculate(subjection_input1,subjection_input2);

   sideLeft(speed+test_adjust_output);
   sideRight(speed-test_adjust_output);
 }



  chassis_brake();
  master.clear(); pros::delay(80);

  master.print(0,0, "left=%lf",final_left); pros::delay(50);
  master.print(1,0, "right=%lf",final_right); pros::delay(50);
  master.print(2,0, "output=%lf",data[4][2]); pros::delay(50);


  //accurate adjust movement
  speed_average=(Left1.get_voltage()+Right2.get_voltage())/800.0; */

  err_last_left=target-Left_encoder;
  err_last_right=target-Right_encoder;
  for(i=0;i<3;)
  {
    err_left=target-Left_encoder;
    err_right=target-Right_encoder;
    if(fabs(err_left)<precision) left_ready=1; else left_ready=0;
    if(fabs(err_right)<precision) right_ready=1; else right_ready=0;
    if(left_ready==1&&right_ready) {i++; chassis_auto_brake(); pros::delay(delay_time); break;}

   /*  index_left=1/fabs(err_left);  if(index_left>1) index_left=1;
     index_right=1/fabs(err_right);  if(index_right>1) index_right=1; */
     if(fabs(err_left)<1)index_left=5; else if(fabs(err_left)<8)index_left=2;  else index_left=1;
     if(fabs(err_right)<10) index_right=2; else index_right=1;

     if(err_left>0) {add_left_positive+=err_left; add_left_negative=0;}
     else {add_left_negative+=err_left; add_left_positive=0;}

 /*    if(err_right>0) {add_right_positive+=err_right; add_right_negative-=0.01;}
     else {add_right_negative+=err_right; add_right_positive-=0.01;} */

   /*  add_left_positive+=err_left;*/
     a_left=err_left*portion;
     a_right=err_right*portion;

     b_left=integral*(add_left_positive+add_left_negative)*index_left;
     b_right=integral*(add_left_positive+add_left_negative)*index_right;

     c_left=derivative*(err_left-err_last_left);
     c_right=derivative*(err_last-err_last_right);

     reslut_left=a_left+b_left+c_right;
     reslut_right=a_left+b_right+c_right;

   if(reslut_left<8&&reslut_left>0) reslut_left=8;
   if(reslut_left>-8&&reslut_left<0) reslut_left=-8;
   if(Left_encoder>Right_encoder)
    {
      sideLeft(reslut_left-1);
      sideRight(reslut_left+1);
    }
    else{
      if(Left_encoder<Right_encoder)
      {
        sideLeft(reslut_left+1);
        sideRight(reslut_left-1);
      }
      else
      {

          sideLeft(reslut_left);
          sideRight(reslut_left);

      }
    }


    err_last_left=err_left;
    err_last_right=err_right;
  }
  chassis_auto_brake();
  pros::delay(delay_time);
  final_right=Right_encoder;
  final_left=Left_encoder;
  master.clear();pros::delay(50);
  master.print(0,0, "left=%lf",final_left); pros::delay(50);
  master.print(1,0, "right=%lf",final_right); pros::delay(50);
}
 void control_move_turn(double target,int time)
 {
   int before=pros::millis();
   double precision_turn=0.1; //.4
   double portion_turn=0.45;
   double integral_turn=0.000001;
   double derivative_turn=1;
   float data[7][3];
   int print=4;
   bool Velocity=0;
   double temp=0;
   float speed=starting_speed;
   float factorVoltage=VelocityMax/127;
   double err_left,err_right;
   double err_last_left,err_last_right;
   double err,err_last,ec,output_E,output_EC,adjust_output;
   float *subjection_input1; //1 stands for left
   float *subjection_input2; //2 stands for right
   float speed_average;
   double a_left=0,a_right=0,b_left=0,b_right=0,c_left=0,c_right=0;
   float index_left,index_right;
   float add_left_positive=0,add_left_negative=0,add_right_positive=0,add_right_negative=0;
   int i,left_ready=0,right_ready=0;
   int reslut_left,reslut_right;
   float final_left,final_right;
   float test_adjust_output;
   resetChassis();
   err=Left_encoder-Right_encoder;

   err_last_left=target-Left_encoder;
   err_last_right=target-Right_encoder;

   for(i=0;i<3;)
   {
     if(pros::millis()-before>time) {chassis_auto_turn_brake(); break;}
     err_left=target-Left_encoder;
     err_right=Right_encoder-target;
     if(fabs(err_left)<precision_turn) left_ready=1; else left_ready=0;

      if(left_ready) {i++; chassis_auto_turn_brake(); pros::delay(200); break;}
      if(fabs(err_left)<10)index_left=4; else if(fabs(err_left)<30)index_left=3;  else index_left=1;
      if(err_left>0) {add_left_positive+=err_left; add_left_negative=0;}
      else {add_left_negative+=err_left; add_left_positive=0;}

      a_left=err_left*portion_turn;


      b_left=integral_turn*(add_left_positive+add_left_negative)*index_left;


      c_left=derivative_turn*(err_left-err_last_left);


      reslut_left=a_left+b_left+c_right;



    if(reslut_left>90) reslut_left=90;

           if(Right_encoder<-Left_encoder)
           {
           sideLeft(reslut_left+6);
           sideRight(-reslut_left+6);
           }
           else{
             if(Right_encoder>-Left_encoder)
            { sideLeft(reslut_left-6);
             sideRight(-reslut_left-6);}
             else
             {
               sideLeft(reslut_left);
               sideRight(-reslut_left);
             }
           }





     err_last_left=err_left;
     err_last_right=err_right;
   }
   chassis_auto_brake();

 }

 void control_move_turn_small(double target,int time)
 {
   int before=pros::millis();
   double precision_turn=0.1; //0.6
   double portion_turn=0.8;
   double integral_turn=0.00001;
   double derivative_turn=100000;
   float data[7][3];
   int print=4;
   bool Velocity=0;
   double temp=0;
   float speed=starting_speed;
   float factorVoltage=VelocityMax/127;
   double err_left,err_right;
   double err_last_left,err_last_right;
   double err,err_last,ec,output_E,output_EC,adjust_output;
   float *subjection_input1; //1 stands for left
   float *subjection_input2; //2 stands for right
   float speed_average;
   double a_left=0,a_right=0,b_left=0,b_right=0,c_left=0,c_right=0;
   float index_left,index_right;
   float add_left_positive=0,add_left_negative=0,add_right_positive=0,add_right_negative=0;
   int i,left_ready=0,right_ready=0;
   int reslut_left,reslut_right;
   float final_left,final_right;
   float test_adjust_output;
   resetChassis();

   err_last_left=target-Left_encoder;


   for(i=0;i<3;)
   {
     if(pros::millis()-before>time) {chassis_auto_turn_brake(); break;}
     err_left=target-Left_encoder;

     if(fabs(err_left)<precision_turn) left_ready=1; else left_ready=0;

      if(left_ready) {i++; chassis_auto_turn_brake(); pros::delay(200); break;}
      if(fabs(err_left)<10)index_left=4; else if(fabs(err_left)<30)index_left=3;  else index_left=1;
      if(err_left>0) {add_left_positive+=err_left; add_left_negative=0;}
      else {add_left_negative+=err_left; add_left_positive=0;}

      a_left=err_left*portion_turn;

      b_left=integral_turn*(add_left_positive+add_left_negative)*index_left;

      c_left=derivative_turn*(err_left-err_last_left);


      reslut_left=a_left+b_left+c_right;



    if(reslut_left>90) reslut_left=90;

           if(Right_encoder<-Left_encoder)
           {
           sideLeft(reslut_left+6);
           sideRight(-reslut_left+6);
           }
           else{
             if(Right_encoder>-Left_encoder)
            { sideLeft(reslut_left-6);
             sideRight(-reslut_left-6);}
             else
             {
               sideLeft(reslut_left);
               sideRight(-reslut_left);
             }
           }





     err_last_left=err_left;

   }
   chassis_auto_brake();

 }
 void turn(double target,int time)
 {
   if(fabs(target)<180) control_move_turn_small(target,time);
   else control_move_turn(target,time);
 }
 void single_shoot_high()
{
  if(limit.get_value())
  {
    while(limit.get_value())
    {
      shootR.move_velocity(200);
      shootL.move_velocity(200);
    }
    while(!limit.get_value())
    {
      shootR.move_velocity(200);
      shootL.move_velocity(200);
    }
    shootR.move_velocity(0);
    shootL.move_velocity(0);
  }
  else
  {
    while(!limit.get_value())
    {
      shootR.move_velocity(200);
      shootL.move_velocity(200);
    }
    while(limit.get_value())
    {
      shootR.move_velocity(200);
      shootL.move_velocity(200);
    }
    shootR.move_velocity(0);
    shootL.move_velocity(0);
  }
}
void single_shoot_low()
{
  while(slip.get_torque()<0.52)
  slip.move_velocity(200);

  if(limit.get_value())
  {
    while(limit.get_value())
   { shootR.move_velocity(200);
    shootL.move_velocity(200);}
    while(!limit.get_value())
   { shootR.move_velocity(200);
    shootL.move_velocity(200);}

  }
  else
  {
    while(!limit.get_value())
   { shootR.move_velocity(200);
    shootL.move_velocity(200);}
    while(limit.get_value())
   { shootR.move_velocity(200);
    shootL.move_velocity(200);}
  }
  roll.move_velocity(200);
  shootR.move_velocity(0);
  shootL.move_velocity(0);

  slip.move_velocity(-200);
  pros::delay(180);
  slip.move_velocity(0);
}
void double_shoot()
{
  int before=pros::millis();
  while(1)
  {
  if(pros::millis()-before>300) break;
  slip.move_velocity(200);
}
  roll.move_velocity(100);
  if(limit.get_value())
  {
    while(limit.get_value())
   { shootR.move_velocity(200);
    shootL.move_velocity(200);}

   while(!limit.get_value())
  { shootR.move_velocity(200);
   shootL.move_velocity(200);
  roll.move_velocity(200);
 }
 }
  else
  {
    while(!limit.get_value())
   { shootR.move_velocity(200);
    shootL.move_velocity(200);}
    while(limit.get_value())
   { shootR.move_velocity(200);
    shootL.move_velocity(200);
  roll.move_velocity(200);
  }
   }


  shootR.move_velocity(0);
  shootL.move_velocity(0);
  roll.move_velocity(200);
  slip.move_velocity(-200);
  pros::delay(230);

  if(limit.get_value())
  {
    while(limit.get_value())
   {
    shootR.move_velocity(200);
    shootL.move_velocity(200);
   }
   while(!limit.get_value())
  {
   shootR.move_velocity(200);
   shootL.move_velocity(200);
 }
  }
  else
  {
    while(!limit.get_value())
   {
    shootR.move_velocity(200);
    shootL.move_velocity(200);
  }
  while(limit.get_value())
 {
  shootR.move_velocity(200);
  shootL.move_velocity(200);
 }
  }
    roll.move_velocity(0);
   shootR.move_velocity(0);
   shootL.move_velocity(0);
   roll.move_velocity(0);
   slip.move_velocity(0);
}

 void control_move_back(bool dir,float target,int mission_parameter,int time)
 {
   int before=pros::millis();
   static double portion=0.35;
   static double integral=0.00001;
   static double derivative=600000;
   float speed=starting_speed;
   double err_left,err_right;
   double err_last_left,err_last_right;
   double err,err_last,ec;
   float *subjection_input1; //1 stands for left
   float *subjection_input2; //2 stands for right
   float speed_average;
   double a_left=0,a_right=0,b_left=0,b_right=0,c_left=0,c_right=0;
   float index_left;
   float add_left_positive=0,add_left_negative=0;
   int i,left_ready=0,right_ready=0;
   float reslut_left,reslut_right;
   float test_adjust_output;
   resetChassis();
   err=fabs(Right_encoder)-fabs(Left_encoder);
   err_last=err;
   mission(mission_parameter);
   while(Left_encoder>target*0.85)
   {
     if(pros::millis()-before>time)  {chassis_auto_turn_brake(); break;}
     err_last=err;
     err=fabs(Right_encoder)-fabs(Left_encoder);
     if(speed<=VelocityMax)   { speed+=accel;} //  -12000-12000value

     ec=err-err_last;

     subjection_input1=AnalyzeInput(err);
     subjection_input2=AnalyzeInput(ec);
     test_adjust_output=test_calculate(subjection_input1,subjection_input2);

     sideRight(-(speed+test_adjust_output));
     sideLeft(-(speed-test_adjust_output));
     mission(mission_parameter);
 }

   err_last_left=Left_encoder-target;
   err_last_right=Right_encoder-target;
   for(i=0;i<3;)
   {
     if(pros::millis()-before>time)  {chassis_auto_turn_brake(); break;}
     err_left=Left_encoder-target;
     err_right=Right_encoder-target;
     if(fabs(err_left)<precision) left_ready=1; else left_ready=0;
     if(fabs(err_right)<precision) right_ready=1; else right_ready=0;
     if(left_ready) {i++; chassis_auto_brake(); pros::delay(delay_time); break;}


      if(fabs(err_left)<1)index_left=5; else if(fabs(err_left)<8)index_left=2;  else index_left=1;


      if(err_left>0) {add_left_positive+=err_left; add_left_negative=0;}
      else {add_left_negative+=err_left; add_left_positive=0;}

      a_left=err_left*portion;

      b_left=integral*(add_left_positive+add_left_negative)*index_left;

      c_left=derivative*(err_left-err_last_left);

      reslut_left=a_left+b_left+c_left;

    if(reslut_left<8&&reslut_left>0) reslut_left=8;
    if(reslut_left>-8&&reslut_left<0) reslut_left=-8;
    if(fabs(Left_encoder)>fabs(Right_encoder))
     {
       sideLeft(-(reslut_left-1));
       sideRight(-(reslut_left+1));
     }
     else{
       if(fabs(Left_encoder)<fabs(Right_encoder))
       {
         sideLeft(-(reslut_left+1));
         sideRight(-(reslut_left-1));
       }
       else
       {

           sideLeft(-reslut_left);
           sideRight(-reslut_left);

       }
     }
     err_last_left=err_left;
     err_last_right=err_right;
     mission(mission_parameter);
   }
   VelocityMax=95;
 }
 void control_move_final(bool dir,float target,int time)
 {
   static double portion=0.35;
   static double integral=0.0000001;
   static double derivative=600000;

   int before=pros::millis();
   float speed=starting_speed;
   double err_left,err_right;
   double err_last_left,err_last_right;
   double err,err_last,ec;
   float *subjection_input1; //1 stands for left
   float *subjection_input2; //2 stands for right
   float speed_average;
   double a_left=0,a_right=0,b_left=0,b_right=0,c_left=0,c_right=0;
   float index_left;
   float add_left_positive=0,add_left_negative=0;
   int i,left_ready=0,right_ready=0;
   float reslut_left,reslut_right;
   float test_adjust_output;
   resetChassis();
   err=Left_encoder-Right_encoder;
   err_last=err;

   VelocityMax=72;
   while(Left_encoder<target*0.85)
   {
     err_last=err;
     err=Left_encoder-Right_encoder;
     if(speed<=VelocityMax)   { speed+=accel;} //  -12000-12000value

     ec=err-err_last;

     subjection_input1=AnalyzeInput(err);
     subjection_input2=AnalyzeInput(ec);
     test_adjust_output=test_calculate(subjection_input1,subjection_input2);

     sideLeft(speed+test_adjust_output);
     sideRight(speed-test_adjust_output);

 }

   err_last_left=target-Left_encoder;
   err_last_right=target-Right_encoder;
   for(i=0;i<3;)
   {
     err_left=target-Left_encoder;
     err_right=target-Right_encoder;
     if(fabs(err_left)<precision) left_ready=1; else left_ready=0;
     if(fabs(err_right)<precision) right_ready=1; else right_ready=0;
     if((pros::millis()-before)>time) break;
     if(left_ready==1&&right_ready) {i++; chassis_auto_brake(); pros::delay(delay_time); break;}


      if(fabs(err_left)<1)index_left=1; else if(fabs(err_left)<8)index_left=1;  else index_left=1;


      if(err_left>0) {add_left_positive+=err_left; add_left_negative=0;}
      else {add_left_negative+=err_left; add_left_positive=0;}

      a_left=err_left*portion;

      b_left=integral*(add_left_positive+add_left_negative)*index_left;

      c_left=derivative*(err_left-err_last_left);

      reslut_left=a_left+b_left+c_left;

    if(err_left<66) reslut_left=2;

    if(Left_encoder>Right_encoder)
     {
       sideLeft(reslut_left-1);
       sideRight(reslut_left+1);
     }
     else{
       if(Left_encoder<Right_encoder)
       {
         sideLeft(reslut_left+1);
         sideRight(reslut_left-1);
       }
       else
       {

           sideLeft(reslut_left);
           sideRight(reslut_left);

       }
     }
     err_last_left=err_left;
     err_last_right=err_right;

   }
   VelocityMax=95;
 }
 void control_move(bool dir,float target,int mission_parameter,int time)
 {
   int before=pros::millis();
   static double portion=0.35;
   static double integral=0.00001;
   static double derivative=600000;
   float speed=starting_speed;
   double err_left,err_right;
   double err_last_left,err_last_right;
   double err,err_last,ec;
   float *subjection_input1; //1 stands for left
   float *subjection_input2; //2 stands for right
   float speed_average;
   double a_left=0,a_right=0,b_left=0,b_right=0,c_left=0,c_right=0;
   float index_left;
   float add_left_positive=0,add_left_negative=0;
   int i,left_ready=0,right_ready=0;
   float reslut_left,reslut_right;
   float test_adjust_output;
   resetChassis();
   err=Left_encoder-Right_encoder;
   err_last=err;
   mission(mission_parameter);
   while(Left_encoder<target*0.85)
   {
     if(pros::millis()-before>time)  {chassis_auto_turn_brake(); break;}
     err_last=err;
     err=Left_encoder-Right_encoder;
     if(speed<=VelocityMax)   { speed+=accel;} //

     ec=err-err_last;

     subjection_input1=AnalyzeInput(err);
     subjection_input2=AnalyzeInput(ec);
     test_adjust_output=test_calculate(subjection_input1,subjection_input2);

     sideLeft(speed+test_adjust_output);
     sideRight(speed-test_adjust_output);
     mission(mission_parameter);
 }

   err_last_left=target-Left_encoder;
   err_last_right=target-Right_encoder;
   for(i=0;i<3;)
   {
     if(pros::millis()-before>time)  {chassis_auto_turn_brake(); break;}
     err_left=target-Left_encoder;
     err_right=target-Right_encoder;
     if(fabs(err_left)<precision) left_ready=1; else left_ready=0;
     if(fabs(err_right)<precision) right_ready=1; else right_ready=0;
     if(left_ready==1&&right_ready) {i++; chassis_auto_brake(); pros::delay(delay_time); break;}


      if(fabs(err_left)<1)index_left=5; else if(fabs(err_left)<8)index_left=2;  else index_left=1;


      if(err_left>0) {add_left_positive+=err_left; add_left_negative=0;}
      else {add_left_negative+=err_left; add_left_positive=0;}

      a_left=err_left*portion;

      b_left=integral*(add_left_positive+add_left_negative)*index_left;

      c_left=derivative*(err_left-err_last_left);

      reslut_left=a_left+b_left+c_left;

    if(reslut_left<8&&reslut_left>0) reslut_left=8;
    if(reslut_left>-8&&reslut_left<0) reslut_left=-8;
    if(Left_encoder>Right_encoder)
     {
       sideLeft(reslut_left-1);
       sideRight(reslut_left+1);
     }
     else{
       if(Left_encoder<Right_encoder)
       {
         sideLeft(reslut_left+1);
         sideRight(reslut_left-1);
       }
       else
       {
           sideLeft(reslut_left);
           sideRight(reslut_left);
       }
     }
     err_last_left=err_left;
     err_last_right=err_right;
     mission(mission_parameter);
   }
   VelocityMax=95;
 }

void move(float distance,int mission,int time)
{
  if(distance>0)
  control_move(1,distance,mission,time);
  else
  {
  control_move_back(-1,distance,mission,time);
  }
}
void mission(int mission_parameter)
{
  switch(mission_parameter)
  {
    case 0: break;

    case 1: VelocityMax=100;
            if(empty&&infrared.get_value()) {count++; empty=0;}
            if(!infrared.get_value()) empty=1; else empty=0;
            if(knife.get_position()<52) knife.move_velocity(100);
            else {knife.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); knife.move_velocity(0);}
            if(Right_encoder>500&&infrared.get_value()==0) roll.move_velocity(130);
            else roll.move_velocity(0); break;
    case 2:
            if(empty&&infrared.get_value()) {count++; empty=0;}
            if(!infrared.get_value()) empty=1; else empty=0;
            if(fabs(Right_encoder)>200) {knife.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); knife.move_velocity(0);}
            if(!infrared.get_value()) roll.move_velocity(130);
            else roll.move_velocity(0);
            if(fabs(Right_encoder)>1200) roll.move_velocity(0); break;
    case 3: VelocityMax=65; precision=0.5;
            if(empty&&infrared.get_value()) {count++; empty=0;}
            if(!infrared.get_value()) empty=1; else empty=0;
            if(Right_encoder>400&&count<2) roll.move_velocity(130);
            if(Right_encoder>735) knife.move_velocity(100);
            break;

    case 4:
            if(empty&&infrared.get_value()) {count++; empty=0;}
            if(!infrared.get_value()) empty=1; else empty=0;
            VelocityMax=65; precision=0.5;
            if(!infrared.get_value()) roll.move_velocity(130); else roll.move_velocity(0);
            if(fabs(Right_encoder)<100&&count<2) knife.move_velocity(80);
            if(fabs(Right_encoder)>345) knife.move_velocity(0);
            break;

    case 5:
            if(Right_encoder<800) knife.move_velocity(100);
            if(Right_encoder>800&&Right_encoder<990) knife.move_velocity(-70);
            if(Right_encoder>991) {knife.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); knife.move_velocity(0);} break;

    case 6:         VelocityMax=65;
                    if(empty&&infrared.get_value()) {count++; empty=0;}
                    if(!infrared.get_value()) empty=1; else empty=0;
                    if(Right_encoder>400&&count<2) roll.move_velocity(130);
                    if(Right_encoder>700) knife.move_velocity(100);
                    break;
    case 7:      VelocityMax=65; precision=0.5;
                            if(empty&&infrared.get_value()) {count++; empty=0;}
                            if(!infrared.get_value()) empty=1; else empty=0;
                            if(Right_encoder>400&&count<2) roll.move_velocity(130);
                            if(Right_encoder>705) knife.move_velocity(100);
                            break;

    case 8:     if(empty&&infrared.get_value()) {count++; empty=0;}
                if(!infrared.get_value()) empty=1; else empty=0;
                VelocityMax=65;
                if(Right_encoder>400&&count<2) roll.move_velocity(130);
                if(Right_encoder>670) knife.move_velocity(100);
                break;

    case 9:     VelocityMax=65;
                if(empty&&infrared.get_value()) {count++; empty=0;}
                if(!infrared.get_value()) empty=1; else empty=0;
                if(Right_encoder>400&&count<2) roll.move_velocity(130);
                if(Right_encoder>720) knife.move_velocity(100);
                break;

    case 10:    VelocityMax=40; precision=0.5;
                if(empty&&infrared.get_value()) {count++; empty=0;}
                if(!infrared.get_value()) empty=1; else empty=0;
                if(Right_encoder>222&&count<2) roll.move_velocity(130); else if(Right_encoder>80&&count<2) roll.move_velocity(200);
                if(Right_encoder>224) knife.move_velocity(100);
                break;

    case 11:    if(empty&&infrared.get_value()) {count++; empty=0;}
                if(!infrared.get_value()) empty=1; else empty=0;
                VelocityMax=40; precision=0.6;
                if(Right_encoder>108&&count<2) roll.move_velocity(130); else if(count<2) roll.move_velocity(200);
                if(Right_encoder>202) knife.move_velocity(100);
                break;
    case 12:VelocityMax=100;
            if(empty&&infrared.get_value()) {count++; empty=0;}
            if(!infrared.get_value()) empty=1; else empty=0;
            if(Right_encoder>500&&infrared.get_value()==0&&count<2) roll.move_velocity(130);
            else roll.move_velocity(0);
            break;
    case 13:
            if(empty&&infrared.get_value()) {count++; empty=0;}
            if(!infrared.get_value()) empty=1; else empty=0;
            VelocityMax=40;
            if(infrared.get_value()==0&&count<2) roll.move_velocity(130);
            else roll.move_velocity(0);
            if(Right_encoder>150) knife.move_velocity(-100);
            break;



    case 14:VelocityMax=60;
            if(count<2) roll.move_velocity(150); else roll.move_velocity(0);
            if(empty&&infrared.get_value()) {count++; empty=0;}
            if(!infrared.get_value()) empty=1; else empty=0;break;
    case 40:VelocityMax=40; break;
    case 50:VelocityMax=50; break;
    case 60:VelocityMax=60; break;
    case 70:VelocityMax=70; break;
    case 100:VelocityMax=100; break;
  }
}
void final_shoot(int para)
{int p_before;
  {
    single_shoot_low();
    slip.move_velocity(-200);
    roll.move_velocity(200);
    pros::delay(300);

    p_before=slip.get_position();
    while(slip.get_position()-p_before<para)
    {
      slip.move_velocity(120);
    }
    slip.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    slip.move_velocity(0);
    if(limit.get_value())
    {
      while(limit.get_value())
     { shootR.move_velocity(200);
      shootL.move_velocity(200);}
      while(!limit.get_value())
     { shootR.move_velocity(200);
      shootL.move_velocity(200);}

    }
    else
    {
      while(!limit.get_value())
     { shootR.move_velocity(200);
      shootL.move_velocity(200);}
      while(limit.get_value())
     { shootR.move_velocity(200);
      shootL.move_velocity(200);}
    }
    slip.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    slip.move_velocity(0);
  }
  roll.move_velocity(0);
}
void trick_move2(bool dir,float target,int mission_parameter,int time,int knife_value)
{
  int before=pros::millis();
  static double portion=0.35;
  static double integral=0.00001;
  static double derivative=600000;
  float speed=starting_speed;
  double err_left,err_right;
  double err_last_left,err_last_right;
  double err,err_last,ec;
  float *subjection_input1; //1 stands for left
  float *subjection_input2; //2 stands for right
  float speed_average;
  double a_left=0,a_right=0,b_left=0,b_right=0,c_left=0,c_right=0;
  float index_left;
  float add_left_positive=0,add_left_negative=0;
  int i,left_ready=0,right_ready=0;
  float reslut_left,reslut_right;
  float test_adjust_output;
  resetChassis();
  knife.tare_position();
  err=Left_encoder-Right_encoder;
  err_last=err;
  mission(mission_parameter);
  while(Left_encoder<target*0.7)
  {
    err_last=err;
    err=Left_encoder-Right_encoder;
    if(speed<=VelocityMax)   { speed+=accel;} //  -12000-12000value

    ec=err-err_last;

    subjection_input1=AnalyzeInput(err);
    subjection_input2=AnalyzeInput(ec);
    test_adjust_output=test_calculate(subjection_input1,subjection_input2);

    sideLeft(speed+test_adjust_output);
    sideRight(speed-test_adjust_output);
    mission(mission_parameter);
    if(knife.get_position()<knife_value&&knife_para) knife.move_velocity(100);
    else if(knife_para) {knife.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); knife.move_velocity(0);}
}

  while(Left_encoder<target*0.95)
{
  sideLeft(speed/1.3);
  sideRight(speed);
  if(speed>=0) speed-=0.0005; else break;
  mission(mission_parameter);
}
  knife.move_velocity(100);
  move(250,13,1000);
  knife.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); knife.move_velocity(0);
  VelocityMax=95;

}
void trick_move(bool dir,float target,int mission_parameter,int time,int knife_value)
{
  int before=pros::millis();
  static double portion=0.35;
  static double integral=0.00001;
  static double derivative=600000;
  float speed=starting_speed;
  double err_left,err_right;
  double err_last_left,err_last_right;
  double err,err_last,ec;
  float *subjection_input1; //1 stands for left
  float *subjection_input2; //2 stands for right
  float speed_average;
  double a_left=0,a_right=0,b_left=0,b_right=0,c_left=0,c_right=0;
  float index_left;
  float add_left_positive=0,add_left_negative=0;
  int i,left_ready=0,right_ready=0;
  float reslut_left,reslut_right;
  float test_adjust_output;
  resetChassis();
  knife.tare_position();
  err=Left_encoder-Right_encoder;
  err_last=err;
  mission(mission_parameter);
  while(Left_encoder<target*0.7)
  {
    err_last=err;
    err=Left_encoder-Right_encoder;
    if(speed<=VelocityMax)   { speed+=accel;} //  -12000-12000value

    ec=err-err_last;

    subjection_input1=AnalyzeInput(err);
    subjection_input2=AnalyzeInput(ec);
    test_adjust_output=test_calculate(subjection_input1,subjection_input2);

    sideLeft(speed+test_adjust_output);
    sideRight(speed-test_adjust_output);
    mission(mission_parameter);
    if(knife.get_position()<knife_value&&knife_para) knife.move_velocity(100);
    else if(knife_para) {knife.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); knife.move_velocity(0);}
}

  while(Left_encoder<target*0.95)
{
  sideLeft(speed);
  sideRight(speed/1.3);
  if(speed>=0) speed-=0.0005; else break;
  mission(mission_parameter);
}
  knife.move_velocity(100);
  move(250,13,1000);
  knife.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); knife.move_velocity(0);
  VelocityMax=95;

}
void to_platform(int time)
{
  sideLeft(20);
  sideRight(20);
  pros::delay(time);
}
void autonomous()
{
  int delay=1;
  switch(autonomous_para)
  {
    int now,before,count,p_before;

    case 1:
            before=pros::millis();
            move(1080,1,2500);
            move(-1217,2,2100);
            turn(-269.9,2200);
            move(1360,100,1800);
            if(delay)
          {  while(1)
            if(pros::millis()-before>12000) break; }
            move(-1238,0,2100);//1215
            turn(372,1600);
            move(500,60,1300);
            turn(-283,1100);
            if(delay)
          {  while(1)
            if(pros::millis()-before>20000) break; }
            single_shoot_low();
            turn(295,1100);
            move(234,10,1300);
            move(-400,4,1800);
            count=pros::millis();
            while(!infrared.get_value())
            {
              if(pros::millis()-count>3000) break;
              roll.move_velocity(130);
            }
            if(infrared.get_value()) count++;
            roll.move_velocity(0);
            turn(146,2400);
            move(659,14,2500);
            roll.move_velocity(0);
            turn(-268,1500);
            pros::delay(500);
            if(0)
            {
            while(1)
            if(pros::millis()-before>37500) break;}
        //    control_move_final(1,1110,3400);
            control_move_final(1,1000,3400);
            to_platform(1500);
            move(-128,40,1000);
            turn(-147,1600);
            if(infrared.get_value())
          {
            if(delay)
            {
              while(1)
            {
              if(pros::millis()-before>42800) break;
            }
            }
            final_shoot(64);
          }
            else
            {
              roll.move_velocity(-180);

              if(delay)
              {
              while(1)
              {
                if(pros::millis()-before>44200) break;
              }
              }
            single_shoot_low();
            }
            now=pros::millis()-before;
            master.clear(); pros::delay(50);
            master.print(0,0,"time:%d Msec",now);
            break;



        case 2:
                          before=pros::millis();
                          move(1080,1,2500);
                          move(-1210,2,2100);
                          turn(272,2100);
                          move(1345,100,1800);
                          if(delay)
                          {while(1)
                          if(pros::millis()-before>12000) break;}

                          move(-1195,0,2100);

                          turn(-376,1600);
                          move(480,60,1850);
                          turn(287,1400);
                          if(delay)
                          {
                          while(1)
                          if(pros::millis()-before>25000) break; }
                          single_shoot_low();
                          turn(-307,2000);
                          move(212,11,1700);
                          move(-415,4,2200);
                          count=pros::millis();
                          while(!infrared.get_value())
                          {
                            if(pros::millis()-count>3000) break;
                            roll.move_velocity(130);
                          }
                          if(infrared.get_value()) count++;
                          roll.move_velocity(0);
                          turn(-140,2500);
                          move(650,14,2800);
                          roll.move_velocity(0);
                          turn(280,2500);
                          if(delay)
                          {while(1)
                          if(pros::millis()-before>37500) break;}
                          control_move_final(1,1150,3400);

                          move(-141,40,1100);
                          turn(142.5,1700);
                          if(infrared.get_value())
                        {
                          if(delay)
                        {   while(1)
                          {
                            if(pros::millis()-before>43000) break;
                          }
                        }
                          final_shoot(52);
                        }
                          else
                          {
                            if(delay)
                            {
                            while(1)
                            {
                              if(pros::millis()-before>44200) break;
                            }
                            }
                          single_shoot_low();
                          }
                          now=pros::millis()-before;
                          master.clear(); pros::delay(50);
                          master.print(0,0,"time:%d Msec",now);
            break;
            case 3:
                    before=pros::millis();
                    move(1080,1,2500);
                    move(-1220,2,2100);
                    turn(-270,2200);
                    move(1350,100,1800);
                    if(delay)
                  {  while(1)
                    if(pros::millis()-before>12000) break; }
                    move(-2000,0,7000);
                    turn(270,2500);
                //    if(delay)
                  //  {
                  //  while(1)
                  //  if(pros::millis()-before>37500) break;}
                    control_move_final(1,1110,3400);
                    to_platform(1500);
                    move(-128,40,1000);
                    turn(-150,1600);
                    if(infrared.get_value())
                  {
                    if(delay)
                    {
                      while(1)
                    {
                      if(pros::millis()-before>42950) break;
                    }
                    }
                    final_shoot(52);
                  }
                    else
                    {
                      if(delay)
                      {
                      while(1)
                      {
                        if(pros::millis()-before>44200) break;
                      }
                      }
                    single_shoot_low();
                    }
                    now=pros::millis()-before;
                    master.clear(); pros::delay(50);
                    master.print(0,0,"time:%d Msec",now);
                    break;
            case 4:
                              before=pros::millis();
                              move(1080,1,2500);
                              move(-1210,2,2100);
                              turn(272,2100);
                              move(1345,100,1800);
                              if(delay)
                              {while(1)
                              if(pros::millis()-before>12000) break;}

                              move(-1940,0,7000);
                              turn(-270,2500);
                            //  if(delay)
                            //  {while(1)
                            //  if(pros::millis()-before>36000) break;}
                              control_move_final(1,1155,3400);
                              to_platform(2000);
                              move(-141,40,1100);
                              turn(147,1700);
                              if(infrared.get_value())
                            {
                              if(delay)
                            {   while(1)
                              {
                                if(pros::millis()-before>43000) break;
                              }
                            }
                              final_shoot(45);
                            }
                              else
                              {
                                if(delay)
                                {
                                while(1)
                                {
                                  if(pros::millis()-before>44200) break;
                                }
                                }
                              single_shoot_low();
                              }
                              now=pros::millis()-before;
                              master.clear(); pros::delay(50);
                              master.print(0,0,"time:%d Msec",now);
                break;
  }

}
