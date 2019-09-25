/*************************************************************************/
/* File:        project1.c                                               */
/* Description: PD control analogs of the biological motor units for     */
/*              every degree of freedom: eyes, arms, base rotate and     */
/*              base translate --- motor units execute every simulated   */
/*              millisecond and are never disengaged. Higher-level       */
/*              control applications submit sequences of setpoints to    */
/*              combinations of motorunits.                              */
/* Date:        1-2015                                                   */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"
#include <stdlib.h>

void update_setpoints();

/*************************************************************************/
/* PROJECT #1 - COMPLETE THE FOLLOWING CONTROL PROCEDURES                */
// gains for the PD controllers EYES
// EYES
// K: 1.0
// B: 0.0205
// double Kp_eye = 10.0;
// double Kd_eye = 0.076;
// double Kp_eye = 0.1;
// double Kd_eye = 0.0025;
double Kp_eye = 1.0;
double Kd_eye = 0.025;
// double Kp_eye = 3.0;
// double Kd_eye = 0.037;
// double Kp_eye = 4.0;
// double Kd_eye = 0.04;
// double Kp_eye = 5.0;
// double Kd_eye = 0.052;
// double Kp_eye = 2.0;
// double Kd_eye = 0.030;
// double Kd_eye = 0.0205;
double passive_Kd_eye = 0.001;
double ul = -1.0;
double ur = -1.0;

// ARMS
// K: 101.25
// B: 4.0 
// double Kp_arm =  101.25;
// double Kd_arm =  4.0;
// double Kp_arm =  80;  // Tie
// double Kd_arm =  10.0;
double Kp_arm =  60;
double Kd_arm =  8.0;
// double Kp_arm =  30;   // CHASETOUCH
// double Kd_arm =  6.5;
// double Kp_arm =  28;
// double Kd_arm =  6.3;
// double Kp_arm =  25;
// double Kd_arm =  6.3;
double passive_Kd_arm = 1.0;

// BASE TRANSLATION
// K: 300.00
// B: 40.0
// double Kp_base_trans = 300.0;
// double Kd_base_trans = 42.0; 
double Kp_base_trans = 100.0;
double Kd_base_trans = 27.0; 
// double Kp_base_trans = 95.0; // Tie
// double Kd_base_trans = 26.4; 
// double Kp_base_trans = 80.0;
// double Kd_base_trans = 16.0; 
// double Kp_base_trans = 50.0;  // CHASETOUCH
// double Kd_base_trans = 15.0; 
// double Kp_base_trans = 40.0;
// double Kd_base_trans = 14.5; 
// double Kp_base_trans = 30.0;
// double Kd_base_trans = 14.0; 
double passive_Kd_base_trans = 2.0;

// BASE ROTATION
// K: 330.00
// B: 20.0
// double Kp_base_rot =  200.0;
// double Kd_base_rot =  18;
double Kp_base_rot =  100.0;
double Kd_base_rot =  12;
// double Kp_base_rot =  80.0; // Tie
// double Kd_base_rot =  12.0;
// double Kp_base_rot =  75.0;
// double Kd_base_rot =  13;
// double Kp_base_rot =  65.0; // CHASETOUCH
// double Kd_base_rot =  12.5;
// double Kp_base_rot =  60.0;
// double Kd_base_rot =  13.0;
// double Kp_base_rot =  50.0;
// double Kd_base_rot =  13;
// double Kp_base_rot =  30.0;
// double Kd_base_rot =  14;
// double Kp_base_rot =  20.0;
// double Kd_base_rot =  7.5;
// double Kd_base_rot =  5.5;
double passive_Kd_base_rot = 1.0;
/*************************************************************************/

FILE *fp = NULL;
int file_created = 0;

/* PROJECT #1.1 - PD CONTROLLER FOR THE EYES                             */
/* setpoints are joint angle values in radians for the eyes              */
void PDController_eyes(roger, time)
Robot * roger;
double time;
{
  int i;
  double theta_error, theta_dot_error;

  for (i = 0; i < NEYES; i++) {
    theta_error = roger->eyes_setpoint[i] - roger->eye_theta[i];
    theta_dot_error = 0.0 - roger->eye_theta_dot[i];
    if (ACTUATE_EYES) {
      // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE EYE
      // USING YOUR GAINS
      // roger->eye_torque[i] = passive_Kd_eye*theta_dot_error;
      roger->eye_torque[i] = (Kp_eye * theta_error) + (Kd_eye * theta_dot_error);
      // if (i == 0) {
      // 	printf("Eye[%d]: \tTime:%f \tT_error:%f \tTdot_error:%f\n", i, time, theta_error, theta_dot_error);
      // 	if (file_created) fprintf(fp, "%f, %f, %f\n", time, theta_error, theta_dot_error);
      // }
    } 
    else roger->eye_torque[i] = passive_Kd_eye*theta_dot_error;
  }
  // printf("Time:%.3f EYE_Theta[L,R]:(%.3f,%.3f) EYE_Setpoint[L,R]:(%.3f,%.3f) \n", 
  // time, roger->eye_theta[LEFT], roger->eye_theta[RIGHT],
  //       roger->eyes_setpoint[LEFT], roger->eyes_setpoint[RIGHT]);

}

/* PROJECT #1.2 - PD CONTROLLER FOR THE ARMS                             */
/* setpoints - joint angles in radians for the shoulders and elbows      */
void PDController_arms(roger, time)
Robot * roger;
double time;
{
  int i, j;
  double theta_error, theta_dot_error;

  for (i=LEFT; i<=RIGHT; ++i) {
    for (j=0; j<NARM_JOINTS; ++j) {

      theta_error = roger->arm_setpoint[i][j] - roger->arm_theta[i][j];
      theta_dot_error = 0.0 - roger->arm_theta_dot[i][j];

      while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
      while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

      // tune kp_arm and kd_arm by changing their value using enter_params()
      if (ACTUATE_ARMS) {
      // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE ARM
      // USING YOUR GAINS
		// roger->arm_torque[i][j] = passive_Kd_arm * theta_dot_error;
      	roger->arm_torque[i][j] = (Kp_arm * theta_error) + (Kd_arm * theta_dot_error);
	      // if (i == 0 && j == 0) {
	      // 	// printf("Arm[%d]: T:%f   T_er1:%f   Td_er1:%f", i, time, theta_error, theta_dot_error);
	      // 	if (file_created) fprintf(fp, "%f, %f, %f,", time, theta_error, theta_dot_error);
	      // }
	      // if (i == 0 && j == 1) {
	      // 	// printf("   T_er2:%f   Td_er2:%f", theta_error, theta_dot_error);
	      // 	if (file_created) fprintf(fp, "%f, %f\n", theta_error, theta_dot_error);
	      // }

      }
      else {
	roger->arm_torque[i][j] = passive_Kd_arm * theta_dot_error;
      }
    }
  }
}

/* Base PD controller, Cartesian reference */
double PDBase_translate(roger, time) 
Robot * roger;
double time;
{ 
  double Fx, error[2], trans_error, trans_vel;

  error[X] = roger->base_setpoint[X] - roger->base_position[X];
  error[Y] = roger->base_setpoint[Y] - roger->base_position[Y];

  trans_error = error[X]*cos(roger->base_position[THETA]) +
    error[Y]*sin(roger->base_position[THETA]);

  trans_vel = roger->base_velocity[X]*cos(roger->base_position[THETA]) +
    roger->base_velocity[Y]*sin(roger->base_position[THETA]);

  if (ACTUATE_BASE) {
    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
    // USING YOUR GAINS
    // Fx = - passive_Kd_base_trans * trans_vel;
    // Fx = 0;
    Fx = (Kp_base_trans * trans_error) - (Kd_base_trans * trans_vel);
  }
  else {
    Fx = - passive_Kd_base_trans * trans_vel;
  }
  return(Fx);
}

/* Base PD controller, Cartesian reference */
double PDBase_rotate(roger, time) 
Robot * roger;
double time;
{
  double Mz, theta_error, theta_dot_error;

  theta_error = roger->base_setpoint[THETA] - roger->base_position[THETA];
  theta_dot_error = 0.0 - roger->base_velocity[THETA];
  while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
  while (theta_error < -M_PI) theta_error += 2.0 * M_PI;

  if (ACTUATE_BASE) {
    // REPLACE THE FOLLOWING LINE WITH THE PD CONTROLLER FOR THE BASE
    // USING YOUR GAINS
    // Mz = passive_Kd_base_rot * theta_dot_error;
    Mz = (Kp_base_rot * theta_error) + (Kd_base_rot * theta_dot_error);
  }
  else {
    Mz = passive_Kd_base_rot * theta_dot_error;
  }
  
  return(Mz);
}

/* PROJECT #1.3 - PD CONTROLLER FOR THE BASE                             */
/* setpoints - (xy) location for translation heading in radians          */

/*   the base differential drive Jacobian:                               */
/*    |tau_l|     |Fx|      |  x_dot  |     |theta_dot_left |            */
/*    |     |= JT |  |      |         | = J |               |            */
/*    |tau_r|     |Mz|      |theta_dot|     |theta_dot_right|            */

double baseJT[2][2] = 
  {{(1.0/2.0), -(1.0/(2.0*R_AXLE))}, {(1.0/2.0), (1.0/(2.0*R_AXLE))}};

void PDController_base(roger, time)
Robot * roger;
double time;
{ 
  double Fx, Mz, PDBase_translate(), PDBase_rotate();

  //  Fx = PDBase_translate(roger,time); // translate along current heading
  //  Mz = 0.0;

  //  Fx = 0.0;                          // rotate in current footprint
  //  Mz = PDBase_rotate(roger,time);

  Fx = PDBase_translate(roger,time);     // translate and rotate 
  Mz = PDBase_rotate(roger,time);

  // integrated wheel torque control
  roger->wheel_torque[LEFT] = baseJT[0][0]*Fx + baseJT[0][1]*Mz;
  roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz;
  // roger->wheel_torque[RIGHT] = baseJT[1][0]*Fx + baseJT[1][1]*Mz;

  /*** FILE ***/
  // if (file_created) {
  //   fprintf(fp, "%f, %f, %f, %f\n", 
  //               time, 
  //               roger->base_setpoint[X] - roger->base_position[X],
  //               -roger->base_setpoint[Y] + roger->base_position[Y],
  //               roger->base_setpoint[THETA] - roger->base_position[THETA]);
  // }

  // printf("Time:%.3f Ref_W(%.3f,%.3f,%.3f) Act_W(%.3f,%.3f,%.3f) \n", 
  //   time, roger->base_setpoint[X], roger->base_setpoint[Y], roger->base_setpoint[THETA],
  //         roger->base_position[X], roger->base_position[Y], roger->base_position[THETA]);

  /*** print BASE_SETPOINT and BASE_POSITION ***/
  // printf("Time:%.3f Ref_W(%.3f,%.3f,%.3f) Act_W(%.3f,%.3f,%.3f) \n", 
  //   time, roger->base_setpoint[X], roger->base_setpoint[Y], roger->base_setpoint[THETA],
  //         roger->base_position[X], roger->base_position[Y], roger->base_position[THETA]);
}

/*************************************************************************/
/*       THE SIMULATOR EXECUTES control_roger() EVERY CONTROL CYCLE      */
/*                        *** DO NOT ALTER ***                           */
/*************************************************************************/

#define MIN_X              -5.0//-4.5 //-2.0
#define MIN_Y              -2.0
#define MAX_X              5.0//4.5 //2.0
#define MAX_Y              2.0

double setpoint_filter(Robot* roger)
{
  double offset = 0.1;
  double roger_reach = L_ARM1 + L_ARM2 + 2.0*R_WHEEL + R_BASE + offset;
  double x_max = 0.0 - roger_reach;
  double x_min = MIN_X + roger_reach;
  double y_max = MAX_Y - roger_reach;
  double y_min = MIN_Y + roger_reach;
  // check base setpoint
  if (roger->base_setpoint[X] > x_max) {
    roger->base_setpoint[X] = x_max;
  }
  if (roger->base_setpoint[X] < x_min) {
    roger->base_setpoint[X] = x_min;
  }
  if (roger->base_setpoint[Y] > y_max) {
    roger->base_setpoint[Y] = y_max;
  }
  if (roger->base_setpoint[Y] < y_min) {
    roger->base_setpoint[Y] = y_min;
  }
  // check arm setpoint
//   double arm_L_x = 0.0;
//   double arm_L_y = 0.0;
//   double arm_R_x = 0.0;
//   double arm_R_y = 0.0;
//   double arm_L_theta1 = 0.0;
//   double arm_L_theta2 = 0.0;
//   double arm_R_theta1 = 0.0;
//   double arm_R_theta2 = 0.0;
//   // get estimated x, y of both arms
//   fwd_arm_kinematics_Wframe(roger->arm_setpoint[LEFT][0], roger->arm_setpoint[LEFT][1], &arm_L_x, &arm_L_y);
//   fwd_arm_kinematics_Wframe(roger->arm_setpoint[RIGHT][0], roger->arm_setpoint[RIGHT][1], &arm_R_x, &arm_R_y);
//   if (arm_L_x > x_max) {
//     inv_arm_kinematics_savetotheta(roger, LEFT, )
//   }


// int inv_arm_kinematics_savetotheta(roger, limb, x, y, theta1, theta2)
// Robot * roger;
// int limb;
// double x, y;
// double *theta1, *theta2;
//   if (roger->arm_setpoint[])
  // check eye setpoint




  // if (x>0.0 && x>x_max) {
  //   return x_max;
  // }
  // if (x<0.0 && x<x_min) {
  //   return x_min;
  // }
  // return x;
}

double position_i = -1.5;
int first = 1;
void control_roger(roger, time)
Robot * roger;
double time;
{

  // Strecthing Left Arm at the beginning
  // if (first) {
    // roger->arm_setpoint[0][0] = 0;
    // roger->arm_setpoint[0][1] = 0;
    // first = 0;
  // }
  // else {

    // if ( (int)(fmod(time*1000, 1000.0)) == 0) {
    //   // printf("time: %f\n", time);
    //   roger->base_setpoint[X] = 1.0;
    //   roger->base_setpoint[Y] = 1.0;
    //   roger->base_setpoint[THE] = 1.0;
    //   position_i += 1.5;
    // }

    /** 0.75m foward **/
    // roger->base_setpoint[X] = 0.0;
    // roger->base_setpoint[Y] = -0.75;
    // roger->base_setpoint[THETA] = 0.0;
    // /** 0.75m left **/
    // roger->base_setpoint[X] = 0.75;
    // roger->base_setpoint[Y] = 0.0;
    // roger->base_setpoint[THETA] = 0.0;


    update_setpoints(roger); // check_GUI_inputs(roger)
    // roger->base_setpoint[X] = -roger->base_setpoint[X];
    // roger->base_setpoint[Y] = -roger->base_setpoint[Y];
    // roger->base_setpoint[THETA] = -roger->base_setpoint[THETA];
    // setpoint_filter(roger);
    // roger->button_event = FALSE;
    /*** BALL TRACKER ***/
    // int ball_seen = average_red_pixel(roger, &ul, &ur);
    // average_red_pixel(roger, &ul, &ur);
    // printf("time:%f ul:%f  ur:%f\n", time, ul, ur);
    // printf("ball_seen: %d\n", ball_seen);
    // if (ball_seen == 1) {
    // if (ul >= 0.0 || ur >= 0.0) {
    //   if (ul >= 0.0) {
    //     printf("BALL SEEN LEFT LEFT LEFT!!!!\n");
    //     double eye_theta_error_l = atan2((ul - 63.5), 64);
    //     roger->eyes_setpoint[0] = roger->eye_theta[0] + eye_theta_error_l;
    //   }
    //   if (ur >= 0.0) {
    //     printf("BALL SEEN \t\t\tRIGHT RIGHT RIGHT!!!!\n");
    //     double eye_theta_error_r = atan2((ur - 63.5), 64);
    //     roger->eyes_setpoint[1] = roger->eye_theta[1] + eye_theta_error_r;
    //   }
    // } else {
    //   roger->eyes_setpoint[0] = 0;
    //   roger->eyes_setpoint[1] = 0;
    // }

      // REFERENCE VALUE
      // double base_setpoint[3]; /* desired world frame base position (x,y,theta) */
        // double arm_setpoint[NARMS][NARM_JOINTS];      /* desired arm joint angles */
      // double eyes_setpoint[NEYES];                    /* desired eye pan angle  */

  /*** EYE ***/
  // if ( (int)(fmod(time*1000, 1000.0)) == 0) {
  //  printf("time: %f\n", time);
  //  roger->eyes_setpoint[0] = position_i;
  //  position_i += 1.5;
  // }

  /*** ARM ***/
  // if ( time != 0 && (int)(fmod(time*1000, 2000.0)) == 0) {
    // printf("time: %f\n", time);
    // roger->arm_setpoint[0][0] = M_PI / 2.0;
    // roger->arm_setpoint[0][1] = M_PI / 2.0;
  // }

  // double range = (3.141592);
  // double div = RAND_MAX / range;
  // double randD = -1.570796; + (rand()/div);
  // srand(time*1000000);
  // printf("%f\n", rand());

  // turn setpoint references into torques
  PDController_eyes(roger, time);
  PDController_arms(roger, time);
  PDController_base(roger,time);
  // printf("control_roger() %f\n", time);

  // printf("Eye angle: (%.4f, %.4f)\n", roger->eye_theta[LEFT], roger->eye_theta[RIGHT]);
  // printf("Arm angle: L:(%.4f, %.4f)  R:(%.4f, %.4f)\n", roger->arm_theta[LEFT][0], roger->arm_theta[LEFT][1], roger->arm_theta[RIGHT][0], roger->arm_theta[RIGHT][1]);

}

/*************************************************************************/
void project1_reset(roger)
Robot* roger;
{ }

/*************************************************************************/
/* prompt for and read user customized input values                      */
/*************************************************************************/
// void project1_enter_params(roger)
// Robot* roger;
void project1_enter_params()
{
  // put anything in here that you would like to change at run-time
  // without re-compiling user project codes
  
  /*** EYE ***/
  /*** EYE GAIN ***/
  // printf("EYE: K=%6.4lf  B=%6.4lf\n", Kp_eye, Kd_eye);
  // printf("EYE: enter 'K B'\n"); fflush(stdout);
  // scanf("%lf %lf", &Kp_eye, &Kd_eye);
  /*** EYE FILE ***/
  // char file_eye_prefix[] = "project1-MotorUnits/eye";
  // char file_eye_concat[80];
  // int eye_intK_pre = (int)(Kp_eye);
  // int eye_intK_dec = (int)((Kp_eye - eye_intK_pre) * 100000);
  // int eye_intB_pre = (int)(Kd_eye);
  // int eye_intB_dec = (int)((Kd_eye - eye_intB_pre) * 100000);
  // sprintf(file_eye_concat, "%s_K%d_%05d_B%d_%05d.txt", file_eye_prefix, eye_intK_pre, eye_intK_dec, eye_intB_pre, eye_intB_dec);
  // fp = fopen(file_eye_concat, "w+");
  // file_created = 1;
  // fprintf(fp, "time, theta_error, theta_dot_error\n");

  /*** ARM ***/
  // Kp_eye = 1.0;
  // Kd_eye = 0.01789;
  // printf("ARM: q1=%6.4lf q2=%6.4lf\n", roger->arm_setpoint[0][0], roger->arm_setpoint[0][1]);
  // printf("ARM: enter q1 q2\n"); fflush(stdout);
  // scanf("%lf %lf", roger->arm_setpoint[0][0], roger->arm_setpoint[0][1]);
  /*** ARM GAIN ***/
  printf("ARM: K=%6.4lf  B=%6.4lf\n", Kp_arm, Kd_arm);
  printf("ARM: enter 'K B'\n"); fflush(stdout);
  scanf("%lf %lf", &Kp_arm, &Kd_arm);
  /*** ARM FILE ***/
  // char file_arm_prefix[] = "project1-MotorUnits/arm";
  // char file_arm_concat[80];
  // int arm_intK_pre = (int)(Kp_arm);
  // int arm_intK_dec = (int)((Kp_arm - arm_intK_pre) * 100000);
  // int arm_intB_pre = (int)(Kd_arm);
  // int arm_intB_dec = (int)((Kd_arm - arm_intB_pre) * 100000);
  // sprintf(file_arm_concat, "%s_K%d_%05d_B%d_%05d.txt", file_arm_prefix, arm_intK_pre, arm_intK_dec, arm_intB_pre, arm_intB_dec);
  // fp = fopen(file_arm_concat, "w+");
  // file_created = 1;
  // fprintf(fp, "time,t_er_arm1,t_dot_er_arm1,t_er_arm2,t_dot_er_arm2\n");

  /*** BASE Translate ***/
  /*** BASE Translate GAIN ***/
  // printf("Base Translate: K=%6.4lf  B=%6.4lf\n", Kp_base_trans, Kd_base_trans);
  // printf("Base Translate: enter 'K B'\n"); fflush(stdout);
  // scanf("%lf %lf", &Kp_base_trans, &Kd_base_trans);

  /*** BASE Rotate ***/
  /*** BASE Rotate GAIN ***/
  // printf("Base Rotate: K=%6.4lf  B=%6.4lf\n", Kp_base_rot, Kd_base_rot);
  // printf("Base Rotate: enter 'K B'\n"); fflush(stdout);
  // scanf("%lf %lf", &Kp_base_rot, &Kd_base_rot);

  /*** BASE Trans & Rotate FILE ***/
  // char file_base_prefix[] = "project1-MotorUnits/data/base_PURE_ROTATE_";
  // char file_base_concat[160];
  // int base_trans_intK_pre = (int)(Kp_base_trans);
  // int base_trans_intK_dec = (int)((Kp_base_trans - base_trans_intK_pre) * 100000);
  // int base_trans_intB_pre = (int)(Kd_base_trans);
  // int base_trans_intB_dec = (int)((Kd_base_trans - base_trans_intB_pre) * 100000);
  // int base_rot_intK_pre = (int)(Kp_base_rot);
  // int base_rot_intK_dec = (int)((Kp_base_rot - base_rot_intK_pre) * 100000);
  // int base_rot_intB_pre = (int)(Kd_base_rot);
  // int base_rot_intB_dec = (int)((Kd_base_rot - base_rot_intB_pre) * 100000);
  // sprintf(file_base_concat, "%s_K%d_%05d_B%d_%05d_rot_K%d_%05d_B%d_%05d.txt", 
  //   file_base_prefix, base_trans_intK_pre, base_trans_intK_dec, base_trans_intB_pre, base_trans_intB_dec,
  //                     base_rot_intK_pre, base_rot_intK_dec, base_rot_intB_pre, base_rot_intB_dec);
  // fp = fopen(file_base_concat, "w+");
  // file_created = 1;
  // fprintf(fp, "time,x_error,y_error,theta_error\n");


}

/*************************************************************************/
// function called when the 'visualize' button on the gui is pressed            
void project1_visualize(roger)
Robot* roger;
{ }

