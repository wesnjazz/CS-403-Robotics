/*************************************************************************/
/* File: project2.c                                                      */
/* Description: Kinematic function for Arms                              */
/*                 fwd_arm_kinematics() - maps joint angles in the arm   */
/*                    into endpoint coordinates in the base frame        */
/*                 inv_arm_kinematics() - maps endpoint position for the */
/*                    specified arm in base frame into two possible joint*/
/*                    angle configurations and selects one of the two    */
/*                    writes arm cspace setpoints                        */
/* Date:        1-2015                                                 */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "matrix_math.h"

/*************************************************************************/
/*** PROJECT #2 - FORWARD KINEMATICS: MAP (THETA1, THETA2) TO (X,Y)_B  ***/
/***              INVERSE KINEMATICS: MAP (X,Y)_B TO (THETA1, THETA2)  ***/
/*************************************************************************/
// void fwd_arm_kinematics(roger, limb, x, y)
// Robot * roger;
// int limb;

FILE* fp_p2 = NULL;
int log_to_file_p2 = 0;
int file_created_p2 = 0;

void fwd_arm_kinematics(theta1, theta2, x, y)
double theta1, theta2;
double *x, *y;
{ 
  // double theta1 = roger->arm_theta[limb][0];
  // double theta2 = roger->arm_theta[limb][1];
  *x = L_ARM1*cos(theta1) + L_ARM2*cos(theta1+theta2);
  *y = L_ARM1*sin(theta1) + L_ARM2*sin(theta1+theta2);
}

void fwd_arm_kinematics_Wframe(roger, theta1, theta2, x, y)
Robot* roger;
double theta1, theta2;
double *x, *y;
{ 
  // double xB = L_ARM1*cos(theta1) + L_ARM2*cos(theta1+theta2);
  // double yB = L_ARM1*sin(theta1) + L_ARM2*sin(theta1+theta2);

  double pos_b[4];
  pos_b[X] = L_ARM1*cos(theta1) + L_ARM2*cos(theta1+theta2);
  pos_b[Y] = L_ARM1*sin(theta1) + L_ARM2*sin(theta1+theta2);
  pos_b[2] = 0.0;
  pos_b[3] = 1.0;
    // convert into world frame
  double pos_w[4];
  double wTb[4][4];
  construct_wTb(roger->base_position, wTb);
  matrix_mult(4, 4, wTb, 1, pos_b, pos_w);
  *x = pos_w[X];
  *y = pos_w[Y];
}

int inv_arm_kinematics_savetotheta(roger, limb, x, y, theta1, theta2)
Robot * roger;
int limb;
double x, y;
double *theta1, *theta2;
{
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;

  // input (x,y) is in world frame coordinates - map it into the base frame
  // r_w = wTb * r_b        // position vector r in World frame = wTb * r in Base frame
  // r_b = wTb^(-1) * r_w   // position vector r in Base frame = inverse of wTb * r in World frame
  // r_b = bTw * r_w        // position vector r in Base frame = bTw * r in World frame
  construct_wTb(roger->base_position, wTb);
  HT_invert(wTb,bTw);

  ref_w[0] = x;
  ref_w[1] = y;
  ref_w[2] = 0.0;
  ref_w[3] = 1.0;

  matrix_mult(4, 4, bTw, 1, ref_w, ref_b);  // ref_b = bTw * ref_w
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;

  r2 = (ref_b[X] * ref_b[X]) + (ref_b[Y] * ref_b[Y]);
  c2 = ( r2 -  (L_ARM1 * L_ARM1) - (L_ARM2 * L_ARM2)) / 
        ( 2 * L_ARM1 * L_ARM2);

  if (c2 >= -1.0 && c2 <= 1.0) {
    /*** Reached **/
    // printf("Reach!\n");
    s2_plus = sqrt(1.0 - (c2 * c2));
    s2_minus = -s2_plus;
    theta2_plus = atan2(s2_plus, c2);
    theta2_minus = atan2(s2_minus, c2);
    k1 = L_ARM1 + (L_ARM2 * c2);
    k2_plus = L_ARM2 * s2_plus;
    k2_minus = L_ARM2 * s2_minus;
    alpha_plus = atan2(k2_plus, k1);
    alpha_minus = atan2(k2_minus, k1);
    theta1_plus = atan2(ref_b[Y], ref_b[X]) - alpha_plus;
    theta1_minus = atan2(ref_b[Y], ref_b[X]) - alpha_minus;

    // printf("s2+:%6.4f  s2-:%6.4f  k1:%6.4f  k2+:%6.4f  k2-:%6.4f  alp+:%6.4f  alp-:%6.4f  \n", s2_plus, s2_minus, k1, k2_plus, k2_minus, alpha_plus, alpha_minus);
    // printf("theta2+:%6.4f   theta2-:%6.4f   theta1+:%6.4f   theta1-:%6.4f\n", theta2_plus, theta2_minus, theta1_plus, theta1_minus);

    double pp = theta1_plus + theta2_plus;
    double pm = theta1_plus + theta2_minus;
    double mp = theta1_minus + theta2_plus;
    double mm = theta1_minus + theta2_minus;

    // printf("pp:%f  pm:%f  mp:%f  mm:%f\n", pp, pm, mp, mm);


    if (limb == LEFT) {
      *theta1 = theta1_minus;
      *theta2 = theta2_minus;
    } else {
      *theta1 = theta1_plus;
      *theta2 = theta2_plus;
    }
    return TRUE;
  }
  else {
    /*** Not Teached ***/
    // printf("Out of Reach\n");
    return FALSE;
  }
  // printf("WorldFrame: x = %6.4lf  y = %6.4lf\n", x, y);
}

int inv_arm_kinematics(roger, limb, x, y)
Robot * roger;
int limb;
double x, y;
{
  double wTb[4][4], bTw[4][4], ref_b[4], ref_w[4];

  double r2, c2, s2_plus, s2_minus, theta2_plus, theta2_minus;
  double k1, k2_plus, k2_minus, alpha_plus, alpha_minus;
  double theta1_plus, theta1_minus;

  // input (x,y) is in world frame coordinates - map it into the base frame
  // r_w = wTb * r_b        // position vector r in World frame = wTb * r in Base frame
  // r_b = wTb^(-1) * r_w   // position vector r in Base frame = inverse of wTb * r in World frame
  // r_b = bTw * r_w        // position vector r in Base frame = bTw * r in World frame
  construct_wTb(roger->base_position, wTb);
  HT_invert(wTb,bTw);

  ref_w[0] = x;
  ref_w[1] = y;
  ref_w[2] = 0.0;
  ref_w[3] = 1.0;

  matrix_mult(4, 4, bTw, 1, ref_w, ref_b);  // ref_b = bTw * ref_w
  if (limb==LEFT) ref_b[Y] -= ARM_OFFSET;
  else ref_b[Y] += ARM_OFFSET;

  // printf("Base Frame: x = %6.4lf  y = %6.4lf \n\n", ref_b[X], ref_b[Y]);
  r2 = (ref_b[X] * ref_b[X]) + (ref_b[Y] * ref_b[Y]);
  c2 = ( r2 -  (L_ARM1 * L_ARM1) - (L_ARM2 * L_ARM2)) / 
        ( 2 * L_ARM1 * L_ARM2);

  // printf("r2:%f  c2:%f\n", r2, c2);

  if (c2 >= -1.0 && c2 <= 1.0) {
    /*** Reached **/
    printf("Reach!\n");
    s2_plus = sqrt(1.0 - (c2 * c2));
    s2_minus = -s2_plus;
    theta2_plus = atan2(s2_plus, c2);
    theta2_minus = atan2(s2_minus, c2);
    k1 = L_ARM1 + (L_ARM2 * c2);
    k2_plus = L_ARM2 * s2_plus;
    k2_minus = L_ARM2 * s2_minus;
    alpha_plus = atan2(k2_plus, k1);
    alpha_minus = atan2(k2_minus, k1);
    theta1_plus = atan2(ref_b[Y], ref_b[X]) - alpha_plus;
    theta1_minus = atan2(ref_b[Y], ref_b[X]) - alpha_minus;

    // printf("s2+:%6.4f  s2-:%6.4f  k1:%6.4f  k2+:%6.4f  k2-:%6.4f  alp+:%6.4f  alp-:%6.4f  \n", s2_plus, s2_minus, k1, k2_plus, k2_minus, alpha_plus, alpha_minus);
    // printf("theta2+:%6.4f   theta2-:%6.4f   theta1+:%6.4f   theta1-:%6.4f\n", theta2_plus, theta2_minus, theta1_plus, theta1_minus);

    double pp = theta1_plus + theta2_plus;
    double pm = theta1_plus + theta2_minus;
    double mp = theta1_minus + theta2_plus;
    double mm = theta1_minus + theta2_minus;

    // printf("pp:%f  pm:%f  mp:%f  mm:%f\n", pp, pm, mp, mm);


    if (limb == LEFT) {
      roger->arm_setpoint[limb][0] = theta1_minus;
      roger->arm_setpoint[limb][1] = theta2_minus;
    } else {
      roger->arm_setpoint[limb][0] = theta1_plus;
      roger->arm_setpoint[limb][1] = theta2_plus;
    }
    return TRUE;
  }
  else {
    /*** Not Teached ***/
    printf("Out of Reach\n");
    return FALSE;
  }
  // printf("WorldFrame: x = %6.4lf  y = %6.4lf\n", x, y);
}

/*************************************************************************/
/* project development interface - called by GUI                         */
/*************************************************************************/
/* executed automatically when                                           */
/* control mode = PROJECT2; input mode = BALL INPUTS                     */
void project2_control(roger, time)
Robot * roger;
double time;
{ 
  // double pos_b[NARMS][4];
  // double pos_w[NARMS][4];

  double x_cur, y_cur;
  double x_set, y_set;
    // printf("%f\n", roger->ext_force[LEFT][0]);

  // fwd_arm_kinematics(roger->arm_setpoint[LEFT][0], roger->arm_setpoint[LEFT][1], &x_set, &y_set);
  // fwd_arm_kinematics(roger->arm_theta[LEFT][0], roger->arm_theta[LEFT][1], &x_cur, &y_cur);
  // double error = sqrt( (x_set-x_cur)*(x_set-x_cur) + (y_set-y_cur)*(y_set-y_cur) );

  // printf("setpoints:(%.3f, %.3f)  current:(%.3f, %.3f)  error:%.3f\n", 
  //   x_set, y_set, x_cur, y_cur, error);

  // if (log_to_file_p2 && file_created_p2) {
  //   fprintf(fp_p2, "%f,%f\n", time, error);
  // }
    

  // for(int limb=0; limb<NARMS; limb++) {
  //   fwd_arm_kinematics(roger, limb, &x, &y);
  //   fwd_arm_kinematics(roger, limb, &x_set, &y_set);
  //   if(limb==LEFT) {
  //     printf("LEFT ARM\t");
  //   } 
  //   else if (limb==RIGHT) {
  //     continue;
  //     printf("RIGHT ARM\t");
  //   }
  //   printf("set_point:(%.3f, %.3f)   (x,y): (%.3f, %.3f)\n",
  //     roger->arm_setpoint[limb][0], roger->arm_setpoint[limb][1], x, y);
  // }
}


void project2_reset(roger)
Robot* roger;
{ }

void project2_enter_params() 
{
  printf("Do you want to log errors into file?\n");
  printf("Yes:1 No:0\n"); fflush(stdout);
  scanf("%d", &log_to_file_p2);

  if (log_to_file_p2 == 1) {
    char file_prefix[] = "project2-ArmKinematics/data/CartesianError";
    char file_concat[200];

    int arm_intK_pre = 30;
    int arm_intK_dec = .0 * 100000;
    int arm_intB_pre = 6;
    int arm_intB_dec = .5 * 100000;

    sprintf(file_concat, "%s_K%d_%05d_B%d_%05d.txt", file_prefix, arm_intK_pre, arm_intK_dec, arm_intB_pre, arm_intB_dec);
    fp_p2 = fopen(file_concat, "w+");
    file_created_p2 = 1;
    if (file_created_p2) {
      fprintf(fp_p2, "time,arm_x_y_error\n");
    }
  }
}

void project2_visualize(roger)
Robot* roger;
{ }


