/*************************************************************************/
/* File:        project9.c                                               */
/* Description: User project #9 - PONG                                   */
/* Date:        12-2014                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

#define RS_CHASE 0
#define RS_AIM 1
#define RS_TOUCH 2
#define RS_PUNCH 3
#define RS_RETREAT 4

// TODO:
// AIM - to attack forward not to my side
// RETREAT - if ball is in my side, then start chasing
// 					1. calculate angle from home position to current position
//					2. rotate towards opposite of home position
//					3. set x and y towards home so that Roger move backwards
//					4. Adjust heading to home[THETA] after arriving.
// BLOCK - never allow the ball to cross home position.
//
// "mode" variable - OFFENSE or DEFENSE
//						after PUNCH() is CONVERGED, then it is DEFFENSE mode.
//						after BLOCK() is CONVERGED, then it is OFFENSE mode.

typedef struct _setpoints {    /* DO NOT ALTER */
  double base[3];             /* (x,y,theta) of the base in world frame */
  double arm[NARMS][NARM_JOINTS]; /* arm joint angles */
  double eyes[NEYES];         /* eye verge angle  relative to base frame */
} SetPoints;

SetPoint recommended_setpoints[5];
Observation obs;

int P9_VERBOSE = TRUE;
int state_chase = NO_REFERENCE;
int state_aim = NO_REFERENCE;
int state_punch = NO_REFERENCE;
int state_retreat = NO_REFERENCE;
int retreat_mode = FALSE;
int ball_LorR = LEFT;
int tiktok_interval = 200;
double base_err_previous = 0.0;
double home[3];
int play_side = LEFT;

int RogerBaseHome(Robot* roger, double time)
{
	roger->base_setpoint[X] = recommended_setpoints[RS_RETREAT].base[X];
	roger->base_setpoint[Y] = recommended_setpoints[RS_RETREAT].base[Y];
	return TRUE;
}

int RogerBaseStay(Robot* roger, double time)
{
	roger->base_setpoint[X] = roger->base_position[X];
	roger->base_setpoint[Y] = roger->base_position[Y];
	return TRUE;
}

int RogerBaseMove(Robot* roger, double time)
{
	roger->base_setpoint[X] = recommended_setpoints[RS_CHASE].base[X];
	roger->base_setpoint[Y] = recommended_setpoints[RS_CHASE].base[Y];
	roger->base_setpoint[THETA] = recommended_setpoints[RS_CHASE].base[THETA];
	return TRUE;
}

int RogerArmHome(Robot* roger, double time)
{
	double home[2][2] = {{2.0, -2.5}, {-2.0, 2.5}};
	roger->arm_setpoint[LEFT][0] = home[0][0];
	roger->arm_setpoint[LEFT][1] = home[0][1];
	roger->arm_setpoint[RIGHT][0] = home[1][0];
	roger->arm_setpoint[RIGHT][1] = home[1][1];

	return TRUE;
}

int RogerArmStay(Robot* roger, double time)
{
	roger->arm_setpoint[LEFT][0] = roger->arm_theta[LEFT][0];
	roger->arm_setpoint[LEFT][1] = roger->arm_theta[LEFT][1];
	roger->arm_setpoint[RIGHT][0] = roger->arm_theta[RIGHT][0];
	roger->arm_setpoint[RIGHT][1] = roger->arm_theta[RIGHT][1];
	return TRUE;
}

int RogerArmMove(Robot* roger, double time)
{
		roger->arm_setpoint[LEFT][0] = recommended_setpoints[RS_PUNCH].arm[LEFT][0];
		roger->arm_setpoint[LEFT][1] = recommended_setpoints[RS_PUNCH].arm[LEFT][1];
		roger->arm_setpoint[RIGHT][0] = recommended_setpoints[RS_PUNCH].arm[RIGHT][0];
		roger->arm_setpoint[RIGHT][1] = recommended_setpoints[RS_PUNCH].arm[RIGHT][1];
	return TRUE;
}

int RogerArmAim(Robot* roger, double time)
{
	roger->arm_setpoint[LEFT][0] = recommended_setpoints[RS_AIM].arm[LEFT][0];
	roger->arm_setpoint[LEFT][1] = recommended_setpoints[RS_AIM].arm[LEFT][1];
	roger->arm_setpoint[RIGHT][0] = recommended_setpoints[RS_AIM].arm[RIGHT][0];
	roger->arm_setpoint[RIGHT][1] = recommended_setpoints[RS_AIM].arm[RIGHT][1];
	return TRUE;
}

int RogerArmTouch(Robot* roger, double time)
{
	roger->arm_setpoint[LEFT][0] = recommended_setpoints[RS_TOUCH].arm[LEFT][0];
	roger->arm_setpoint[LEFT][1] = recommended_setpoints[RS_TOUCH].arm[LEFT][1];
	roger->arm_setpoint[RIGHT][0] = recommended_setpoints[RS_TOUCH].arm[RIGHT][0];
	roger->arm_setpoint[RIGHT][1] = recommended_setpoints[RS_TOUCH].arm[RIGHT][1];
	return TRUE;
}

int RogerArmPunch(Robot* roger, double time)
{
	roger->arm_setpoint[LEFT][0] = recommended_setpoints[RS_PUNCH].arm[LEFT][0];
	roger->arm_setpoint[LEFT][1] = recommended_setpoints[RS_PUNCH].arm[LEFT][1];
	roger->arm_setpoint[RIGHT][0] = recommended_setpoints[RS_PUNCH].arm[RIGHT][0];
	roger->arm_setpoint[RIGHT][1] = recommended_setpoints[RS_PUNCH].arm[RIGHT][1];
	return TRUE;
}

int STATE_RESET() {
	state_chase = NO_REFERENCE;
	state_aim = NO_REFERENCE;
	state_punch = NO_REFERENCE;
}

int STATE_CONVERGED() {
	state_chase = CONVERGED;
	state_aim = CONVERGED;
	state_punch = CONVERGED;
}


#ifndef BALL_RND
#define BALL_RND	-0.0
#endif

#define NBINS_BPS 30

typedef struct _ballpos {
	double x[NBINS_BPS];
	double y[NBINS_BPS];
	double time[NBINS_BPS];
	int head;
	int tail;
	int size;
	int cnt;
	double vel_x;
	double vel_y;
} BallPos;

int init_BallPos(BallPos* bps) {
	for (int i=0; i<NBINS_BPS; i++) {
		bps->x[i] = BALL_RND;
		bps->y[i] = BALL_RND;
		bps->time[i] = BALL_RND;
	}
	bps->head = 0;
	bps->tail = 0;
	bps->size = NBINS_BPS;
	bps->cnt = 0;
	bps->vel_x = 0.0;
	bps->vel_y = 0.0;
}

double prev_x;
double prev_y;
int record_BallPos(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps) 
{
	int head = bps->head;
	int tail = bps->tail;
	int size = bps->size;
	int cnt = bps->cnt;

	double x = obs->pos[X];
	double y = obs->pos[Y];
	if (time > 0.001) { // avoid record BALL_RND at the beginning
		if (x > MAX_X || x < MIN_X) { // if x is out of bounds, record trash value
			return NO_REFERENCE;
			x = BALL_RND;
		}
		if (y > MAX_Y || y < MIN_Y) { // if y is out of bounds, record trash value
			return NO_REFERENCE;
			y = BALL_RND;
		}
	}

	if (cnt == 0) { // if it is recording for the first time
		bps->head = 0;
		bps->tail = 0;
		bps->cnt = 1;
		bps->time[0] = time;
		bps->x[0] = x;
		bps->y[0] = y;
		prev_x = x;
		prev_y = y;
		return TRUE;
	}

	// get difference between current x,y and previous x,y
	// if difference bigger than epsilon, it is a noise, so record trash value
	double prev_x = (tail > 0) ? (bps->x[tail-1]) : (bps->x[NBINS_BPS-1]);
	double prev_y = (tail > 0) ? (bps->y[tail-1]) : (bps->y[NBINS_BPS-1]);
	double diff_x = fabs(x - prev_x);
	double diff_y = fabs(y - prev_y);
	double epsilon = 5.5;
	if (prev_x != BALL_RND && diff_x > epsilon) {
		x = prev_x;
	}
	if (prev_y != BALL_RND && diff_y > epsilon) {
		y = prev_y;
	}

	// record new x, y
	tail++;
	if (tail >= size) {
		tail = 0;
	}
	if (tail == head) {
		head++;
	}
	if (head >= size) {
		head = 0;
	}
	bps->time[tail] = time;
	bps->x[tail] = x;
	bps->y[tail] = y;
	bps->tail = tail;
	bps->head = head;
	prev_x = x;
	prev_y = y;
	if (cnt < NBINS_BPS) {
		bps->cnt++;
	}
	return TRUE;
}

int velocity_Ball(Robot* roger, double time, BallPos* bps, double* vel_x, double* vel_y)
{
	// if there is not enough data to calculate velocity
	int N = 25;
	if (bps->cnt < N) {
		return FALSE;
	}

	int head = bps->head;
	int tail = bps->tail;
	int size = bps->size;
	int cnt = bps->cnt;

	double epsilon = 100.0;
	double x_last = bps->x[tail];
	while (x_last > epsilon) {
		tail--;
		if (tail < 0) {
			tail = NBINS_BPS-1;
		}
		x_last = bps->x[tail];
	}
	double x_prev = bps->x[head];
	while (x_prev > epsilon) {
		head++;
		if (head >= NBINS_BPS) {
			head = 0;
		}
		x_prev = bps->x[head];
	}
	double y_last = bps->y[tail];
	while (y_last > epsilon) {
		tail--;
		if (tail < 0) {
			tail = NBINS_BPS-1;
		}
		y_last = bps->y[tail];
	}
	double y_prev = bps->y[head];
	while (y_prev > epsilon) {
		head++;
		if (head >= NBINS_BPS) {
			head = 0;
		}
		y_prev = bps->y[head];
	}

	*vel_x = (bps->x[tail] - bps->x[head]) / (NBINS_BPS);
	*vel_y = (bps->y[tail] - bps->y[head]) / (NBINS_BPS);
}

int CHASE2(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	if (state_punch == TRANSIENT || state_retreat == TRANSIENT) {
		return CONVERGED;
	}
	if (SEARCHTRACK(roger, time) == CONVERGED) {
		stereo_observation(roger, obs);
		recommended_setpoints[RS_CHASE].base[X] = obs->pos[X];
		recommended_setpoints[RS_CHASE].base[Y] = obs->pos[Y];

		double angle = atan2(obs->pos[Y] - roger->base_position[Y], obs->pos[X] - roger->base_position[X]);
		recommended_setpoints[RS_CHASE].base[THETA] = angle;
		double angle_err = roger->base_position[THETA] - angle;
		double angle_offset = 0.3;
		if (angle_err < 0) {ball_LorR = LEFT;}
		else {ball_LorR = RIGHT;}
		
		double X_err = obs->pos[X] - roger->base_position[X];
		double Y_err = obs->pos[Y] - roger->base_position[Y];
		double base_err = sqrt(X_err*X_err + Y_err*Y_err);
		double base_offset = 0.00;
		// double base_offset = 0.45;
		double epsilon = R_BALL + R_BASE + base_offset;
		// printf("%.4f- base_err:%.4f  epsilon:%.4f\n", time, base_err, epsilon);
		if (base_err > epsilon) { state_chase = TRANSIENT; return TRANSIENT; }
		else { state_chase = CONVERGED; return CONVERGED; }
	}
	// printf("state_chase = NO_REFERENCE  from CHASEforPUNCH()\n");
	state_chase = NO_REFERENCE;
	return NO_REFERENCE;
}

int TOUCH2(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	double x = obs->pos[X];
	double y = obs->pos[Y];

	double theta1_left = 0.0, theta2_left = 0.0, theta1_right = 0.0, theta2_right = 0.0;
	int left_reach = inv_arm_kinematics_savetotheta(roger, LEFT, x, y, &theta1_left, &theta2_left);
	int right_reach = inv_arm_kinematics_savetotheta(roger, RIGHT, x, y, &theta1_right, &theta2_right);

	if (left_reach == FALSE && right_reach == FALSE) { return NO_REFERENCE; }

	double epsilon = 0.1;

	int left_touched = FALSE;
	int right_touched = FALSE;
	
	if (left_reach == TRUE && right_reach == TRUE) {
		double left_joint_0_offset = 0.25;
		double left_joint_1_offset = 0.25;
		double right_joint_0_offset = -0.25;
		double right_joint_1_offset = -0.25;
		recommended_setpoints[RS_TOUCH].arm[LEFT][0] = theta1_left + left_joint_0_offset;
		recommended_setpoints[RS_TOUCH].arm[LEFT][1] = theta2_left + left_joint_1_offset;
		recommended_setpoints[RS_TOUCH].arm[RIGHT][0] = theta1_right + right_joint_0_offset;
		recommended_setpoints[RS_TOUCH].arm[RIGHT][1] = theta2_right + right_joint_1_offset;
	}

	double tolerance = 0.001;
	if (fabs(roger->ext_force[LEFT][0]) > tolerance && IsTactileRedBall(roger, time, LEFT) == TRUE) {
		left_touched = TRUE;
	}
	if (fabs(roger->ext_force[RIGHT][0]) > tolerance && IsTactileRedBall(roger, time, RIGHT) == TRUE) {
		right_touched = TRUE;
	}
	if (left_touched == TRUE && right_touched == TRUE) { return CONVERGED; } 
	else { return TRANSIENT; }
	
	return NO_REFERENCE;
}

int tiktok_punch_nohit = 0;
int tiktok_punch_hit = 0;
int left_touched = FALSE;
int right_touched = FALSE;
int PUNCH2(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	double punch_pose_L[NARMS][NARM_JOINTS] = { -1.6 , -1.5, -1.5, 0.0 };
	double punch_pose_R[NARMS][NARM_JOINTS] = { 1.5 , 0.0, 1.6, 1.0 };

	// printf("%3.3f- obs:(%3.3f,%3.3f) bps:(%3.3f,%3.3f)\n",
		// time, obs->pos[X], obs->pos[Y], bps->x, bps->y);

	if (state_retreat == TRANSIENT) {
		return CONVERGED;
	}
	
	double Larm_angle0, Larm_angle1, Rarm_angle0, Rarm_angle1;

	if (ball_LorR == LEFT) {
		inv_arm_kinematics_savetotheta(roger, LEFT, obs->pos[X], obs->pos[Y], &Larm_angle0, &Larm_angle1);
		// recommended_setpoints[RS_PUNCH].arm[LEFT][0] = Larm_angle0;
		// recommended_setpoints[RS_PUNCH].arm[LEFT][1] = Larm_angle1;
		recommended_setpoints[RS_PUNCH].arm[LEFT][0] = 0;
		recommended_setpoints[RS_PUNCH].arm[LEFT][1] = 0;
	} else if (ball_LorR == RIGHT) {
		inv_arm_kinematics_savetotheta(roger, RIGHT, obs->pos[X], obs->pos[Y], &Rarm_angle0, &Rarm_angle1);
		// recommended_setpoints[RS_PUNCH].arm[RIGHT][0] = Rarm_angle0;
		// recommended_setpoints[RS_PUNCH].arm[RIGHT][1] = Rarm_angle1;
		recommended_setpoints[RS_PUNCH].arm[RIGHT][0] = 0;
		recommended_setpoints[RS_PUNCH].arm[RIGHT][1] = 0;
	}

}

int ballseen = FALSE;
int RETREAT(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	if (state_punch == CONVERGED) {
		// printf("RETREAT!!\n");
		SEARCHTRACK(roger, time);
		recommended_setpoints[RS_RETREAT].base[X] = home[X];
		recommended_setpoints[RS_RETREAT].base[Y] = home[Y];

		double X_err = roger->base_setpoint[X] - roger->base_position[X];
		double Y_err = roger->base_setpoint[Y] - roger->base_position[Y];
		double epsilon = 0.01;
		// printf("%3.3f- base_set:(%3.3f,%3.3f)  base_pos:(%3.3f,%3.3f)  err:(%3.3f,%3.3f)\n", time, 
			// roger->base_setpoint[X], roger->base_setpoint[Y], roger->base_position[X], roger->base_position[Y], X_err, Y_err);
		if (fabs(X_err) <= epsilon) {
			state_retreat = CONVERGED; return CONVERGED; 
		}

		state_retreat = TRANSIENT;
		return TRANSIENT;
	}
	return NO_REFERENCE;
}

#define ROGER_MODE_DEFENSE 0
#define ROGER_MODE_OFFENSE 1

int Roger_mode = ROGER_MODE_OFFENSE;

int PONG(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	if (Roger_mode == ROGER_MODE_DEFENSE) {
		DEFENSE(roger, time, rsp, obs, bps);
	} else if (Roger_mode == ROGER_MODE_OFFENSE) {
		OFFENSE(roger, time, rsp, obs, bps);
	}
}

int OFFENSE(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	int internal_state[3];
	internal_state[0] = CHASE2(roger, time, rsp, obs, bps);
	internal_state[1] = TOUCH2(roger, time, rsp, obs, bps);
	internal_state[2] = RETREAT(roger, time, rsp, obs, bps);

	int state = internal_state[0]*9 + internal_state[1]*3 + internal_state[2];
	if (P9_VERBOSE) printf("%3.3f, Chase:%d  Punch:%d  Retreat:%d  Case:%d\n", time, internal_state[0], internal_state[1], internal_state[2], state);
	int return_state;
	switch (state) {
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			RogerBaseHome(roger, time);
			RogerArmHome(roger, time);
			return_state = NO_REFERENCE;
			break;
		case 9:
		case 10:
		case 11:
			RogerBaseMove(roger, time);
			RogerArmHome(roger, time);
			return_state = TRANSIENT;
			break;
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
		case 17:
			RogerBaseMove(roger, time);
			RogerArmTouch(roger, time);
			return_state = TRANSIENT;
			break;
		case 18:
		case 19:
		case 20:
			RogerBaseMove(roger, time);
			RogerArmHome(roger, time);
			return_state = TRANSIENT;
			break;
		case 21:
		case 22:
		case 23:
		case 24:
		case 25:
			RogerBaseMove(roger, time);
			RogerArmTouch(roger, time);
			return_state = TRANSIENT;
			break;
		case 26:
			RogerBaseMove(roger, time);
			RogerArmHome(roger, time);
			return_state = TRANSIENT;
			break;
		default:
			break;
	}

	
}

int DEFENSE(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
}



SetPoints rsp_global[8];
Observation obs_global;
BallPos bps_global;
int bps_created = FALSE;

/************************************************************************/
void project9_control(roger, time)
Robot *roger;
double time;
{
	if (time < 0.0010) {
		printf("Recording Home position at time %.4f\n", time);
		home[X] = roger->base_position[X];
		home[Y] = roger->base_position[Y];
		home[THETA] = roger->base_position[THETA];
		if (home[X] < 3.1 && home[X] > 2.9) { play_side = RIGHT; }
		else if (home[X] < -2.9 && home[X] > -3.1) {play_side = LEFT; } 	
	}

	if (!bps_created) {
		init_BallPos(&bps_global);
		bps_created = TRUE;
	}

	PONG(roger, time, rsp_global, &obs_global, &bps_global);
}

/************************************************************************/
void project9_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project9_enter_params() 
{
	Roger_mode = (Roger_mode == ROGER_MODE_DEFENSE) ? ROGER_MODE_OFFENSE : ROGER_MODE_DEFENSE;  
}

//function called when the 'visualize' button on the gui is pressed
void project9_visualize(roger)
Robot* roger;
{ }

