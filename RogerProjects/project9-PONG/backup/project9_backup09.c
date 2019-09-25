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
#define RS_PUNCH 2
#define RS_RETREAT 3

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

SetPoint recommended_setpoints[4];
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

int RogerArmAim(Robot* roger, double time)
{
	roger->arm_setpoint[LEFT][0] = recommended_setpoints[RS_AIM].arm[LEFT][0];
	roger->arm_setpoint[LEFT][1] = recommended_setpoints[RS_AIM].arm[LEFT][1];
	roger->arm_setpoint[RIGHT][0] = recommended_setpoints[RS_AIM].arm[RIGHT][0];
	roger->arm_setpoint[RIGHT][1] = recommended_setpoints[RS_AIM].arm[RIGHT][1];
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

int RogerBaseHome(Robot* roger, double time)
{
	roger->base_setpoint[X] = recommended_setpoints[RS_RETREAT].base[X];
	roger->base_setpoint[Y] = recommended_setpoints[RS_RETREAT].base[Y];
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

int CHASEforPUNCH(roger, time)
Robot* roger;
double time;
{
	if (state_punch == TRANSIENT || state_retreat == TRANSIENT) {
		return CONVERGED;
	}
	if (SEARCHTRACK(roger, time) == CONVERGED) {
		stereo_observation(roger, &obs);
		recommended_setpoints[RS_CHASE].base[X] = obs.pos[X];
		recommended_setpoints[RS_CHASE].base[Y] = obs.pos[Y];

		double X_err = obs.pos[X] - roger->base_position[X];
		double Y_err = obs.pos[Y] - roger->base_position[Y];
		double base_err = sqrt(X_err*X_err + Y_err*Y_err);
		double base_offset = -1.00;
		double epsilon = R_BALL + R_BASE + base_offset;
		// printf("%.4f- base_err:%.4f  epsilon:%.4f\n", time, base_err, epsilon);
		if (base_err > epsilon) { state_chase = TRANSIENT; return TRANSIENT; }
		else { state_chase = CONVERGED; return CONVERGED; }
	}
	// printf("state_chase = NO_REFERENCE  from CHASEforPUNCH()\n");
	state_chase = NO_REFERENCE;
	return NO_REFERENCE;
}

int tiktok_aim = 1;
int tiktok_aim_interval = 500;
int AIM(Robot* roger, double time)
{
	double aim_pose_L[NARMS][NARM_JOINTS] = { 2.0, -1.5, 1.3, 1.5 };
	double aim_pose_R[NARMS][NARM_JOINTS] = { -1.3, -1.5, -2.0, 1.5 };

	if (state_punch == TRANSIENT || state_retreat == TRANSIENT) {
		return CONVERGED;
	}
	if (state_chase == CONVERGED && state_punch != TRANSIENT) {
		tiktok_aim++;
		double angle = atan2(obs.pos[Y] - roger->base_position[Y], obs.pos[X] - roger->base_position[X]);
		double angle_err = roger->base_position[THETA] - angle;
		double angle_offset = 0.3;
		if (angle_err < 0) {ball_LorR = LEFT;}
		else {ball_LorR = RIGHT;}
		// printf("%3.3f- Roger_THETA:%3.3f  angleToBall:%3.3f  angle_err:%3.3f", time, roger->base_position[THETA], angle, angle_err);
		// if (ball_LorR == LEFT) printf("\tBall: Left");
		// else if (ball_LorR == RIGHT) printf("\tBall: Right");
		// printf("\n");

		if (ball_LorR == LEFT) {
			recommended_setpoints[RS_AIM].arm[LEFT][0] = aim_pose_L[LEFT][0];
			recommended_setpoints[RS_AIM].arm[LEFT][1] = aim_pose_L[LEFT][1];
			recommended_setpoints[RS_AIM].arm[RIGHT][0] = aim_pose_L[RIGHT][0];
			recommended_setpoints[RS_AIM].arm[RIGHT][1] = aim_pose_L[RIGHT][1];
		} else if (ball_LorR == RIGHT) {
			recommended_setpoints[RS_AIM].arm[LEFT][0] = aim_pose_R[LEFT][0];
			recommended_setpoints[RS_AIM].arm[LEFT][1] = aim_pose_R[LEFT][1];
			recommended_setpoints[RS_AIM].arm[RIGHT][0] = aim_pose_R[RIGHT][0];
			recommended_setpoints[RS_AIM].arm[RIGHT][1] = aim_pose_R[RIGHT][1];
		}

		if (tiktok_aim % tiktok_interval != 0) {
			return TRANSIENT;
		}
		tiktok_aim = 1;

		if (state_aim == TRANSIENT) {

		double tolerance = 0.2;
		double eL0 = fabs(recommended_setpoints[RS_AIM].arm[LEFT][0] - roger->arm_theta[LEFT][0]);
		double eL1 = fabs(recommended_setpoints[RS_AIM].arm[LEFT][1] - roger->arm_theta[LEFT][1]);
		double eR0 = fabs(recommended_setpoints[RS_AIM].arm[RIGHT][0] - roger->arm_theta[RIGHT][0]);
		double eR1 = fabs(recommended_setpoints[RS_AIM].arm[RIGHT][1] - roger->arm_theta[RIGHT][1]);
		// printf("%3.3f- RS-L:(%3.3f,%3.3f,)-R:(%3.3f,%3.3f)  Arm_set-L:(%3.3f,%3.3f)-R:(%3.3f,%3.3f)\n Arm_theta-L:(%3.3f,%3.3f)-R:(%3.3f,%3.3f)  error-L:(%3.3f,%3.3f)-R:(%3.3f,%3.3f)\n",
		// 	time,
		// 	recommended_setpoints[RS_AIM].arm[LEFT][0], recommended_setpoints[RS_AIM].arm[LEFT][1],
		// 	recommended_setpoints[RS_AIM].arm[RIGHT][0], recommended_setpoints[RS_AIM].arm[RIGHT][1],
		// 	roger->arm_setpoint[LEFT][0], roger->arm_setpoint[LEFT][1],
		// 	roger->arm_setpoint[RIGHT][0], roger->arm_setpoint[RIGHT][1],
		// 	roger->arm_theta[LEFT][0], roger->arm_theta[LEFT][1],
		// 	roger->arm_theta[RIGHT][0], roger->arm_theta[RIGHT][1],
		// 	eL0, eL1, eR0, eR1
		// 	);
			if (eL0 < tolerance && eL1 < tolerance && eR0 < tolerance && eR1 < tolerance) {
				state_aim = CONVERGED;
				return CONVERGED;
			}
		}
		state_aim = TRANSIENT;
		return TRANSIENT;
	} else {
		state_aim = NO_REFERENCE;
		return NO_REFERENCE;
	}
}

int tiktok_punch_nohit = 0;
int tiktok_punch_hit = 0;
int left_touched = FALSE;
int right_touched = FALSE;
int PUNCH(Robot* roger, double time)
{
	// double aim_pose_L[NARMS][NARM_JOINTS] = { 2.0, -1.5, 1.3, 1.5 };
	// double aim_pose_R[NARMS][NARM_JOINTS] = { -1.3, -1.5, -2.0, 1.5 };
	double punch_pose_L[NARMS][NARM_JOINTS] = { -1.6 , -1.5, -1.5, 0.0 };
	double punch_pose_R[NARMS][NARM_JOINTS] = { 1.5 , 0.0, 1.6, 1.0 };

	if (state_retreat == TRANSIENT) {
		return CONVERGED;
	}
	if (state_chase == CONVERGED && state_aim == CONVERGED && state_punch != CONVERGED) {
		tiktok_punch_nohit++;
		if (ball_LorR == LEFT) {
			roger->base_setpoint[THETA] = roger->base_setpoint[THETA] + M_PI;	// turn base
			recommended_setpoints[RS_PUNCH].arm[LEFT][0] = punch_pose_L[LEFT][0];
			recommended_setpoints[RS_PUNCH].arm[LEFT][1] = punch_pose_L[LEFT][1];
			recommended_setpoints[RS_PUNCH].arm[RIGHT][0] = punch_pose_L[RIGHT][0];
			recommended_setpoints[RS_PUNCH].arm[RIGHT][1] = punch_pose_L[RIGHT][1];
		} else if (ball_LorR == RIGHT) {
			roger->base_setpoint[THETA] = roger->base_setpoint[THETA] - M_PI; // turn base
			recommended_setpoints[RS_PUNCH].arm[LEFT][0] = punch_pose_R[LEFT][0];
			recommended_setpoints[RS_PUNCH].arm[LEFT][1] = punch_pose_R[LEFT][1];
			recommended_setpoints[RS_PUNCH].arm[RIGHT][0] = punch_pose_R[RIGHT][0];
			recommended_setpoints[RS_PUNCH].arm[RIGHT][1] = punch_pose_R[RIGHT][1];
		}

		double tolerance = 0.001;
		if (fabs(roger->ext_force[LEFT][0]) > tolerance && IsTactileRedBall(roger, time, LEFT) == TRUE) {
			// printf("%.4f- Left Touched!\n", time);
			left_touched = TRUE;
		}
		if (fabs(roger->ext_force[RIGHT][0]) > tolerance && IsTactileRedBall(roger, time, RIGHT) == TRUE) {
			// printf("%.4f- Right Touched!\n", time);
			right_touched = TRUE;
		}
		if (left_touched == TRUE || right_touched == TRUE) { 
			tiktok_punch_hit++;
			if (tiktok_punch_hit % tiktok_interval == 0) {
				printf("STATE_CONVERGED()!!!\n");
				STATE_CONVERGED();
				return CONVERGED; 
			}
		}

		if (tiktok_punch_nohit % tiktok_interval == 0 && left_touched == FALSE && right_touched == FALSE) {
			printf("No hit  STATE_RESET()\n");
			tiktok_punch_nohit = 0;
			STATE_RESET();
			return NO_REFERENCE;
		}

		state_punch = TRANSIENT;
		return TRANSIENT;
	}
	state_punch = NO_REFERENCE;
	return NO_REFERENCE;
}

int ballseen = FALSE;
int RETREAT2(Robot* roger, double time)
{
	if (state_punch == CONVERGED) {
		printf("RETREAT!!\n");
		SEARCHTRACK(roger, time);
		recommended_setpoints[RS_RETREAT].base[X] = home[X];
		recommended_setpoints[RS_RETREAT].base[Y] = home[Y];

		double X_err = roger->base_setpoint[X] - roger->base_position[X];
		double Y_err = roger->base_setpoint[Y] - roger->base_position[Y];
		double epsilon = 0.01;
		printf("%3.3f- base_set:(%3.3f,%3.3f)  base_pos:(%3.3f,%3.3f)  err:(%3.3f,%3.3f)\n", time, 
			roger->base_setpoint[X], roger->base_setpoint[Y], roger->base_position[X], roger->base_position[Y], X_err, Y_err);
		if (fabs(X_err) <= epsilon) {
			state_retreat = CONVERGED; return CONVERGED; 
		}

		state_retreat = TRANSIENT;
		return TRANSIENT;
	}
	return NO_REFERENCE;
}

int CHASEPUNCHRETREAT(Robot* roger, double time)
{
	int internal_state[4];
	internal_state[0] = CHASEforPUNCH(roger, time);
	internal_state[1] = AIM(roger, time);
	internal_state[2] = PUNCH(roger, time);
	internal_state[3] = RETREAT(roger, time);
	int state = internal_state[0]*27 + internal_state[1]*9 + internal_state[2]*3 + internal_state[3];
	// printf("%.4f- CHASE:%d  AIM:%d  PUNCH:%d  state:%d\n", time, internal_state[0], internal_state[1], internal_state[2], state);
	int return_state;
	// if (P9_VERBOSE) printf("%3.3f, Chase:%d  Aim:%d  Punch:%d  Retreat:%d  Case:%d\n", time, internal_state[0], internal_state[1], internal_state[2], internal_state[3], state);
	switch (state) {
		case 0:	case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8: case 9: case 10: 
		case 11: case 12: case 13: case 14: case 15: case 16: case 17: case 18: case 19: case 20:
		case 21: case 22: case 23: case 24: case 25: case 26: 
			RogerBaseStay(roger, time);
			RogerArmHome(roger, time);
			return_state = NO_REFERENCE;
			break;
		case 27: case 28: case 29: case 30:
		case 31: case 32: case 33: case 34: case 35: case 36: case 37: case 38: case 39: case 40:
		case 41: case 42: case 43: case 44: case 45: case 46: case 47: case 48: case 49: case 50:
		case 51: case 52: case 53:
			RogerBaseMove(roger, time);
			RogerArmHome(roger, time);
			return_state = TRANSIENT;
			break;
		case 54: case 55: case 56: case 57: case 58: case 59: case 60:
		case 61: case 62: case 63: case 64: case 65: case 66: case 67: case 68: case 69: case 70:
		case 71:
			RogerBaseStay(roger, time);
			RogerArmAim(roger, time);
			return_state = TRANSIENT;
			break;
		case 72: case 73: case 74: case 75: case 76: case 77:
			RogerBaseMove(roger, time);
			RogerArmPunch(roger, time);
			return_state = TRANSIENT;
			break;
		case 78: case 79: case 80:
			// RogerBaseStay(roger, time);
			RogerArmHome(roger, time);
			RogerBaseHome(roger, time);
			return_state = CONVERGED;
			break;
		default:
		break;
	}
	return return_state;
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
			// printf("ASFASDFASDFASDFASDFASDF  x:%3.3f\n", x);
			return NO_REFERENCE;
			x = BALL_RND;
		}
		if (y > MAX_Y || y < MIN_Y) { // if y is out of bounds, record trash value
			// printf("\t\t\t\tASFASDFASDFASDFASDFASDF  y:%3.3f\n", y);
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
	// printf("%3.3f- x,y:(%3.3f,%3.3f)  prev:(%3.3f,%3.3f)  head:%d tail:%d cnt:%d time[%d]:%3.3f \n",
			// time,  bps->x[bps->tail], bps->y[bps->tail], prev_x, prev_y, bps->head, bps->tail, bps->cnt, bps->tail, bps->time[bps->tail]);
	// double epsilon = 0.2;
	double epsilon = 5.5;
	if (prev_x != BALL_RND && diff_x > epsilon) {
		// printf("\t\t\t\t\t\t\tCut X!!!\n");
		x = prev_x;
		// x = (bps->x[tail] + bps->x[head]) / 2.0;
	}
	if (prev_y != BALL_RND && diff_y > epsilon) {
		// printf("\t\t\t\t\t\t\tCut Y!!!\n");
		y = prev_y;
		// x = (bps->y[tail] + bps->y[head]) / 2.0;
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
	// printf("%.4f- recording Ball Position: t:%.4f x:%.4f y:%.4f at head:%d  tail:%d  cnt:%d  size:%d\n",
					// time, bps->time[tail], bps->x[tail], bps->y[tail], bps->head, bps->tail, bps->cnt, bps->size);
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

/*** OBSERVEBALL()
	- Keep an eye on the ball on the Home position
	- Get red ball's velocity in every ms
	- OUTPUT: take the velocity over T ms and take the average
		- if one of the ball's position was close to the wall then start over

***/
int time_logged = FALSE;

int tiktok_observeball = 1;
int tiktok_observeball_interval = 100;
int to_record_velocity = TRUE;
int OBSERVEBALL(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{

	if (SEARCHTRACK(roger, time) == CONVERGED) {
		stereo_observation(roger, obs);


		double epsilon = 0.4;
		double x_err_MIN = fabs(obs->pos[X] - MIN_X);
		double x_err_MAX = fabs(obs->pos[X] - MAX_X);
		double y_err_MIN = fabs(obs->pos[Y] - MIN_Y);
		double y_err_MAX = fabs(obs->pos[Y] - MAX_Y);
		if (x_err_MIN < epsilon || x_err_MAX < epsilon || y_err_MIN < epsilon || y_err_MAX < epsilon) {
			init_BallPos(bps);
			to_record_velocity = TRUE;
			time_logged = FALSE;
			return NO_REFERENCE;
		}

		tiktok_observeball++;
		record_BallPos(roger, time, rsp, obs, bps);
		// printf("%3.3f- obs-pos:(%3.3f,%3.3f)  cov:(%3.3f,%3.3f,%3.3f,%3.3f) \n", 
				// time, obs->pos[X], obs->pos[Y], obs->cov[0][0], obs->cov[0][1], obs->cov[1][0], obs->cov[1][1]);

		int head = bps->head;
		int tail = bps->tail;
		int size = bps->size;
		int cnt = bps->cnt;

		// double vel_x_temp;
		// double vel_y_temp;
		if ( to_record_velocity == TRUE && bps->cnt > 0 && (tiktok_observeball % tiktok_observeball_interval == 0) ) {
			// vel_x_temp = (bps->x[tail] - bps->x[head]) / (NBINS_BPS+0.00000000001);
			// vel_y_temp = (bps->y[tail] - bps->y[head]) / (NBINS_BPS+0.00000000001);
			bps->vel_x = (bps->x[tail] - bps->x[head]) / (NBINS_BPS);
			bps->vel_y = (bps->y[tail] - bps->y[head]) / (NBINS_BPS);
			// to_record_velocity = FALSE;
			tiktok_observeball = 1;
			return CONVERGED;
		}
		// printf("%3.3f- velocity:(%3.3f,%3.3f)\n", time, vel_x_temp, vel_y_temp);
		return TRANSIENT;
	}
}

/*** PREDICTPATH()
	- Calculate ball's x, y position after t time
***/
double t = 0.0;
int PREDICTPATH(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	// calculate velocity of the ball
	// calculate the end position of the ball
	// time = distance / velocity

	double offset = 50.0;
	// if (!to_record_velocity && !time_logged) {
		t = fabs(obs->pos[X] - (home[X])) / bps->vel_x;
		t -= offset;
		// if (t <= 0) {
			// to_record_velocity = TRUE;
		// }
		// time_logged = TRUE;
	// }

	printf("%3.3f- ball_pos:(%3.3f,%3.3f)  vel:(%3.3f,%3.3f)  t:%3.3f\n", 
		time, obs->pos[X], obs->pos[Y], bps->vel_x, bps->vel_y, t) ;
}

int INTERCEPT(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	// go to the predicted end position of the ball
	// if there was external force or ball is not moving backwards, then CONVERGED
}


int block_observed = FALSE;
int block_predicted = FALSE;
int block_intercept = FALSE;
int block_interval = 300;
double block_ballpos[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
int block_ballpos_idx = 0;
int tiktok = 0;
int interval = 40;
double dest_x = 0.0;
double dest_y = 0.0;
// double vel_x = 0.0;
// double vel_y = 0.0;
double t_x = 0.0;

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

int CHASE2(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	if (state_punch == TRANSIENT || state_retreat == TRANSIENT) {
		return CONVERGED;
	}
	if (SEARCHTRACK(roger, time) == CONVERGED) {
		stereo_observation(roger, obs);
		recommended_setpoints[RS_CHASE].base[X] = obs->pos[X];
		recommended_setpoints[RS_CHASE].base[Y] = obs->pos[Y];

		double X_err = obs->pos[X] - roger->base_position[X];
		double Y_err = obs->pos[Y] - roger->base_position[Y];
		double base_err = sqrt(X_err*X_err + Y_err*Y_err);
		double base_offset = 1.00;
		double epsilon = R_BALL + R_BASE + base_offset;
		// printf("%.4f- base_err:%.4f  epsilon:%.4f\n", time, base_err, epsilon);
		if (base_err > epsilon) { state_chase = TRANSIENT; return TRANSIENT; }
		else { state_chase = CONVERGED; return CONVERGED; }
	}
	// printf("state_chase = NO_REFERENCE  from CHASEforPUNCH()\n");
	state_chase = NO_REFERENCE;
	return NO_REFERENCE;
}

int PUNCH2(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	double punch_pose_L[NARMS][NARM_JOINTS] = { -1.6 , -1.5, -1.5, 0.0 };
	double punch_pose_R[NARMS][NARM_JOINTS] = { 1.5 , 0.0, 1.6, 1.0 };

	printf("%3.3f- obs:(%3.3f,%3.3f) bps:(%3.3f,%3.3f)\n",
		time, obs->pos[X], obs->pos[Y], bps->x, bps->y);

	if (state_retreat == TRANSIENT) {
		return CONVERGED;
	}
	if (state_chase == CONVERGED && state_aim == CONVERGED && state_punch != CONVERGED) {
		tiktok_punch_nohit++;
		if (ball_LorR == LEFT) {
			roger->base_setpoint[THETA] = roger->base_setpoint[THETA] + M_PI;	// turn base
			recommended_setpoints[RS_PUNCH].arm[LEFT][0] = punch_pose_L[LEFT][0];
			recommended_setpoints[RS_PUNCH].arm[LEFT][1] = punch_pose_L[LEFT][1];
			recommended_setpoints[RS_PUNCH].arm[RIGHT][0] = punch_pose_L[RIGHT][0];
			recommended_setpoints[RS_PUNCH].arm[RIGHT][1] = punch_pose_L[RIGHT][1];
		} else if (ball_LorR == RIGHT) {
			roger->base_setpoint[THETA] = roger->base_setpoint[THETA] - M_PI; // turn base
			recommended_setpoints[RS_PUNCH].arm[LEFT][0] = punch_pose_R[LEFT][0];
			recommended_setpoints[RS_PUNCH].arm[LEFT][1] = punch_pose_R[LEFT][1];
			recommended_setpoints[RS_PUNCH].arm[RIGHT][0] = punch_pose_R[RIGHT][0];
			recommended_setpoints[RS_PUNCH].arm[RIGHT][1] = punch_pose_R[RIGHT][1];
		}

		double tolerance = 0.001;
		if (fabs(roger->ext_force[LEFT][0]) > tolerance && IsTactileRedBall(roger, time, LEFT) == TRUE) {
			// printf("%.4f- Left Touched!\n", time);
			left_touched = TRUE;
		}
		if (fabs(roger->ext_force[RIGHT][0]) > tolerance && IsTactileRedBall(roger, time, RIGHT) == TRUE) {
			// printf("%.4f- Right Touched!\n", time);
			right_touched = TRUE;
		}
		if (left_touched == TRUE || right_touched == TRUE) { 
			tiktok_punch_hit++;
			if (tiktok_punch_hit % tiktok_interval == 0) {
				printf("STATE_CONVERGED()!!!\n");
				STATE_CONVERGED();
				return CONVERGED; 
			}
		}

		if (tiktok_punch_nohit % tiktok_interval == 0 && left_touched == FALSE && right_touched == FALSE) {
			printf("No hit  STATE_RESET()\n");
			tiktok_punch_nohit = 0;
			STATE_RESET();
			return NO_REFERENCE;
		}

		state_punch = TRANSIENT;
		return TRANSIENT;
	}
	state_punch = NO_REFERENCE;
	return NO_REFERENCE;
}

int OFFENSE(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	int internal_state[3];
	internal_state[0] = CHASE2(roger, time, rsp, obs, bps);
	internal_state[1] = PUNCH2(roger, time, rsp, obs, bps);
	internal_state[2] = RETREAT2(roger, time, rsp, obs, bps);

	

	// int s = STOPBALL(roger, time);
	// int c = CENTERBALL(roger, time);
	// int a = AIM(roger, time);
	// int r = RETREAT(roger, time);



}

int DEFENSE(Robot* roger, double time, SetPoints* rsp, Observation* obs, BallPos* bps)
{
	int internal_state[4];

	// double vel_x = 0.0, vel_y = 0.0;
	// int o = OBSERVEBALL(roger, time, rsp, obs, bps);
	// printf("%3.3f- velocity:(%3.3f,%3.3f)\n", time, bps->vel_x, bps->vel_y);

	// int p = PREDICTPATH(roger, time, rsp, obs, bps);
	// int b = INTERCEPT(roger, time, rsp, obs, bps);
}



// int BLOCKKK(Robot* roger, double time)
// {

// 	if (SEARCHTRACK(roger, time)) {
// 		tiktok++;
// 		if (tiktok == interval * 2) {
// 		// if (block_intercept) {
// 			block_observed = FALSE;
// 			tiktok = 0;
// 			block_ballpos[0][X] = 0.0;
// 			block_ballpos[0][Y] = 0.0;
// 			block_ballpos[1][X] = 0.0;
// 			block_ballpos[1][Y] = 0.0;
// 		}
// 		if (!block_observed) {
// 			stereo_observation(roger, &obs);
// 			if (tiktok == interval * 0) {
// 				block_ballpos[0][X] = obs.pos[X];
// 				block_ballpos[0][Y] = obs.pos[Y];
// 			} else if (tiktok == interval * 1 ) {
// 				block_ballpos[1][X] = obs.pos[X];
// 				block_ballpos[1][Y] = obs.pos[Y];
// 				block_observed = TRUE;
// 			} 
// 		} else {
// 			if (!block_predicted) {
// 				// predict the destination of the ball
// 				vel_x = (block_ballpos[1][X] - block_ballpos[0][X]) / interval;
// 				vel_y = (block_ballpos[1][Y] - block_ballpos[0][Y]) / interval;
// 				t_x = (block_ballpos[1][X] - roger->base_position[X]) / vel_x;
// 				dest_x = block_ballpos[1][X] + vel_x * t_x;
// 				dest_y = block_ballpos[1][Y] + vel_y * t_x;
// 				printf("%3.3f- vel:(%3.3f,%3.3f)  dest:(%3.3f,%3.3f)  t_x:%3.3f\n", time, vel_x, vel_y, dest_x, dest_y, t_x);
// 	// printf("%3.3f- block_ballpos[0]:(%3.3f,%3.3f)  [1]:(%3.3f,%3.3f)  tiktok:%d\n", 
// 			// time, block_ballpos[0][X], block_ballpos[0][Y], block_ballpos[1][X], block_ballpos[1][Y], tiktok);
// 				if (dest_x > 0.0 || dest_x < MIN_X || dest_y > MAX_Y || dest_y < MIN_Y) {
// 					block_predicted = FALSE;
// 					block_observed = FALSE;
// 				} else {
// 					block_predicted = TRUE;
// 				}
// 			} else {
// 				if (!block_intercept) {
// 					// intercept the ball
// 					printf("%3.3f- dest:(%3.3f,%3.3f)\n", time, dest_x, dest_y);
// 					roger->base_setpoint[X] = dest_x;
// 					roger->base_setpoint[Y] = dest_y;
// 				}
// 			}
// 		}
// 	}
// 	// if (!observed)
// 	//    observe two positions of the ball
// 	// else
// 	//    if (!predicted)
// 	//       calculate the end point of the ball
// 	//    else
// 	//        if (!intercept)
// 	//            try to intercept the ball
// }






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


	// setpoint_filter(roger);

	// BLOCK(roger, time);
	// RETREAT(roger, time);
	// CHASEPUNCHRETREAT(roger, time);
	// CHASETOUCH(roger, time);

	// printf("%.4f- arm_setpoint:(%.4f,%.4f,%.4f,%.4f)\n", time, roger->arm_setpoint[LEFT][0], roger->arm_setpoint[LEFT][1], roger->arm_setpoint[RIGHT][0], roger->arm_setpoint[RIGHT][1]);
	// printf("%.4f- home:[%.4f,%.4f,%.4f]\n", time, home[X], home[Y], home[THETA]);
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

