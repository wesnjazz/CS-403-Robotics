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

/***
Offense()
	SearchTrack()
		Search()
		Track()
	ChaseGrabCenterAimPunch()
		ChaseGrab()
			Chase()
			Grab()
		CenterAimPunch()
			Center()
			Aim()
			Punch()
	Retreat()

Defense()
	SearchTrack()
		Search()
		Track()
	Block()
		Velocity()
		Predict()
		Intercept()

***/

#define RS_HOME 														0

#define RS_SEARCH 													1
#define RS_TRACK 														2
#define RS_SEARCH_TRACK 										3

#define RS_CHASE 														4
#define RS_PUNCH 														5
#define RS_CHASE_PUNCH 											6

#define RS_RETREAT 													7
#define RS_CHASE_PUNCH_RETREAT							8

#define RS_TURN90 													9

#define RS_SAMPLE														10
#define RS_BACKFORTH												11
#define RS_SAMPLE_BACKFORTH 								12

#define RS_BLOCK														13
#define RS_SAMPLE_BACKFORTH_BLOCK						14

#define RS_TURN90_SAMPLE_BACKFORTH_BLOCK		15

#define BALL_RND	-0.0
#define NBINS_BPS 50

#define ROGER_MODE_DEFENSE 0
#define ROGER_MODE_OFFENSE 1
int Roger_mode = ROGER_MODE_DEFENSE;
// int Roger_mode = ROGER_MODE_OFFENSE;

typedef struct _setpoints {    /* base[THETA] added onto pre-existed struct */
  double base[3];             /* (x,y,theta) of the base in world frame */
  double arm[NARMS][NARM_JOINTS]; /* arm joint angles */
  double eye[NEYES];         /* eye verge angle relative to base frame */
} SetPoints;

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
	int bps_created;
} BallPos;

SetPoints rec_setp[16];
Observation obs;
BallPos bps;
int bps_created = FALSE;
int state_search_track = NO_REFERENCE;

int init_BallPos() {
	for (int i=0; i<NBINS_BPS; i++) {
		bps.x[i] = BALL_RND;
		bps.y[i] = BALL_RND;
		bps.time[i] = BALL_RND;
	}
	bps.head = 0;
	bps.tail = 0;
	bps.size = NBINS_BPS;
	bps.cnt = 0;
	bps.vel_x = 0.0;
	bps.vel_y = 0.0;
}

double prev_x;
double prev_y;
int record_BallPos(Robot* roger, double time) 
{
	int head = bps.head;
	int tail = bps.tail;
	int size = bps.size;
	int cnt = bps.cnt;

	double x = obs.pos[X];
	double y = obs.pos[Y];
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
		bps.head = 0;
		bps.tail = 0;
		bps.cnt = 1;
		bps.time[0] = time;
		bps.x[0] = x;
		bps.y[0] = y;
		prev_x = x;
		prev_y = y;
		return TRUE;
	}

	// get difference between current x,y and previous x,y
	// if difference bigger than epsilon, it is a noise, so record trash value
	double prev_x = (tail > 0) ? (bps.x[tail-1]) : (bps.x[NBINS_BPS-1]);
	double prev_y = (tail > 0) ? (bps.y[tail-1]) : (bps.y[NBINS_BPS-1]);
	double diff_x = fabs(x - prev_x);
	double diff_y = fabs(y - prev_y);
	// printf("%3.3f- prev:(%3.3f,%3.3f)  now:(%3.3f,%3.3f)  diff:(%3.3f,%3.3f)\n", time, prev_x, prev_y, bps.x[tail], bps.y[tail], diff_x, diff_y);
	double epsilon = 1.0;
	if (prev_x != BALL_RND && diff_x > epsilon) {
		x = prev_x;
		diff_x = fabs(x - prev_x);
	// printf("\t\t%3.3f- prev:(%3.3f,%3.3f)  now:(%3.3f,%3.3f)  diff:(%3.3f,%3.3f)\n", time, prev_x, prev_y, bps.x[tail], bps.y[tail], diff_x, diff_y);
	}
	if (prev_y != BALL_RND && diff_y > epsilon) {
		y = prev_y;
		diff_y = fabs(y - prev_y);
	// printf("\t\t%3.3f- prev:(%3.3f,%3.3f)  now:(%3.3f,%3.3f)  diff:(%3.3f,%3.3f)\n", time, prev_x, prev_y, bps.x[tail], bps.y[tail], diff_x, diff_y);
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
	bps.time[tail] = time;
	bps.x[tail] = x;
	bps.y[tail] = y;
	bps.tail = tail;
	bps.head = head;
	prev_x = x;
	prev_y = y;
	if (cnt < NBINS_BPS) {
		bps.cnt++;
	}
	return TRUE;
}



/**************************************************************************************/
/*** SEARCH_TRACK ***/
/**************************************************************************************/

int tt_search = 100;
int tt_search_interval = 200;
double prev_heading = 0.0;
int SEARCH(Robot* roger, double time)
/** eye[LEFT,RIGHT], base[THETA] **/
{
	tt_search++;
	double heading = 0.0;

	/** change eye angle every 100 and 200 ms from -M_PI to M_PI **/
	if (tt_search == 100) {
		rec_setp[RS_SEARCH].eye[LEFT] = -M_PI;
		rec_setp[RS_SEARCH].eye[RIGHT] = -M_PI;
	}
	if (tt_search == 200) {
		rec_setp[RS_SEARCH].eye[LEFT] = M_PI;
		rec_setp[RS_SEARCH].eye[RIGHT] = M_PI;
	}

	/** don't get new sample if the given interval is not fulfilled **/
	if (tt_search % tt_search_interval != 0) {
		return TRANSIENT;
	}

	tt_search = 0;

	/** if sampling succeeded **/
	if (sample_gaze_direction(&heading) == TRUE) {
		while (heading > M_PI || heading < 0.0) {	/** when heading is over M_PI, e.g., 3.1907 **/
			sample_gaze_direction(&heading);
		}
		while (fabs(prev_heading - heading) < M_PI/2.0) {
			sample_gaze_direction(&heading);
		}
		// printf("%3.3f- prev_heading:%3.3f  heading:%3.3f\n", time, prev_heading, heading);

		/** adjust gaze as eyes get closer to the heading **/
		double left_gaze = zeroTo2PI(heading) - zeroTo2PI(roger->base_position[THETA]);
		double right_gaze = left_gaze;

		/** set base_setpoint to the heading **/
		rec_setp[RS_SEARCH].base[THETA] = heading;
		prev_heading = heading;

		double cover_min = roger->base_position[THETA] - (M_PI/9.0);
		double cover_max = roger->base_position[THETA] + (M_PI/9.0);
		if (roger->base_setpoint[THETA] >= cover_min && roger->base_setpoint[THETA] <= cover_max) {
			/** if new setpoint is within range **/
			return CONVERGED;
		} else {
			/** if not within range **/
			return TRANSIENT;
		}
	} else {
		/** if sampling failed **/
		return NO_REFERENCE;
	}
}

int TRACK(Robot* roger, double time)
/** eye[LEFT,RIGHT], base[THETA] **/
{
	double ul = -1.0;
	double ur = -1.0;

	/** if a red pixel is detected on both eyes **/
	if ( average_red_pixel(roger, &ul, &ur) == TRUE ) {
		stereo_observation(roger, &obs);

		// printf("%3.3f- obs:(%3.3f,%3.3f)\n", time, obs.pos[X], obs.pos[Y]);
		/** get the eye angle error **/
		double eye_theta_error_l = atan2((ul - 63.5), FOCAL_LENGTH);
		double eye_theta_error_r = atan2((ur - 63.5), FOCAL_LENGTH);
		/** foveate **/
		rec_setp[RS_TRACK].eye[LEFT] = roger->eye_theta[LEFT] + eye_theta_error_l;
		rec_setp[RS_TRACK].eye[RIGHT] = roger->eye_theta[RIGHT] + eye_theta_error_r;
		rec_setp[RS_TRACK].base[THETA] = roger->base_position[THETA] + (roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]) / 2.0;

		/** check if foveated **/
		double eye_diff = fabs(roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]);
		double epsilon = 1.0;
		// double epsilon = 0.5;
		if (eye_diff < epsilon) {
			return CONVERGED;
		} else {
			/** if not foveated **/
			return TRANSIENT;
		}
	} else {
		/** if no red pixel is detected **/
		return NO_REFERENCE;
	}
}

int SEARCH_TRACK(Robot* roger, double time)
{
	int internal_state[2];
	internal_state[0] = SEARCH(roger, time); // assigns values to rec_setp[0]
	internal_state[1] = TRACK(roger, time); // assigns values to rec_setp[1]
	int state = internal_state[1]*3 + internal_state[0];
	// printf("%3.3f, SEARCH:%d  TRACK:%d  Case:%d\n", time, internal_state[0], internal_state[1], state);
	int return_state;
	switch (state) {
						// TRACK 					SEARCH
		case 0: // NO_REFERENCE - NO_REFERENCE
		case 1: // NO_REFERENCE - TRANSIENT
		case 2: // NO_REFERENCE - CONVERGED
			rec_setp[RS_SEARCH_TRACK].eye[LEFT] = rec_setp[RS_SEARCH].eye[LEFT];
			rec_setp[RS_SEARCH_TRACK].eye[RIGHT] = rec_setp[RS_SEARCH].eye[RIGHT];
			rec_setp[RS_SEARCH_TRACK].base[THETA] = rec_setp[RS_SEARCH].base[THETA];
			rec_setp[RS_SEARCH_TRACK].base[X] = roger->base_position[X];
			rec_setp[RS_SEARCH_TRACK].base[Y] = roger->base_position[Y];
			rec_setp[RS_SEARCH_TRACK].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_SEARCH_TRACK].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_SEARCH_TRACK].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_SEARCH_TRACK].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			return_state = NO_REFERENCE;
			state_search_track = NO_REFERENCE;
			break;
	
		case 3: // TRANSIENT - NO_REFERENCE
		case 4: // TRANSIENT - TRANSIENT
		case 5: // TRANSIENT - CONVERGED
			rec_setp[RS_SEARCH_TRACK].eye[LEFT] = rec_setp[RS_TRACK].eye[LEFT];
			rec_setp[RS_SEARCH_TRACK].eye[RIGHT] = rec_setp[RS_TRACK].eye[RIGHT];
			rec_setp[RS_SEARCH_TRACK].base[THETA] = rec_setp[RS_TRACK].base[THETA];
			rec_setp[RS_SEARCH_TRACK].base[X] = roger->base_position[X];
			rec_setp[RS_SEARCH_TRACK].base[Y] = roger->base_position[Y];
			rec_setp[RS_SEARCH_TRACK].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_SEARCH_TRACK].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_SEARCH_TRACK].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_SEARCH_TRACK].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			// roger->eyes_setpoint[LEFT] = rec_setp[RS_TRACK].eye[LEFT];
			// roger->eyes_setpoint[RIGHT] = rec_setp[RS_TRACK].eye[RIGHT];
			// roger->base_setpoint[THETA] = rec_setp[RS_TRACK].base[THETA];
			return_state = TRANSIENT;
			state_search_track = TRANSIENT;
			break;
	
		case 6: // CONVERGED - NO_REFERENCE
		case 7: // CONVERGED - TRANSIENT
		case 8: // CONVERGED - CONVERGED
			rec_setp[RS_SEARCH_TRACK].eye[LEFT] = rec_setp[RS_TRACK].eye[LEFT];
			rec_setp[RS_SEARCH_TRACK].eye[RIGHT] = rec_setp[RS_TRACK].eye[RIGHT];
			rec_setp[RS_SEARCH_TRACK].base[THETA] = rec_setp[RS_TRACK].base[THETA];
			rec_setp[RS_SEARCH_TRACK].base[X] = roger->base_position[X];
			rec_setp[RS_SEARCH_TRACK].base[Y] = roger->base_position[Y];
			rec_setp[RS_SEARCH_TRACK].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_SEARCH_TRACK].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_SEARCH_TRACK].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_SEARCH_TRACK].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			return_state = CONVERGED;
			state_search_track = CONVERGED;
			break;
		default:
			break;
	}
	return return_state;
}





/**************************************************************************************/
/*** OFFENSE ***/
/**************************************************************************************/

int state_chase = NO_REFERENCE;
int state_punch = NO_REFERENCE;
int state_retreat = NO_REFERENCE;
int CHASE(Robot* roger, double time)
/** base[X,Y,THETA] **/
{
	if (state_punch == TRANSIENT || state_retreat == TRANSIENT) { 
		state_chase = CONVERGED; 
		return CONVERGED; 
	}

	if (state_search_track == CONVERGED) {	/** if found the red ball **/
		/** set setpoints (x,y,theta) towards the red ball **/
		double angle = atan2(obs.pos[Y] - roger->base_position[Y], obs.pos[X] - roger->base_position[X]);
		rec_setp[RS_CHASE].base[X] = obs.pos[X];
		rec_setp[RS_CHASE].base[Y] = obs.pos[Y];
		rec_setp[RS_CHASE].base[THETA] = roger->base_position[THETA] + (roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]) / 2.0;

		/** determine if the base is reached the red ball within the range **/
		double X_err = obs.pos[X] - roger->base_position[X];
		double Y_err = obs.pos[Y] - roger->base_position[Y];
		double base_err = sqrt(X_err*X_err + Y_err*Y_err);
		double base_offset = 0.65;
		double epsilon = R_BALL + R_BASE + base_offset;
		if (base_err > epsilon) { state_chase = TRANSIENT; return TRANSIENT; }
		else { 
			/** stop moving if reached to the red ball **/
			rec_setp[RS_CHASE].base[X] = roger->base_position[X];
			rec_setp[RS_CHASE].base[Y] = roger->base_position[Y];
			state_chase = CONVERGED;
			return CONVERGED; 
		}
	}
	state_chase = NO_REFERENCE;
	return NO_REFERENCE;
}

int IsTactileRedBall(Robot* roger, double time, int limb)
{
	Observation obsBframe;
	double arm_x = 0.0, arm_y = 0.0;
	
	if (limb == LEFT) {
		fwd_arm_kinematics(roger->arm_theta[LEFT][0], roger->arm_theta[LEFT][1], &arm_x, &arm_y);
	} else if (limb == RIGHT) {
		fwd_arm_kinematics(roger->arm_theta[RIGHT][0], roger->arm_theta[RIGHT][1], &arm_x, &arm_y);
	}
	
	stereo_observation_Bframe(roger, &obsBframe);
	double d = sqrt( (obsBframe.pos[X] - arm_x)*(obsBframe.pos[X] - arm_x) + (obsBframe.pos[Y] - arm_y)*(obsBframe.pos[Y] - arm_y) );
	// printf("%.4f: R_BALL:%.4f  arm:(%.4f, %.4f)  ball:(%.4f, %.4f)  d=%.4f\n", time, R_BALL, arm_x, arm_y, obsBframe.pos[X], obsBframe.pos[Y], d);

	if (d < R_BALL + 0.6) { return TRUE; }
	else { return FALSE; }
}

int tiktok_punch_hit = 1;
int tiktok_punch_nohit = 1;
int tiktok_punch_interval = 150;
int left_touched = FALSE;
int right_touched = FALSE;
int PUNCH(Robot* roger, double time)
/** base[X,Y,THETA], arms **/
{
	if (state_retreat == TRANSIENT) {
		state_punch = CONVERGED;
		return CONVERGED;
	}
	if (state_chase == CONVERGED && state_punch != CONVERGED && state_retreat != CONVERGED) {
		tiktok_punch_nohit++;

		stereo_observation(roger, &obs);
		int ball_LorR = LEFT;
		double angle = atan2(obs.pos[Y] - roger->base_position[Y], obs.pos[X] - roger->base_position[X]);
		double angle_err = roger->base_position[THETA] - angle;
		double angle_offset = 0.3;
		if (angle_err < 0) {ball_LorR = LEFT;}
		else {ball_LorR = RIGHT;}

		if (ball_LorR == LEFT) {
			// printf("\t\t\t\t\tBall is LEFT\n");
			rec_setp[RS_PUNCH].base[X] = obs.pos[X];
			rec_setp[RS_PUNCH].base[Y] = obs.pos[Y];
			rec_setp[RS_PUNCH].base[THETA] = roger->base_position[THETA] - M_PI/2.0;
			rec_setp[RS_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_PUNCH].arm[LEFT][1] = -1.2;
			rec_setp[RS_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_PUNCH].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
		} else if (ball_LorR == RIGHT) {
			// printf("\t\t\t\t\tBall is RIGHT\n");
			rec_setp[RS_PUNCH].base[X] = obs.pos[X];
			rec_setp[RS_PUNCH].base[Y] = obs.pos[Y];
			rec_setp[RS_PUNCH].base[THETA] = roger->base_position[THETA] + M_PI/2.0;
			rec_setp[RS_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_PUNCH].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_PUNCH].arm[RIGHT][1] = 1.2;
		} else {
			rec_setp[RS_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_PUNCH].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_PUNCH].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
		}

		double tolerance = 0.001;
		if (fabs(roger->ext_force[LEFT][1]) > tolerance && IsTactileRedBall(roger, time, LEFT) == TRUE) {
			printf("%.4f- Left Touched!\n", time);
			left_touched = TRUE;
		}
		if (fabs(roger->ext_force[RIGHT][1]) > tolerance && IsTactileRedBall(roger, time, RIGHT) == TRUE) {
			printf("%.4f- Right Touched!\n", time);
			right_touched = TRUE;
		}

		/** punched, and hit the red ball successfully **/
		if (left_touched == TRUE || right_touched == TRUE) { 
			tiktok_punch_hit++;
			if (tiktok_punch_hit % tiktok_punch_interval == 0) {
				printf("\t\t\t\t\t\t[+] HIT Succeeded!!!\n");
				left_touched = right_touched = FALSE;
				tiktok_punch_hit = 1;
				state_punch = CONVERGED;
				return CONVERGED; 
			}
		}

		/** punched, and hit failed **/
		if (tiktok_punch_nohit % tiktok_punch_interval == 0 && left_touched == FALSE && right_touched == FALSE) {
			printf("\t\t\t\t\t\t[-] NO HIT...\n");
			tiktok_punch_nohit = 1;
			// STATE_RESET();
			state_punch = NO_REFERENCE;
			return NO_REFERENCE;
		}
		state_punch = TRANSIENT;
		return TRANSIENT;
	}
	state_punch = NO_REFERENCE;
	return NO_REFERENCE;
}

int CHASE_PUNCH(Robot* roger, double time)
{
	int internal_state[2];
	internal_state[0] = CHASE(roger, time);
	internal_state[1] = PUNCH(roger, time);
	int state = internal_state[0]*3 + internal_state[1];
	// printf("%3.3f, CHASE:%d  PUNCH:%d  Case:%d\n", time, internal_state[0], internal_state[1], state);
	int return_state;
	switch (state) {
						// CHASE 					PUNCH
		case 0: // NO_REFERENCE - NO_REFERENCE
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_SEARCH_TRACK].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_SEARCH_TRACK].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			return_state = NO_REFERENCE;
			break;
		case 1: // NO_REFERENCE - TRANSIENT
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_SEARCH_TRACK].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_SEARCH_TRACK].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_PUNCH].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_PUNCH].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_PUNCH].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_PUNCH].arm[RIGHT][1];
			return_state = NO_REFERENCE;
			break;
		case 2: // NO_REFERENCE - CONVERGED
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_SEARCH_TRACK].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_SEARCH_TRACK].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			return_state = NO_REFERENCE;
			break;
		case 3: // TRANSIENT - NO_REFERENCE
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_CHASE].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_CHASE].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 4: // TRANSIENT - TRANSIENT
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_CHASE].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_CHASE].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;

		case 5: // TRANSIENT - CONVERGED
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_CHASE].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_CHASE].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 6: // CONVERGED - NO_REFERENCE
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_CHASE].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_CHASE].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 7: // CONVERGED - TRANSIENT
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_PUNCH].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_PUNCH].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_PUNCH].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_PUNCH].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_PUNCH].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_PUNCH].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_PUNCH].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 8: // CONVERGED - CONVERGED
			rec_setp[RS_CHASE_PUNCH].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH].base[THETA] = rec_setp[RS_PUNCH].base[THETA];
			rec_setp[RS_CHASE_PUNCH].base[X] = rec_setp[RS_PUNCH].base[X];
			rec_setp[RS_CHASE_PUNCH].base[Y] = rec_setp[RS_PUNCH].base[Y];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][0] = rec_setp[RS_PUNCH].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH].arm[LEFT][1] = rec_setp[RS_PUNCH].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0] = rec_setp[RS_PUNCH].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1] = rec_setp[RS_PUNCH].arm[RIGHT][1];
			return_state = CONVERGED;
			break;
		default:
			break;
	}
	return return_state;
}

int RETREAT(Robot* roger, double time)
/** base[X,Y,THETA], arms **/
{
	if (state_punch == CONVERGED) {
		rec_setp[RS_RETREAT].arm[LEFT][0] = rec_setp[RS_HOME].arm[LEFT][0];
		rec_setp[RS_RETREAT].arm[LEFT][1] = rec_setp[RS_HOME].arm[LEFT][1];
		rec_setp[RS_RETREAT].arm[RIGHT][0] = rec_setp[RS_HOME].arm[RIGHT][0];
		rec_setp[RS_RETREAT].arm[RIGHT][1] = rec_setp[RS_HOME].arm[RIGHT][1];

		double X_err = rec_setp[RS_HOME].base[X] - roger->base_position[X];
		double Y_err = rec_setp[RS_HOME].base[Y] - roger->base_position[Y];

		double epsilon = 0.2;
		if (fabs(X_err) < epsilon && fabs(Y_err) < epsilon) {
			state_retreat = CONVERGED;
	Roger_mode = ROGER_MODE_DEFENSE;
			return CONVERGED;
		}

		double home_angle = atan2(Y_err, X_err);

		rec_setp[RS_RETREAT].base[X] = rec_setp[RS_HOME].base[X];
		rec_setp[RS_RETREAT].base[Y] = rec_setp[RS_HOME].base[Y];
		rec_setp[RS_RETREAT].base[THETA] = M_PI + home_angle;
		// rec_setp[RS_RETREAT].base[THETA] = roger->base_position[THETA];

		state_retreat = TRANSIENT;
		return TRANSIENT;
	}
	state_retreat = NO_REFERENCE;
	return NO_REFERENCE;

}

int CHASE_PUNCH_RETREAT(Robot* roger, double time)
{
	int internal_state[2];
	internal_state[0] = CHASE_PUNCH(roger, time);
	internal_state[1] = RETREAT(roger, time);
	int state = internal_state[0]*3 + internal_state[1];
	// printf("%3.3f, CHASE_PUNCH:%d  RETREAT:%d  Case:%d\n", time, internal_state[0], internal_state[1], state);
	int return_state;
	switch (state) {
						// CHASE_PUNCH 		RETREAT
		case 0: // NO_REFERENCE - NO_REFERENCE
		case 1: // NO_REFERENCE - TRANSIENT
		case 2: // NO_REFERENCE - CONVERGED
		case 3: // TRANSIENT - NO_REFERENCE
		case 4: // TRANSIENT - TRANSIENT
		case 5: // TRANSIENT - CONVERGED
			rec_setp[RS_CHASE_PUNCH_RETREAT].eye[LEFT] = rec_setp[RS_CHASE_PUNCH].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH_RETREAT].eye[RIGHT] = rec_setp[RS_CHASE_PUNCH].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[THETA] = rec_setp[RS_CHASE_PUNCH].base[THETA];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[X] = rec_setp[RS_CHASE_PUNCH].base[X];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[Y] = rec_setp[RS_CHASE_PUNCH].base[Y];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][0] = rec_setp[RS_CHASE_PUNCH].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][1] = rec_setp[RS_CHASE_PUNCH].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][0] = rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][1] = rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;

		case 6: // CONVERGED - NO_REFERENCE
			rec_setp[RS_CHASE_PUNCH_RETREAT].eye[LEFT] = rec_setp[RS_CHASE_PUNCH].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH_RETREAT].eye[RIGHT] = rec_setp[RS_CHASE_PUNCH].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[THETA] = rec_setp[RS_CHASE_PUNCH].base[THETA];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[X] = rec_setp[RS_CHASE_PUNCH].base[X];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[Y] = rec_setp[RS_CHASE_PUNCH].base[Y];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][0] = rec_setp[RS_CHASE_PUNCH].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][1] = rec_setp[RS_CHASE_PUNCH].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][0] = rec_setp[RS_CHASE_PUNCH].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][1] = rec_setp[RS_CHASE_PUNCH].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 7: // CONVERGED - TRANSIENT
			rec_setp[RS_CHASE_PUNCH_RETREAT].eye[LEFT] = rec_setp[RS_CHASE_PUNCH].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH_RETREAT].eye[RIGHT] = rec_setp[RS_CHASE_PUNCH].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[THETA] = rec_setp[RS_RETREAT].base[THETA];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[X] = rec_setp[RS_RETREAT].base[X];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[Y] = rec_setp[RS_RETREAT].base[Y];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][0] = rec_setp[RS_RETREAT].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][1] = rec_setp[RS_RETREAT].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][0] = rec_setp[RS_RETREAT].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][1] = rec_setp[RS_RETREAT].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 8: // CONVERGED - CONVERGED
			rec_setp[RS_CHASE_PUNCH_RETREAT].eye[LEFT] = rec_setp[RS_CHASE_PUNCH].eye[LEFT];
			rec_setp[RS_CHASE_PUNCH_RETREAT].eye[RIGHT] = rec_setp[RS_CHASE_PUNCH].eye[RIGHT];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[THETA] = rec_setp[RS_RETREAT].base[THETA];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[X] = rec_setp[RS_RETREAT].base[X];
			rec_setp[RS_CHASE_PUNCH_RETREAT].base[Y] = rec_setp[RS_RETREAT].base[Y];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][0] = rec_setp[RS_RETREAT].arm[LEFT][0];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][1] = rec_setp[RS_RETREAT].arm[LEFT][1];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][0] = rec_setp[RS_RETREAT].arm[RIGHT][0];
			rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][1] = rec_setp[RS_RETREAT].arm[RIGHT][1];
			return_state = CONVERGED;
			break;
		default:
			break;
	}
	return return_state;
}

int SEARCH_TRACK_CHASE_PUNCH_RETREAT(Robot* roger, double time)
{
	/***********************************************************************************************/
	/* internal_state=[ 0:SEARCH 1:TRACK ] */
	/***********************************************************************************************/
	int internal_state[2];
	internal_state[0] = SEARCH_TRACK(roger, time);
	internal_state[1] = CHASE_PUNCH_RETREAT(roger, time);
	int state = internal_state[0]*3 + internal_state[1];
	// printf("%3.3f, SEARCH_TRACK:%d  CHASE_PUNCH_RETREAT:%d  Case:%d\n", time, internal_state[0], internal_state[1], state);
	int return_state;
	switch (state) {
						// SEARCH_TRACK 	CHASE_PUNCH
		case 0: // NO_REFERENCE - NO_REFERENCE
		case 1: // NO_REFERENCE - TRANSIENT
		case 2: // NO_REFERENCE - CONVERGED
			roger->eyes_setpoint[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			roger->eyes_setpoint[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			roger->base_setpoint[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			roger->base_setpoint[X] = rec_setp[RS_SEARCH_TRACK].base[X];
			roger->base_setpoint[Y] =	rec_setp[RS_SEARCH_TRACK].base[Y];
			roger->arm_setpoint[LEFT][0] = rec_setp[RS_SEARCH_TRACK].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = rec_setp[RS_SEARCH_TRACK].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = rec_setp[RS_SEARCH_TRACK].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = rec_setp[RS_SEARCH_TRACK].arm[RIGHT][1];
			return_state = NO_REFERENCE;
			break;
	
		case 3: // TRANSIENT - NO_REFERENCE
		case 4: // TRANSIENT - TRANSIENT
		case 5: // TRANSIENT - CONVERGED
		case 6: // CONVERGED - NO_REFERENCE
		case 7: // CONVERGED - TRANSIENT
			roger->eyes_setpoint[LEFT] = rec_setp[RS_CHASE_PUNCH_RETREAT].eye[LEFT];
			roger->eyes_setpoint[RIGHT] = rec_setp[RS_CHASE_PUNCH_RETREAT].eye[RIGHT];
			roger->base_setpoint[THETA] = rec_setp[RS_CHASE_PUNCH_RETREAT].base[THETA];
			roger->base_setpoint[X] = rec_setp[RS_CHASE_PUNCH_RETREAT].base[X];
			roger->base_setpoint[Y] =	rec_setp[RS_CHASE_PUNCH_RETREAT].base[Y];
			roger->arm_setpoint[LEFT][0] = rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 8: // CONVERGED - CONVERGED
			roger->eyes_setpoint[LEFT] = rec_setp[RS_CHASE_PUNCH_RETREAT].eye[LEFT];
			roger->eyes_setpoint[RIGHT] = rec_setp[RS_CHASE_PUNCH_RETREAT].eye[RIGHT];
			roger->base_setpoint[THETA] = rec_setp[RS_CHASE_PUNCH_RETREAT].base[THETA];
			roger->base_setpoint[X] = rec_setp[RS_CHASE_PUNCH_RETREAT].base[X];
			roger->base_setpoint[Y] =	rec_setp[RS_CHASE_PUNCH_RETREAT].base[Y];
			roger->arm_setpoint[LEFT][0] = rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = rec_setp[RS_CHASE_PUNCH_RETREAT].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = rec_setp[RS_CHASE_PUNCH_RETREAT].arm[RIGHT][1];
			return_state = CONVERGED;
			break;
		default:
			break;
	}
	return return_state;
}





int sampled_two = TRUE;
int TURN90(Robot* roger, double time)
{
	double cur_angle = roger->base_position[THETA];
	double des_angle = -1.57;
	double epsilon = 0.1;
	if (fabs(fabs(roger->base_position[THETA]) - fabs(des_angle)) < epsilon) {
		sampled_two = FALSE;
		return CONVERGED;
	} else {
		if (rec_setp[RS_HOME].base[X] < 0.0) {
			rec_setp[RS_TURN90].eye[LEFT] = M_PI;
			rec_setp[RS_TURN90].eye[RIGHT] = M_PI;
			rec_setp[RS_TURN90].base[X] = rec_setp[RS_HOME].base[X];
			rec_setp[RS_TURN90].base[THETA] = -1.57;
			rec_setp[RS_TURN90].arm[LEFT][0] = -2.2;
			rec_setp[RS_TURN90].arm[LEFT][1] = -2.6;
			rec_setp[RS_TURN90].arm[RIGHT][0] = 0.0;
			rec_setp[RS_TURN90].arm[RIGHT][1] = 2.0;
		} else {
			rec_setp[RS_TURN90].eye[LEFT] = M_PI;
			rec_setp[RS_TURN90].eye[RIGHT] = M_PI;
			rec_setp[RS_TURN90].base[X] = rec_setp[RS_HOME].base[X];
			rec_setp[RS_TURN90].base[THETA] = 1.57;
			rec_setp[RS_TURN90].arm[LEFT][0] = -2.2;
			rec_setp[RS_TURN90].arm[LEFT][1] = -2.6;
			rec_setp[RS_TURN90].arm[RIGHT][0] = 0.0;
			rec_setp[RS_TURN90].arm[RIGHT][1] = 2.0;
		}
		return TRANSIENT;
	}
}

int tiktok_sample = 1;
int tiktok_sample_interval = 15;
double ul_1st = -1.0;
double ul_2nd = -1.0;
double last_ul = -1.0;
#define DIR_FORTH -1
#define DIR_BACK	1
int direction = DIR_FORTH;
int SAMPLE(Robot* roger, double time)
{
	double ul = -1.0;
	double ur = -1.0;
	if(average_red_pixel(roger, &ul, &ur) == TRUE) {
		/** no recorded two samples yet **/
		if (sampled_two != TRUE) {
			tiktok_sample++;
			/** record 1st sample **/
			if (ul_1st <= 0.0) {
				ul_1st = ul;
			} 
			/** record 2nd sample **/
			if (tiktok_sample % tiktok_sample_interval == 0 && ul_1st >= 0.0) {
				ul_2nd = ul;
				sampled_two = TRUE;
				/** determine the direction of the red ball **/
				double diff = ul_1st - ul_2nd;
				double epsilon = 0.00001;
				if (diff > 0.0) { direction = DIR_FORTH; } /** direction is FORTH: ul change is positive: from right to left on the eye image plane **/
				else if (diff < 0.0) { direction = DIR_BACK; } /** direction is BACK: ul change is negative: from left to right on the eye image plane **/
				else if (diff < epsilon) {	/** ball is at the same place **/
					double diff_2 = last_ul - ul_1st;
					if (diff_2 > 0.0) { direction = DIR_FORTH; }	/** red pixel is on the right on the eye **/
					else { direction = DIR_BACK; }	/** red pixel is on the left on the eye **/
				}
				if (rec_setp[RS_HOME].base[X] > 0.0) { direction = -direction; }

		// printf("%3.3f- eye:(%3.3f,%3.3f)\n", time, roger->eye_theta[LEFT], roger->eye_theta[RIGHT]);
		// printf("%3.3f- base:(%3.3f,%3.3f,%3.3f)\n", time, roger->base_position[X], roger->base_position[Y], roger->base_position[THETA]);
		// printf("%3.3f- tt:%d  ul,ur:(%3.3f,%3.3f)   ul_1st:%3.3f ul_2nd:%3.3f   direction:", time, tiktok_sample, ul, ur, ul_1st, ul_2nd);
		// if (direction == DIR_FORTH) { printf(" FORTH\n"); }
		// else if (direction == DIR_BACK) { printf(" BACK\n"); }
				// return CONVERGED;
			}
		}
	}
	else { return NO_REFERENCE; }

	/** resample after n ms **/
	if (sampled_two == TRUE && tiktok_sample % tiktok_sample_interval == 0) {
		sampled_two = FALSE;
		ul_1st = -1.0;
		ul_2nd = -1.0;
		last_ul = ul_1st;
		tiktok_sample = 1;
		// return CONVERGED;
	}
	return TRANSIENT;
}

int BACKFORTH(Robot* roger, double time)
{
	if (direction == DIR_FORTH) {
		rec_setp[RS_BACKFORTH].eye[LEFT] = roger->eye_theta[LEFT];
		rec_setp[RS_BACKFORTH].eye[RIGHT] = roger->eye_theta[RIGHT];
		rec_setp[RS_BACKFORTH].base[Y] = MIN_Y;
	} else {
		rec_setp[RS_BACKFORTH].eye[LEFT] = -roger->eye_theta[LEFT];
		rec_setp[RS_BACKFORTH].eye[RIGHT] = -roger->eye_theta[RIGHT];
		rec_setp[RS_BACKFORTH].base[Y] = MAX_Y;
	}
	return TRANSIENT;
}

int SAMPLE_BACKFORTH(Robot* roger, double time)
{
	int internal_state[2];
	internal_state[0] = SAMPLE(roger, time);
	internal_state[1] = BACKFORTH(roger, time);

	int state = internal_state[0];
	// printf("%3.3f, SAMPLE:%d  BACKFORTH:%d  Case:%d\n", time, internal_state[0], internal_state[1], state);
	int return_state;
	switch (state) {
						// SAMPLE 				BACKFORTH
		case 0: // NO_REFERENCE - TRANSIENT
		case 1: // TRANSIENT - 		TRANSIENT
			rec_setp[RS_SAMPLE_BACKFORTH].eye[LEFT] = rec_setp[RS_BACKFORTH].eye[LEFT];
			rec_setp[RS_SAMPLE_BACKFORTH].eye[RIGHT] = rec_setp[RS_BACKFORTH].eye[RIGHT];
			rec_setp[RS_SAMPLE_BACKFORTH].base[X] = rec_setp[RS_TURN90].base[X];
			rec_setp[RS_SAMPLE_BACKFORTH].base[Y] = rec_setp[RS_BACKFORTH].base[Y];
			rec_setp[RS_SAMPLE_BACKFORTH].base[THETA] = rec_setp[RS_TURN90].base[THETA];
			rec_setp[RS_SAMPLE_BACKFORTH].arm[LEFT][0] = rec_setp[RS_TURN90].arm[LEFT][0];
			rec_setp[RS_SAMPLE_BACKFORTH].arm[LEFT][1] = rec_setp[RS_TURN90].arm[LEFT][1];
			rec_setp[RS_SAMPLE_BACKFORTH].arm[RIGHT][0] = rec_setp[RS_TURN90].arm[RIGHT][0];
			rec_setp[RS_SAMPLE_BACKFORTH].arm[RIGHT][1] = rec_setp[RS_TURN90].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		default:
			break;
	}
	return return_state;
}

int BLOCK(Robot* roger, double time)
{
	stereo_observation(roger, &obs);

	double Larm0, Larm1, Rarm0, Rarm1;
	int left_reach = inv_arm_kinematics_savetotheta(roger, LEFT, roger->base_position[X]+1.0, obs.pos[Y], &Larm0, &Larm1);
	int right_reach = inv_arm_kinematics_savetotheta(roger, RIGHT, roger->base_position[X]+1.0, obs.pos[Y], &Rarm0, &Rarm1);

	if (-M_PI/2.0 <= roger->base_position[THETA] && roger->base_position[THETA] <= M_PI/2.0) {
		if (obs.pos[Y] > roger->base_position[Y]) {
			inv_arm_kinematics_savetotheta(roger, LEFT, roger->base_position[X]-0.2, obs.pos[Y]-2.0, &Larm0, &Larm1);
			rec_setp[RS_BLOCK].arm[LEFT][0] = Larm0;
			rec_setp[RS_BLOCK].arm[LEFT][1] = Larm1;
			rec_setp[RS_BLOCK].base[Y] = obs.pos[Y]-0.2;
		} else {
			inv_arm_kinematics_savetotheta(roger, LEFT, roger->base_position[X]-0.2, obs.pos[Y]+2.0, &Larm0, &Larm1);
			rec_setp[RS_BLOCK].arm[LEFT][0] = Larm0;
			rec_setp[RS_BLOCK].arm[LEFT][1] = Larm1;
			rec_setp[RS_BLOCK].base[Y] = obs.pos[Y]+0.2;
		}
	} else {
		if (obs.pos[Y] > roger->base_position[Y]) {
			inv_arm_kinematics_savetotheta(roger, RIGHT, roger->base_position[X]-0.2, obs.pos[Y]-2.0, &Rarm0, &Rarm1);
			rec_setp[RS_BLOCK].arm[RIGHT][0] = Rarm0;
			rec_setp[RS_BLOCK].arm[RIGHT][1] = Rarm1;
			rec_setp[RS_BLOCK].base[Y] = obs.pos[Y]-0.2;
		} else {
			inv_arm_kinematics_savetotheta(roger, RIGHT, roger->base_position[X]-0.2, obs.pos[Y]+2.0, &Rarm0, &Rarm1);
			rec_setp[RS_BLOCK].arm[RIGHT][0] = Rarm0;
			rec_setp[RS_BLOCK].arm[RIGHT][1] = Rarm1;
			rec_setp[RS_BLOCK].base[Y] = obs.pos[Y]+0.2;
		}
	}

	if (left_reach || right_reach) {
		return TRANSIENT;
	}

	int left_touched = FALSE;
	int right_touched = FALSE;
	int body_touched = FALSE;
	double tolerance = 0.00001;

	double ext_forceL = roger->ext_force[LEFT][1];
	double ext_forceR = roger->ext_force[RIGHT][1];
	double ext_forceBX = roger->ext_force_body[X];
	double ext_forceBY = roger->ext_force_body[Y];

	if (fabs(ext_forceL) > tolerance && IsTactileRedBall(roger, time, LEFT) == TRUE) {
		// printf("%.4f- Left Touched!\n", time);
		left_touched = TRUE;
	}
	if (fabs(ext_forceR) > tolerance && IsTactileRedBall(roger, time, RIGHT) == TRUE) {
		// printf("%.4f- Right Touched!\n", time);
		right_touched = TRUE;
	}
	if ((fabs(ext_forceBX) > tolerance || fabs(ext_forceBY > tolerance))) {
		// printf("%.4f- BODY touched!\n", time);
		body_touched = TRUE;
	} 
	// printf("%3.3f- ext_force(L,R):(%3.3f,%3.3f)   ext_force_body(x,y):(%3.3f,%3.3f)\n",
		// time, ext_forceL, ext_forceR, ext_forceBX, ext_forceBY);

	if (left_touched || right_touched || body_touched) {
		Roger_mode = ROGER_MODE_OFFENSE;
		return CONVERGED;
	}

	return NO_REFERENCE;
}

int SAMPLE_BACKFORTH_BLOCK(Robot* roger, double time)
{
	int internal_state[2];
	internal_state[0] = BLOCK(roger, time);
	internal_state[1] = SAMPLE_BACKFORTH(roger, time);

	int state = internal_state[0];
	// printf("%3.3f, BLOCK:%d  SAMPLE_BACKFORTH:%d  Case:%d\n", time, internal_state[0], internal_state[1], state);
	int return_state;
	switch (state) {
						// BLOCK 								SAMPLE_BACKFORTH
		case 0: // NO_REFERENCE - 			TRANSIENT
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].eye[LEFT] = rec_setp[RS_TURN90].eye[LEFT];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].eye[RIGHT] = rec_setp[RS_TURN90].eye[RIGHT];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[X] = rec_setp[RS_TURN90].base[X];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[Y] = rec_setp[RS_SAMPLE_BACKFORTH].base[Y];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[THETA] = rec_setp[RS_TURN90].base[THETA];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[LEFT][0] = rec_setp[RS_TURN90].arm[LEFT][0];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[LEFT][1] = rec_setp[RS_TURN90].arm[LEFT][1];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[RIGHT][0] = rec_setp[RS_TURN90].arm[RIGHT][0];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[RIGHT][1] = rec_setp[RS_TURN90].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 1: // TRANSIENT - 					TRANSIENT
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].eye[LEFT] = rec_setp[RS_TURN90].eye[LEFT];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].eye[RIGHT] = rec_setp[RS_TURN90].eye[RIGHT];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[X] = rec_setp[RS_TURN90].base[X];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[Y] = rec_setp[RS_BLOCK].base[Y];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[THETA] = rec_setp[RS_TURN90].base[THETA];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[LEFT][0] = rec_setp[RS_BLOCK].arm[LEFT][0];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[LEFT][1] = rec_setp[RS_BLOCK].arm[LEFT][1];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[RIGHT][0] = rec_setp[RS_BLOCK].arm[RIGHT][0];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[RIGHT][1] = rec_setp[RS_BLOCK].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 2: // CONVERGED - 					TRANSIENT
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].eye[LEFT] = rec_setp[RS_SEARCH_TRACK].eye[LEFT];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].eye[RIGHT] = rec_setp[RS_SEARCH_TRACK].eye[RIGHT];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[X] = rec_setp[RS_SEARCH_TRACK].base[X];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[Y] = rec_setp[RS_SEARCH_TRACK].base[Y];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[THETA] = rec_setp[RS_SEARCH_TRACK].base[THETA];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[LEFT][0] = rec_setp[RS_SEARCH_TRACK].arm[LEFT][0];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[LEFT][1] = rec_setp[RS_SEARCH_TRACK].arm[LEFT][1];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[RIGHT][0] = rec_setp[RS_SEARCH_TRACK].arm[RIGHT][0];
			rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[RIGHT][1] = rec_setp[RS_SEARCH_TRACK].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		default:
			break;
	}
	return return_state;
}



int TURN90_SAMPLE_BACKFORTH_BLOCK(Robot* roger, double time)
{
	int internal_state[2];
	internal_state[0] = TURN90(roger, time); // assigns values to rec_setp[0]
	internal_state[1] = SAMPLE_BACKFORTH_BLOCK(roger, time); // assigns values to rec_setp[1]
	int state = internal_state[0]*3 + internal_state[1];
	// printf("%3.3f, TURN90:%d  SAMPLE_BACKFORTH_BLOCK:%d  Case:%d\n", time, internal_state[0], internal_state[1], state);
	int return_state;
	switch (state) {
						// TURN90 					SAMPLE_BACKFORTH_BLOCK
		case 0: // NO_REFERENCE - 	NO_REFERENCE
		case 1: // NO_REFERENCE - 	TRANSIENT
		case 2: // NO_REFERENCE - 	CONVERGED
		case 3: // TRANSIENT - 			NO_REFERENCE
		case 4: // TRANSIENT - 			TRANSIENT
		case 5: // TRANSIENT - 			CONVERGED
			roger->eyes_setpoint[LEFT] = rec_setp[RS_TURN90].eye[LEFT];
			roger->eyes_setpoint[RIGHT] = rec_setp[RS_TURN90].eye[RIGHT];
			roger->base_setpoint[THETA] = rec_setp[RS_TURN90].base[THETA];
			roger->base_setpoint[X] = rec_setp[RS_TURN90].base[X];
			roger->base_setpoint[Y] =	rec_setp[RS_TURN90].base[Y];
			roger->arm_setpoint[LEFT][0] = rec_setp[RS_TURN90].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = rec_setp[RS_TURN90].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = rec_setp[RS_TURN90].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = rec_setp[RS_TURN90].arm[RIGHT][1];
			return_state = TRANSIENT;
			break;
		case 6: // CONVERGED - 			NO_REFERENCE
		case 7: // CONVERGED - 			TRANSIENT
		case 8: // CONVERGED - 			CONVERGED
			roger->eyes_setpoint[LEFT] = rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].eye[LEFT];
			roger->eyes_setpoint[RIGHT] = rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].eye[RIGHT];
			roger->base_setpoint[THETA] = rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[THETA];
			roger->base_setpoint[X] = rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[X];
			roger->base_setpoint[Y] =	rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].base[Y];
			roger->arm_setpoint[LEFT][0] = rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = rec_setp[RS_SAMPLE_BACKFORTH_BLOCK].arm[RIGHT][1];
			return_state = NO_REFERENCE;
			break;
		default:
			break;
	}
	return return_state;
}





int PONG(Robot* roger, double time)
{
	// printf("%3.3f- ROGER_MODE: %d", time, Roger_mode);
	if (Roger_mode == ROGER_MODE_DEFENSE) {
		// printf("  DEFENSE\n");
		TURN90_SAMPLE_BACKFORTH_BLOCK(roger, time);
	} else if (Roger_mode == ROGER_MODE_OFFENSE) {
		// printf("  OFFENSE\n");
		SEARCH_TRACK_CHASE_PUNCH_RETREAT(roger, time);
	}
}

/************************************************************************/
void project9_control(roger, time)
Robot *roger;
double time;
{
	if (time < 0.0010) {
		printf("Recording Home position at time %.4f\n", time);
		if (roger->base_position[X] < 0.0) {
			rec_setp[RS_HOME].base[X] = -4.0;
		} else {
			rec_setp[RS_HOME].base[X] = 4.0;
		}
		// rec_setp[RS_HOME].base[X] = roger->base_position[X];
		rec_setp[RS_HOME].base[Y] = roger->base_position[Y];
		rec_setp[RS_HOME].base[THETA] = roger->base_position[THETA];
		rec_setp[RS_HOME].arm[LEFT][0] = 2.0;
		rec_setp[RS_HOME].arm[LEFT][1] = -2.7;
		rec_setp[RS_HOME].arm[RIGHT][0] = -2.0;
		rec_setp[RS_HOME].arm[RIGHT][1] = 2.7;
		rec_setp[RS_HOME].eye[LEFT] = 0.0;
		rec_setp[RS_HOME].eye[RIGHT] = 0.0;
	}

	if (!bps_created) {
		init_BallPos();
		bps_created = TRUE;
	}

	PONG(roger, time);
}

/************************************************************************/
void project9_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project9_enter_params() 
{
	// Roger_mode = (Roger_mode == ROGER_MODE_DEFENSE) ? ROGER_MODE_OFFENSE : ROGER_MODE_DEFENSE;
	
}

//function called when the 'visualize' button on the gui is pressed
void project9_visualize(roger)
Robot* roger;
{ }

