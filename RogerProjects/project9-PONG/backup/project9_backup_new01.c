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
		PredictPath()
		Intercept()

***/

#define RS_HOME 														0
#define RS_SEARCH 													1
#define RS_TRACK 														2
#define RS_SEARCH_TRACK 										3
#define RS_CHASE 														4
#define RS_GRAB															5
#define RS_CHASE_GRAB 											6
#define RS_CENTER 													7
#define RS_AIM 															8
#define RS_PUNCH 														9
#define RS_CENTER_AIM_PUNCH 								10
#define RS_CHASE_GRAB_CENTER_AIM_PUNCH			11
#define RS_RETREAT 													12
#define RS_VELOCITY 												13
#define RS_PREDICT 													14
#define RS_INTERCEPT 												15
#define RS_BLOCK														16
#define BALL_RND	-0.0
#define NBINS_BPS 50

typedef struct _setpoints {    /* DO NOT ALTER */
  double base[3];             /* (x,y,theta) of the base in world frame */
  double arm[NARMS][NARM_JOINTS]; /* arm joint angles */
  double eye[NEYES];         /* eye verge angle  relative to base frame */
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

int tt_search = 100;
int tt_search_interval = 200;
double prev_heading = 0.0;
int SEARCH(Robot* roger, double time, SetPoints* rec_setp, Observation* obs, BallPos* bps)
{
	tt_search++;
	double heading = 0.0;

	if (tt_search == 100) {
		rec_setp[RS_SEARCH].eye[LEFT] = -M_PI;
		rec_setp[RS_SEARCH].eye[RIGHT] = -M_PI;
	}
	if (tt_search == 200) {
		rec_setp[RS_SEARCH].eye[LEFT] = M_PI;
		rec_setp[RS_SEARCH].eye[RIGHT] = M_PI;
	}

	if (tt_search % tt_search_interval != 0) {
		return TRANSIENT;
	}
	tt_search = 0;

	// if (tt_search % 50 == 0) {
	// 	rec_setp[RS_SEARCH].eye[LEFT] = -rec_setp[RS_SEARCH].eye[LEFT];
	// 	rec_setp[RS_SEARCH].eye[RIGHT] = -rec_setp[RS_SEARCH].eye[RIGHT];
	// }

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
		// rec_setp[RS_SEARCH].eye[LEFT] = left_gaze;
		// rec_setp[RS_SEARCH].eye[RIGHT] = right_gaze;

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

int TRACK(Robot* roger, double time, SetPoints* rec_setp, Observation* obs, BallPos* bps)
{
	double ul = -1.0;
	double ur = -1.0;

	/** if a red pixel is detected on both eyes **/
	if ( average_red_pixel(roger, &ul, &ur) == TRUE ) {
		/** get the eye angle error **/
		double eye_theta_error_l = atan2((ul - 63.5), FOCAL_LENGTH);
		double eye_theta_error_r = atan2((ur - 63.5), FOCAL_LENGTH);
		/** foveate **/
		rec_setp[RS_TRACK].eye[LEFT] = roger->eye_theta[LEFT] + eye_theta_error_l;
		rec_setp[RS_TRACK].eye[RIGHT] = roger->eye_theta[RIGHT] + eye_theta_error_r;
		rec_setp[RS_TRACK].base[THETA] = roger->base_position[THETA] + (roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]) / 2.0;

		double eye_diff = fabs(roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]);
		double epsilon = 0.3;
		if (eye_diff < epsilon) {
			return CONVERGED;
		} else {
			/** if foveated **/
			return TRANSIENT;
		}
	} else {
		/** if no red pixel is detected **/
		return NO_REFERENCE;
	}
}

int SEARCHTRACK(Robot* roger, double time, SetPoints* rec_setp, Observation* obs, BallPos* bps)
{
	/***********************************************************************************************/
	/* internal_state=[ 0:SEARCH 1:TRACK ] */
	/***********************************************************************************************/
	int internal_state[2];
	internal_state[0] = SEARCH(roger, time, rec_setp, &obs, &bps); // assigns values to rec_setp[0]
	internal_state[1] = TRACK(roger, time, rec_setp, &obs, &bps); // assigns values to rec_setp[1]
	int state = internal_state[1]*3 + internal_state[0];
	printf("%3.3f, SEARCH:%d  TRACK:%d  Case:%d\n", time, internal_state[0], internal_state[1], state);
	int return_state;
	switch (state) {
						// Track 					SEARCH
		case 0: // NO_REFERENCE - NO_REFERENCE
		case 1: // NO_REFERENCE - TRANSIENT
		case 2: // NO_REFERENCE - CONVERGED
			roger->eyes_setpoint[LEFT] = rec_setp[RS_SEARCH].eye[LEFT];
			roger->eyes_setpoint[RIGHT] = rec_setp[RS_SEARCH].eye[RIGHT];
			roger->base_setpoint[THETA] = rec_setp[RS_SEARCH].base[THETA];
			return_state = TRANSIENT;
			break;
	
		case 3: // TRANSIENT - NO_REFERENCE
		case 4: // TRANSIENT - TRANSIENT
		case 5: // TRANSIENT - CONVERGED
			roger->eyes_setpoint[LEFT] = rec_setp[RS_TRACK].eye[LEFT];
			roger->eyes_setpoint[RIGHT] = rec_setp[RS_TRACK].eye[RIGHT];
			roger->base_setpoint[THETA] = rec_setp[RS_TRACK].base[THETA];
			return_state = TRANSIENT;
			break;
	
		case 6: // CONVERGED - NO_REFERENCE
		case 7: // CONVERGED - TRANSIENT
		case 8: // CONVERGED - CONVERGED
			roger->eyes_setpoint[LEFT] = rec_setp[RS_TRACK].eye[LEFT];
			roger->eyes_setpoint[RIGHT] = rec_setp[RS_TRACK].eye[RIGHT];
			roger->base_setpoint[THETA] = rec_setp[RS_TRACK].base[THETA];
			return_state = CONVERGED;
			break;
		default:
		break;
	}
	return return_state;
}





int PONG(Robot* roger, double time, SetPoints* rec_setp, Observation* obs, BallPos* bps)
{
	return TRANSIENT;
}

/************************************************************************/
void project9_control(roger, time)
Robot *roger;
double time;
{
	if (time < 0.0010) {
		printf("Recording Home position at time %.4f\n", time);
		rec_setp[RS_HOME].base[X] = roger->base_position[X];
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
		init_BallPos(&bps);
		bps_created = TRUE;
	}

	SEARCHTRACK(roger, time, rec_setp, &obs, &bps);
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

