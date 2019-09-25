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
#define RS_TOUCH 1
#define RS_PUNCH 2
#define RS_HOME 3

SetPoint recommended_setpoints[4];
Observation obs;

int state_chase = NO_REFERENCE;
int state_aim = NO_REFERENCE;
int state_punch = NO_REFERENCE;
double base_err_previous = 0.0;
double home[3];

int RogerArmPunch(Robot* roger, double time)
{
	// printf("RogerArmPunch\n");
	roger->arm_setpoint[LEFT][0] = recommended_setpoints[RS_PUNCH].arm[LEFT][0];
	roger->arm_setpoint[LEFT][1] = recommended_setpoints[RS_PUNCH].arm[LEFT][1];
	roger->arm_setpoint[RIGHT][0] = recommended_setpoints[RS_PUNCH].arm[RIGHT][0];
	roger->arm_setpoint[RIGHT][1] = recommended_setpoints[RS_PUNCH].arm[RIGHT][1];
	return TRUE;
}

int CHASEforPUNCH(roger, time)
Robot* roger;
double time;
{
	int status = SEARCHTRACK(roger, time);  // if no log needed, this should be a local variable.
	printf("%.4f - status from SEARCHTRACK():%d\n", status);
	if (status == CONVERGED) {
		stereo_observation(roger, &obs);
		recommended_setpoints[RS_CHASE].base[X] = obs.pos[X];
		recommended_setpoints[RS_CHASE].base[Y] = obs.pos[Y];

		double X_err = obs.pos[X] - roger->base_position[X];
		double Y_err = obs.pos[Y] - roger->base_position[Y];
		double base_err = sqrt(X_err*X_err + Y_err*Y_err);
		double base_offset = 0.85;
		double base_err_tolerane = 0.0;
		double epsilon = R_BALL + R_BASE + base_offset;
		printf("%.4f- base_err:%.4f  epsilon:%.4f\n", time, base_err, epsilon);
		if (base_err > epsilon) { return TRANSIENT; }
		else { return CONVERGED; }
	}
	return NO_REFERENCE;
}


int AIM(Robot* roger, double time)
{
	// double aim_pose[NARMS][NARM_JOINTS] = { 3.14 - 0.8 - 0.01, 0.0, -3.14 + 0.8 + 0.01, 0.0 };
	// double aim_pose[NARMS][NARM_JOINTS] = { 3.10, -2.0, 3.10, 2.0 };
	double aim_pose[NARMS][NARM_JOINTS] = { -1.5, -1.5, -3.0, 1.5 };
	if (state_aim != CONVERGED) {
		recommended_setpoints[RS_TOUCH].arm[LEFT][0] = aim_pose[LEFT][0];
		recommended_setpoints[RS_TOUCH].arm[LEFT][1] = aim_pose[LEFT][1];
		recommended_setpoints[RS_TOUCH].arm[RIGHT][0] = aim_pose[RIGHT][0];
		recommended_setpoints[RS_TOUCH].arm[RIGHT][1] = aim_pose[RIGHT][1];

		double tolerance = 0.1;
		double eL0 = fabs(roger->arm_setpoint[LEFT][0] - roger->arm_theta[LEFT][0]);
		double eL1 = fabs(roger->arm_setpoint[LEFT][1] - roger->arm_theta[LEFT][1]);
		double eR0 = fabs(roger->arm_setpoint[RIGHT][0] - roger->arm_theta[RIGHT][0]);
		double eR1 = fabs(roger->arm_setpoint[RIGHT][1] - roger->arm_theta[RIGHT][1]);
		if (eL0 < tolerance && eL1 < tolerance && eR0 < tolerance && eR1 < tolerance) {
			state_aim = CONVERGED;
			return CONVERGED;
		}
		state_aim = TRANSIENT;
		return TRANSIENT;
	}
}

int PUNCH(Robot* roger, double time)
{
	// double punch_pose[NARMS][NARM_JOINTS] = { 0.0, 0.0, 0.0, 0.0 };
	double punch_pose[NARMS][NARM_JOINTS] = { 1.6 , 1.6, 0.1, 1.0 };

	if (state_aim == CONVERGED) {
		// printf("punch!\n");
		recommended_setpoints[RS_PUNCH].arm[LEFT][0] = punch_pose[LEFT][0];
		recommended_setpoints[RS_PUNCH].arm[LEFT][1] = punch_pose[LEFT][1];
		recommended_setpoints[RS_PUNCH].arm[RIGHT][0] = punch_pose[RIGHT][0];
		recommended_setpoints[RS_PUNCH].arm[RIGHT][1] = punch_pose[RIGHT][1];

		int left_touched = FALSE;
		int right_touched = FALSE;
		double tolerance = 0.001;
		if (fabs(roger->ext_force[LEFT][0]) > tolerance && IsTactileRedBall(roger, time, LEFT) == TRUE) {
			// printf("%.4f- Left Touched!\n", time);
			left_touched = TRUE;
		}
		if (fabs(roger->ext_force[RIGHT][0]) > tolerance && IsTactileRedBall(roger, time, RIGHT) == TRUE) {
			// printf("%.4f- Right Touched!\n", time);
			right_touched = TRUE;
		}
		if (left_touched == TRUE && right_touched == TRUE) { state_punch = CONVERGED; return CONVERGED; }

		state_punch = TRANSIENT;
		return TRANSIENT;
	}
	state_punch = NO_REFERENCE;
	return NO_REFERENCE;
}

int CHASEPUNCH(Robot* roger, double time)
{
	int internal_state[3];
	internal_state[0] = CHASEforPUNCH(roger, time);
	internal_state[1] = AIM(roger, time);
	internal_state[2] = PUNCH(roger, time);
	int state = internal_state[0]*9 + internal_state[1]*3 + internal_state[2];
	// printf("%.4f- CHASE:%d  AIM:%d  PUNCH:%d  state:%d\n", time, internal_state[0], internal_state[1], internal_state[2], state);
	int return_state;
	switch (state) {
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
			return_state = NO_REFERENCE;
			break;
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
		case 16:
			RogerBaseMove(roger, time);
			RogerArmMove(roger, time);
			return_state = TRANSIENT;
			break;
		case 17:
			RogerBaseMove(roger, time);
			RogerArmPunch(roger, time);
			return_state = TRANSIENT;
			break;
		case 18:
		case 19:
		case 20:
		case 21:
		case 22:
		case 23:
			RogerBaseStay(roger, time);
			RogerArmPunch(roger, time);
			return_state = TRANSIENT;
			break;
		case 24:
		case 25:
			RogerBaseStay(roger, time);
			RogerArmPunch(roger, time);
			return_state = TRANSIENT;
			break;
		case 26:
			RogerBaseStay(roger, time);
			RogerArmPunch(roger, time);
			return_state = CONVERGED;
			break;
		default:
		break;
	}
	return return_state;
}

int ballseen = FALSE;
int RETREAT(Robot* roger, double time)
{
	if (SEARCHTRACK(roger, time) == CONVERGED)
	{
		ballseen = TRUE;
		CHASETOUCH(roger, time);
		return TRUE;
	} 

	if (ballseen && SEARCHTRACK(roger, time) != CONVERGED) {
		double pos_x = roger->base_position[X] - home[X];
		double pos_y = roger->base_position[Y] - home[Y];
		double pos_t = roger->base_position[THETA];
		double angle = atan2(pos_y, pos_x);
		// printf("%.4f- base_theta:%.4f  angle:%.4f\n", time, pos_t, angle);

		roger->base_setpoint[THETA] = angle;
		roger->base_setpoint[X] = -pos_x;
		roger->base_setpoint[Y] = -pos_y;
	}

	return TRUE;
}

int OBSERVEBALL(Robot* roger, double time)
{
	// get ball position every t time
	// if 2 positions detected, then CONVERGED

	// if time == 0
	//    observed_two_ballpos = FALSE
	// if (SEARCHTRACK(roger, time) == CONVERGED) {
	// 	if (!observed_two_ballpos) {
			// increase tiktok
			// if tiktok == 1
			//   ballpos_1st
			// if tiktok == 2
			//   ballpos 2nd
			//   observed_two_ballpos = TRUE
			//   tiktok = 0
		// } else {

			// observed_two_ballpos = TRUE;
		// }
	// }
}

int PREDICTPATH(Robot* roger, double time)
{
	// calculate velocity of the ball
	// calculate the end position of the ball
}

int INTERCEPT(Robot* roger, double time)
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
int BLOCK(Robot* roger, double time)
{
	int interval = 40;

	if (SEARCHTRACK(roger, time) == CONVERGED) {
		tiktok++;
		if (tiktok == interval * 2) {
			block_observed = FALSE;
			tiktok = 0;
			block_ballpos[0][0] = 0.0;
			block_ballpos[0][1] = 0.0;
			block_ballpos[1][0] = 0.0;
			block_ballpos[1][1] = 0.0;
		}
		if (!block_observed) {
			stereo_observation(roger, &obs);
			if (tiktok == interval * 0) {
				block_ballpos[0][0] = obs.pos[X];
				block_ballpos[0][1] = obs.pos[Y];
			} else if (tiktok == interval * 1 ) {
				block_ballpos[1][0] = obs.pos[X];
				block_ballpos[1][1] = obs.pos[Y];
			} 
		} else {
			printF("Hahahaha\t\t\t\tHAHAHAHA\n");
		}
	}

	printf("%3.3f- block_ballpos[0]:(%3.3f,%3.3f)  [1]:(%3.3f,%3.3f)  tiktok:%d\n", 
			time, block_ballpos[0][0], block_ballpos[0][1], block_ballpos[1][0], block_ballpos[1][1], tiktok);
	// if (!observed)
	//    observe two positions of the ball
	// else
	//    if (!predicted)
	//       calculate the end point of the ball
	//    else
	//        if (!intercept)
	//            try to intercept the ball




}






// int BLOCK(Robot* roger, double time)
// {
// 	int ticktock = 0;
// 	int internal_state[3];
// 	internal_state[0] = OBSERVEBALL(roger, time); // assigns values to recommended_setpoints[0]
// 	internal_state[1] = PREDICTPATH(roger, time);
// 	internal_state[2] = INTERCEPT(roger, time);

// 	int state = internal_state[0]*9 + internal_state[1]*3 + internal_state[2];
// 	int return_state;

// 	switch (state) {
// 		case 0:
// 		case 1:
// 		case 2:
// 		case 3:
// 		case 4:
// 		case 5:
// 		case 6:
// 		case 7:
// 		case 8:
// 			return_state = NO_REFERENCE;
// 			break;
// 		case 9:
// 		case 10:
// 		case 11:
// 		case 12:
// 		case 13:
// 		case 14:
// 		case 15:
// 		case 16:
// 		case 17:
// 		case 18:
// 		case 19:
// 		case 20:
// 		case 21:
// 		case 22:
// 		case 23:
// 			return_state = TRANSIENT;
// 			break;
// 		case 24:
// 		case 25:
// 			return_state = TRANSIENT;
// 			break;
// 		case 26:
// 			return_state = CONVERGED;
// 		default:
// 			break;
// 	}
// }


/************************************************************************/
void project9_control(roger, time)
Robot *roger;
double time;
{
	// if (time < 0.0010) {
	// 	printf("Recording Home position at time %.4f\n", time);
	// 	home[X] = roger->base_position[X];
	// 	home[Y] = roger->base_position[Y];
	// 	home[THETA] = roger->base_position[THETA];
	// }

	// setpoint_filter(roger);

	BLOCK(roger, time);
	// RETREAT(roger, time);
	// CHASEPUNCH(roger, time);

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
  printf("Project 9 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project9_visualize(roger)
Robot* roger;
{ }

