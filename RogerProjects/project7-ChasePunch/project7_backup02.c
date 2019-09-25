/*************************************************************************/
/* File:        project7.c                                               */
/* Description: User project #7 - empty project directory for project    */
/*              development                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

#define R_CHASE 0
#define R_TOUCH 1
#define LEFT_JOINT_0 2
#define LEFT_JOINT_1 3
#define RIGHT_JOINT_0 4
#define RIGHT_JOINT_1 5

SetPoint recommended_setpoints[2];

// recommended_setpoints[0]: CHASE()
// recommended_setpoints[1]: TOUCH()
// recommended_setpoints[.][0] : x
// recommended_setpoints[.][1] : y
// recommended_setpoints[.][2] : arm_left_joint_0
// recommended_setpoints[.][3] : arm_left_joint_1
// recommended_setpoints[.][4] : arm_right_joint_0
// recommended_setpoints[.][5] : arm_right_joint_1

// double recommended_setpoints[2][6];

int CHASE(roger, time)
Robot* roger;
double time;
{
	/***
	CHASE()
	{
		if ( SEARCHTRACK() == CONVERGED ) {
			x, y = stereo_observation()
			set base_setpoint to x, y
			base_err = base_setpoint - base_position[x, y]
			epsilon = Roger's arm length
			if (base_err > epsilon) { return TRANSIENT }
			else { return CONVERGED }
		}

		return NO_REFERENCE
	}
	***/

	// printf("base_setpoint:(%.4f, %.4f)\n", roger->base_setpoint[X], roger->base_setpoint[Y]);

	int status = SEARCHTRACK();
	// printf("SEARCHTRACK(): %d\n", status);
	if (status == CONVERGED) {
	// 	printf("SEARCHTRACK(): CONVERGED\n");
		Observation obs;
		stereo_observation(roger, &obs);
		recommended_setpoints[R_CHASE].base[X] = obs.pos[X];
		recommended_setpoints[R_CHASE].base[Y] = obs.pos[Y];

	// 	// printf("base_setpoint:(%.4f, %.4f)  obs:(%.4f, %.4f)\n", 
	// 	// 	roger->base_setpoint[X], roger->base_setpoint[Y],
	// 	// 	obs.pos[X], obs.pos[Y]);

		double X_err = roger->base_setpoint[X] - roger->base_position[X];
		double Y_err = roger->base_setpoint[Y] - roger->base_position[Y];
		double base_err = sqrt(X_err*X_err + Y_err*Y_err);
		double base_offset = 1.0;
		double epsilon = L_ARM1 + L_ARM2 + base_offset;
		if (base_err > epsilon) { return TRANSIENT; }
		else { return CONVERGED; }
	}

	return NO_REFERENCE;
}

int TOUCH(roger, time)
Robot* roger;
double time;
{
	/***
	TOUCH()
	{
		if SEARCHTRACK() != CONVERGED
			return NO_REFERENCE
		x = base_pos[X]
		y = base_pos[Y]
		left_reach = IK(theta1_left, theta2_left)
		right_reach = IK(theta1_right, theta2_right)

		if (left_reach == FALSE && right_reach == FALSE)
			return NO_REFERENCE
	

		if( left reach ) {
			arm_left_setpoint = theta1, theta2 + offset
		}

		if ( left_reach == TRUE && right_reach == TRUE ) {
			arm_setpoint = x, y
			fingertip_err = ball position - current x,y from ForwrdKinematic()
			epsilon = constant
			if (fingertip_err > epsilon) { return TRANSIENT }
			else { return CONVERGED }
		}
		return NO_REFERENCE
	}
	***/

	// roger->ext_force[LEFT][0] = 0.0;
	// roger->ext_force[LEFT][1] = 0.0;
	// roger->ext_force[RIGHT][0] = 0.0;
	// roger->ext_force[RIGHT][1] = 0.0;
	// roger->ext_force_body[0] = 0.0;
	// roger->ext_force_body[1] = 0.0;
	// printf("%f\n", roger->ext_force[LEFT][0]);
	// printf("%f\n", roger->ext_force[LEFT][1]);
	// printf("%f\n", roger->ext_force[RIGHT][0]);
	// printf("%f\n", roger->ext_force[RIGHT][1]);
	// printf("ext_force: L:(%.4f, %.4f) R:(%.4f, %.4f)\n", roger->ext_force[LEFT][0], roger->ext_force[LEFT][1], roger->ext_force[RIGHT][0], roger->ext_force[RIGHT][1]);

	if (SEARCHTRACK() != CONVERGED) { return NO_REFERENCE; }

	double x = roger->base_setpoint[X];
	double y = roger->base_setpoint[Y];
	// printf("base_setpoint:(%.4f, %.4f)\n", x, y);
	double theta1_left = 0.0, theta2_left = 0.0, theta1_right = 0.0, theta2_right = 0.0;
	int left_reach = inv_arm_kinematics(roger, LEFT, x, y, &theta1_left, &theta2_left);
	int right_reach = inv_arm_kinematics(roger, RIGHT, x, y, &theta1_right, &theta2_right);

	if (left_reach == FALSE && right_reach == FALSE) { return NO_REFERENCE; }

	double epsilon = 0.1;

	int left_touched = FALSE;
	int right_touched = FALSE;
	if (left_reach == TRUE) {
		double left_joint_0_offset = 0.1;
		double left_joint_1_offset = 0.1;
		recommended_setpoints[R_TOUCH].arm[LEFT][0] = theta1_left + left_joint_0_offset;
		recommended_setpoints[R_TOUCH].arm[LEFT][1] = theta2_left + left_joint_1_offset;
		// recommended_setpoints[R_TOUCH][LEFT_JOINT_0] = theta1_left + left_joint_0_offset;
		// recommended_setpoints[R_TOUCH][LEFT_JOINT_1] = theta2_left + left_joint_1_offset;
		double finger_left_x = 0.0;
		double finger_left_y = 0.0;
		fwd_arm_kinematics(roger->arm_theta[LEFT][0], roger->arm_theta[LEFT][1], &finger_left_x, &finger_left_y);
		double ferr_x = roger->base_setpoint[X] - finger_left_x;
		double ferr_y = roger->base_setpoint[X] - finger_left_y;
		double finger_err_left = sqrt(ferr_x*ferr_x + ferr_y*ferr_y);
		if (finger_err_left < epsilon) {
			left_touched = TRUE;
		}
	}
	if (right_reach == TRUE) {
		double right_joint_0_offset = -0.1;
		double right_joint_1_offset = -0.1;
		recommended_setpoints[R_TOUCH].arm[RIGHT][0] = theta1_right + right_joint_0_offset;
		recommended_setpoints[R_TOUCH].arm[RIGHT][1] = theta2_right + right_joint_1_offset;
		// recommended_setpoints[R_TOUCH][RIGHT_JOINT_0] = theta1_right + right_joint_0_offset;
		// recommended_setpoints[R_TOUCH][RIGHT_JOINT_1] = theta2_right + right_joint_1_offset;
		double finger_right_x = 0.0;
		double finger_right_y = 0.0;
		fwd_arm_kinematics(roger->arm_theta[RIGHT][0], roger->arm_theta[RIGHT][1], &finger_right_x, &finger_right_y);
		double ferr_x = roger->base_setpoint[X] - finger_right_x;
		double ferr_y = roger->base_setpoint[X] - finger_right_y;
		double finger_err_right = sqrt(ferr_x*ferr_x + ferr_y*ferr_y);
		if (finger_err_right < epsilon) {
			right_touched = TRUE;
		}
	}
	// printf("%.4f, left_reach:%d  right_reach:%d\n", time, left_reach, right_reach);

	if (left_touched == TRUE && right_touched == TRUE) {
		return CONVERGED;
	} else {
		return TRANSIENT;
	}

	return NO_REFERENCE;
}

int CHASETOUCH(roger, time)
Robot* roger;
double time;
{

	// // /***********************************************************************************************/
	// // /* internal_state=[ 0:CHASE 1:TOUCH ] */
	// // /***********************************************************************************************/
	int internal_state[2];
	internal_state[0] = CHASE(roger, time); // assigns values to recommended_setpoints[0]
	internal_state[1] = TOUCH(roger, time); // assigns values to recommended_setpoints[1]
	// for N=2
	int state = internal_state[1]*3 + internal_state[0];
	int return_state;
	double home_1[2][2] = {{2.0, -1.5}, {-2.0, 1.5}};
	double home_2[2][2] = {{3.0, -2.7}, {-3.0, 2.7}};
	switch (state) {
						// TOUCH 					CHASE
		case 0: // NO_REFERENCE - NO_REFERENCE
			// If no reference for CHASE(), then stop moving to previous setpoints.
			roger->base_setpoint[X] = roger->base_position[X];
			roger->base_setpoint[Y] = roger->base_position[Y];
			roger->arm_setpoint[LEFT][0] = home_2[0][0];
			roger->arm_setpoint[LEFT][1] = home_2[0][1];
			roger->arm_setpoint[RIGHT][0] = home_2[1][0];
			roger->arm_setpoint[RIGHT][1] = home_2[1][1];
			// roger->arm_setpoint[LEFT][0] = home_1[LEFT][0];
			// roger->arm_setpoint[LEFT][1] = home_1[LEFT][1];
			// roger->arm_setpoint[RIGHT][0] = home_1[RIGHT][0];
			// roger->arm_setpoint[RIGHT][1] = home_1[RIGHT][1];
			printf("%.4f, Chase:%d  Touch:%d Case 1 2 3\n", time, internal_state[0], internal_state[1]);
		break;
		case 1: // NO_REFERENCE - TRANSIENT
		case 2: // NO_REFERENCE - CONVERGED

			// TOUCH = NO_R -> Ball is out of reach. Set arms to "Home" position.

			// printf("%f %f\n", recommended_setpoints[R_CHASE][X], recommended_setpoints[R_CHASE][Y]);
			roger->base_setpoint[X] = recommended_setpoints[R_CHASE].base[X];
			roger->base_setpoint[Y] = recommended_setpoints[R_CHASE].base[Y];
			roger->arm_setpoint[LEFT][0] = home_2[0][0];
			roger->arm_setpoint[LEFT][1] = home_2[0][1];
			roger->arm_setpoint[RIGHT][0] = home_2[1][0];
			roger->arm_setpoint[RIGHT][1] = home_2[1][1];

			// roger->base_setpoint[THETA] = recommended_setpoints[0][THETA];
			// roger->eyes_setpoint[LEFT] = recommended_setpoints[0][LEFT];
			// roger->eyes_setpoint[RIGHT] = recommended_setpoints[0][RIGHT];
			printf("%.4f, Chase:%d  Touch:%d Case 1 2 3\n", time, internal_state[0], internal_state[1]);
			return_state = TRANSIENT;
		break;
	
		case 3: // TRANSIENT - NO_REFERENCE
		case 4: // TRANSIENT - TRANSIENT
		case 5: // TRANSIENT - CONVERGED
			roger->base_setpoint[X] = recommended_setpoints[R_CHASE].base[X];
			roger->base_setpoint[Y] = recommended_setpoints[R_CHASE].base[Y];
			roger->arm_setpoint[LEFT][0] = recommended_setpoints[R_TOUCH].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = recommended_setpoints[R_TOUCH].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = recommended_setpoints[R_TOUCH].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = recommended_setpoints[R_TOUCH].arm[RIGHT][1];
			// roger->base_setpoint[THETA] = recommended_setpoints[1][THETA];
			// roger->eyes_setpoint[LEFT] = recommended_setpoints[1][LEFT];
			// roger->eyes_setpoint[RIGHT] = recommended_setpoints[1][RIGHT];
			printf("%.4f, Chase:%d  Touch:%d Case 3 4 5\n", time, internal_state[0], internal_state[1]);
			return_state = TRANSIENT;
		break;
	
		case 6: // CONVERGED - NO_REFERENCE
		case 7: // CONVERGED - TRANSIENT
			// roger->base_setpoint[X] = recommended_setpoints[R_CHASE][X];
			// roger->base_setpoint[Y] = recommended_setpoints[R_CHASE][Y];
			roger->arm_setpoint[LEFT][0] = recommended_setpoints[R_TOUCH].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = recommended_setpoints[R_TOUCH].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = recommended_setpoints[R_TOUCH].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = recommended_setpoints[R_TOUCH].arm[RIGHT][1];
			// roger->base_setpoint[THETA] = recommended_setpoints[1][THETA];
			// roger->eyes_setpoint[LEFT] = recommended_setpoints[1][LEFT];
			// roger->eyes_setpoint[RIGHT] = recommended_setpoints[1][RIGHT];
			// printf("%.4f, Chase:%d  Touch:%d Case 6 7 8\n", time, internal_state[0], internal_state[1]);
			// return_state = CONVERGED;
			// break;
		case 8: // CONVERGED - CONVERGED
			roger->base_setpoint[X] = roger->base_position[X];
			roger->base_setpoint[Y] = roger->base_position[Y];
			roger->arm_setpoint[LEFT][0] = recommended_setpoints[R_TOUCH].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = recommended_setpoints[R_TOUCH].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = recommended_setpoints[R_TOUCH].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = recommended_setpoints[R_TOUCH].arm[RIGHT][1];
			printf("%.4f, Chase:%d  Touch:%d Case 6 7 8\n", time, internal_state[0], internal_state[1]);
			return_state = CONVERGED;
			break;
		default:
		break;
	}
	return return_state;
}



void project7_control(roger, time)
Robot* roger;
double time;
{ 
	// CHASE(roger, time);
	// printf("ASDFASDFASF");
	CHASETOUCH(roger, time);
}

/************************************************************************/
void project7_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project7_enter_params() 
{
  // printf("Project 7 enter_params called. \n");
}

//function called when the 'visualize' button on the gui is pressed
void project7_visualize(roger)
Robot* roger;
{ }
