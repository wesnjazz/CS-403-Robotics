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

#define P7_CHASE 0
#define P7_TOUCH 1

SetPoint recommended_setpoints[2];
Observation obs;

int IsTactileRedBall(roger, time, limb)
Robot* roger;
double time;
int limb;
{
	/**
	filter to determine if the external forces is due to the red ball.
	IsTactileRedBall( roger, time )
		get x, y position of the arm
		convert arm position into world frame
		distance = sqrt( (ball_x-arm_x)^2 + (ball_y-arm_y)^2 )
		if distance < Ball_radius
			return TRUE
		else
			return FALSE
	**/

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

	if (d < R_BALL + 0.5) { return TRUE; }
	else { return FALSE; }
}

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
	// printf("%f\n", roger->base_position[THETA]);

	int status = SEARCHTRACK(roger, time);
	if (status == CONVERGED) {
		// printf("%.4f, SEARCHTRACK(): CONVERGED\n");
		stereo_observation(roger, &obs);
		recommended_setpoints[P7_CHASE].base[X] = obs.pos[X];
		recommended_setpoints[P7_CHASE].base[Y] = obs.pos[Y];

	// 	// printf("base_setpoint:(%.4f, %.4f)  obs:(%.4f, %.4f)\n", 
	// 	// 	roger->base_setpoint[X], roger->base_setpoint[Y],
	// 	// 	obs.pos[X], obs.pos[Y]);

		// Observation obsBframe;
		// stereo_observation_Bframe(roger, &obsBframe);

		double X_err = obs.pos[X] - roger->base_position[X];
		double Y_err = obs.pos[X] - roger->base_position[X];
		// double X_err = roger->base_setpoint[X] - roger->base_position[X];
		// double Y_err = roger->base_setpoint[Y] - roger->base_position[Y];
		double base_err = sqrt(X_err*X_err + Y_err*Y_err);
		double base_offset = 0.25;
		double epsilon = R_BALL + R_BASE + base_offset;
		// printf("obs:(%.4f, %.4f)  base_pos:(%.4f, %.4f)  base_err:%.4f\n", obs.pos[X], obs.pos[Y], roger->base_position[X], roger->base_position[Y], base_err);
		// double epsilon = L_ARM1 + L_ARM2 + base_offset;
		// printf("%.4f- distance from ball and base: %.4f\n", time, base_err);
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

		if left_reach == TRUE
			arm_left_recommendpoint = theta1, theta2 + offset
			if there is an external forces
				if it is from RED_BALL
					left_touched = TRUE

		if right_reach == TRUE
			arm_right_recommendpoint = theta1, theta2 + offset
			if ext_force is detected on right hand
				right_touched = TRUE
				if it is from RED_BALL
					right_touched = TRUE

		if left_touched == TRUE && right_touched == TRUE
			return CONVERGED

		return TRANSIENT
	}
	***/

	// printf("ext_force: L:(%.4f, %.4f) R:(%.4f, %.4f)\n", roger->ext_force[LEFT][0], roger->ext_force[LEFT][1], roger->ext_force[RIGHT][0], roger->ext_force[RIGHT][1]);
	// printf("%.4f-Tactile Sensor: Left:(%.4f, %.4f), Right:(%.4f, %.4f)\n", 
	// 	time,
	// 	roger->ext_force[LEFT][0], roger->ext_force[LEFT][1],
	// 	roger->ext_force[RIGHT][0], roger->ext_force[RIGHT][1]);

	if (SEARCHTRACK(roger, time) != CONVERGED) { return NO_REFERENCE; }

	double x = roger->base_setpoint[X];
	double y = roger->base_setpoint[Y];
	// printf("base_setpoint:(%.4f, %.4f)\n", x, y);
	double theta1_left = 0.0, theta2_left = 0.0, theta1_right = 0.0, theta2_right = 0.0;
	int left_reach = inv_arm_kinematics_savetotheta(roger, LEFT, x, y, &theta1_left, &theta2_left);
	int right_reach = inv_arm_kinematics_savetotheta(roger, RIGHT, x, y, &theta1_right, &theta2_right);

	if (left_reach == FALSE && right_reach == FALSE) { return NO_REFERENCE; }

	double epsilon = 0.1;

	int left_touched = FALSE;
	int right_touched = FALSE;
	
	if (left_reach == TRUE && right_reach == TRUE) {
		double left_joint_0_offset = 0.2;
		double left_joint_1_offset = 0.2;
		double right_joint_0_offset = -0.2;
		double right_joint_1_offset = -0.2;
		recommended_setpoints[P7_TOUCH].arm[LEFT][0] = theta1_left + left_joint_0_offset;
		recommended_setpoints[P7_TOUCH].arm[LEFT][1] = theta2_left + left_joint_1_offset;
		recommended_setpoints[P7_TOUCH].arm[RIGHT][0] = theta1_right + right_joint_0_offset;
		recommended_setpoints[P7_TOUCH].arm[RIGHT][1] = theta2_right + right_joint_1_offset;
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


	// if (left_reach == TRUE) {
	// 	double left_joint_0_offset = 0.2;
	// 	double left_joint_1_offset = 0.2;
	// 	recommended_setpoints[P7_TOUCH].arm[LEFT][0] = theta1_left + left_joint_0_offset;
	// 	recommended_setpoints[P7_TOUCH].arm[LEFT][1] = theta2_left + left_joint_1_offset;

	// 	double tolerance = 0.001;
	// 	if (fabs(roger->ext_force[LEFT][0]) > tolerance && IsTactileRedBall(roger, time, LEFT) == TRUE) {
	// 		// printf("%.4f- Left Touched!\n", time);
	// 		left_touched = TRUE;
	// 	}
	// }

	// if (right_reach == TRUE) {
	// 	double right_joint_0_offset = -0.2;
	// 	double right_joint_1_offset = -0.2;
	// 	recommended_setpoints[P7_TOUCH].arm[RIGHT][0] = theta1_right + right_joint_0_offset;
	// 	recommended_setpoints[P7_TOUCH].arm[RIGHT][1] = theta2_right + right_joint_1_offset;

	// 	double tolerance = 0.001;
	// 	if (fabs(roger->ext_force[RIGHT][0]) > tolerance && IsTactileRedBall(roger, time, RIGHT) == TRUE) {
	// 		// printf("%.4f- Right Touched!\n", time);
	// 		right_touched = TRUE;
	// 	}
	// }

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
	internal_state[0] = TOUCH(roger, time); // assigns values to recommended_setpoints[1]
	internal_state[1] = CHASE(roger, time); // assigns values to recommended_setpoints[0]
	// for N=2
	int state = internal_state[1]*3 + internal_state[0];
	int return_state;
	double home_1[2][2] = {{2.0, -1.5}, {-2.0, 1.5}};
	// double home_2[2][2] = {{3.1, -3.0}, {-3.1, 3.0}};
	// double home_2[2][2] = {{1.57, 0.0}, {-1.57, 0.0}};
	double home_2[2][2] = {{3.0, -2.7}, {-3.0, 2.7}};
	switch (state) {
						// CHASE 					TOUCH
		case 0: // NO_REFERENCE - NO_REFERENCE
		case 1: // NO_REFERENCE - TRANSIENT
		case 2: // NO_REFERENCE - CONVERGED
			/*** Ball is NOT SEEN. No reference to Chase => Body stop moving. Arms to "Home" position. ***/
			roger->base_setpoint[X] = roger->base_position[X];
			roger->base_setpoint[Y] = roger->base_position[Y];
			roger->arm_setpoint[LEFT][0] = home_2[0][0];
			roger->arm_setpoint[LEFT][1] = home_2[0][1];
			roger->arm_setpoint[RIGHT][0] = home_2[1][0];
			roger->arm_setpoint[RIGHT][1] = home_2[1][1];
			printf("%.4f, Chase:%d  Touch:%d Case 0 1 2\n", time, internal_state[1], internal_state[0]);
			return_state = NO_REFERENCE;
		break;
	
		case 3: // TRANSIENT - NO_REFERENCE
			/*** Ball is SEEN. Body is in Chasing. Ball NOT in REACH => Body moving. Arms to "Home" position. ***/
			roger->base_setpoint[X] = recommended_setpoints[P7_CHASE].base[X];
			roger->base_setpoint[Y] = recommended_setpoints[P7_CHASE].base[Y];
			roger->arm_setpoint[LEFT][0] = home_2[0][0];
			roger->arm_setpoint[LEFT][1] = home_2[0][1];
			roger->arm_setpoint[RIGHT][0] = home_2[1][0];
			roger->arm_setpoint[RIGHT][1] = home_2[1][1];
			printf("%.4f, Chase:%d  Touch:%d Case 3\n", time, internal_state[1], internal_state[0]);
			return_state = TRANSIENT;
			break;			
		case 4: // TRANSIENT - TRANSIENT
		case 5: // TRANSIENT - CONVERGED
			/*** Ball is SEEN. Body is in Chasing. Ball in REACH => Body moving. Arms to reach the ball. ***/
			roger->base_setpoint[X] = recommended_setpoints[P7_CHASE].base[X];
			roger->base_setpoint[Y] = recommended_setpoints[P7_CHASE].base[Y];
			roger->arm_setpoint[LEFT][0] = recommended_setpoints[P7_TOUCH].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = recommended_setpoints[P7_TOUCH].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = recommended_setpoints[P7_TOUCH].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = recommended_setpoints[P7_TOUCH].arm[RIGHT][1];
			printf("%.4f, Chase:%d  Touch:%d Case 4 5\n", time, internal_state[1], internal_state[0]);
			return_state = TRANSIENT;
		break;
	
		case 6: // CONVERGED - NO_REFERENCE
			/*** Ball is SEEN. Chasing completed. Ball NOT in REACH => Body stop moving. Arms to "Home" position. ***/
			/*** This will never happens ***/
			roger->base_setpoint[X] = roger->base_position[X];
			roger->base_setpoint[Y] = roger->base_position[Y];
			roger->arm_setpoint[LEFT][0] = home_2[0][0];
			roger->arm_setpoint[LEFT][1] = home_2[0][1];
			roger->arm_setpoint[RIGHT][0] = home_2[1][0];
			roger->arm_setpoint[RIGHT][1] = home_2[1][1];
			printf("%.4f, Chase:%d  Touch:%d Case 6\n", time, internal_state[1], internal_state[0]);
			return_state = TRANSIENT;
			break;
		case 7: // CONVERGED - TRANSIENT
			/*** Ball is SEEN. Chasing completed. Ball in REACH => Body moving. Arms to reach the ball. ***/
			roger->base_setpoint[X] = recommended_setpoints[P7_CHASE].base[X];
			roger->base_setpoint[Y] = recommended_setpoints[P7_CHASE].base[Y];
			roger->arm_setpoint[LEFT][0] = recommended_setpoints[P7_TOUCH].arm[LEFT][0];
			roger->arm_setpoint[LEFT][1] = recommended_setpoints[P7_TOUCH].arm[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = recommended_setpoints[P7_TOUCH].arm[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = recommended_setpoints[P7_TOUCH].arm[RIGHT][1];
			printf("%.4f, Chase:%d  Touch:%d Case 7\n", time, internal_state[1], internal_state[0]);
			return_state = TRANSIENT;
			break;
		case 8: // CONVERGED - CONVERGED
			/*** Ball is SEEN. Chasing completed. Ball in REACH. Ball is touched on both hands. => Body stop moving. Arms stop moving. ***/
			roger->base_setpoint[X] = roger->base_position[X];
			roger->base_setpoint[Y] = roger->base_position[Y];
			roger->arm_setpoint[LEFT][0] = roger->arm_theta[LEFT][0];
			roger->arm_setpoint[LEFT][1] = roger->arm_theta[LEFT][1];
			roger->arm_setpoint[RIGHT][0] = roger->arm_theta[RIGHT][0];
			roger->arm_setpoint[RIGHT][1] = roger->arm_theta[RIGHT][1];
			printf("%.4f, Chase:%d  Touch:%d Case 8\n", time, internal_state[1], internal_state[0]);
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
