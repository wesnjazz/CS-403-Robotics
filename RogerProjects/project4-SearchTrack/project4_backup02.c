/*************************************************************************/
/* File:        project4.c                                               */
/* Description: User project #4 - empty project directory for project    */
/*              developement                                             */
/* Date:        01-2015                                                  */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include "Xkw/Xkw.h"
#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"

int converged = TRUE;
double recommended_setpoints[2][3];
double heading_prev = 0.0;
double heading = 0.0;
double time_prev = 0.0;
int ticktock = 0;

FILE *fileP4 = NULL;
int fileP4created = FALSE;

// double left_gaze = 0.0;
// double right_gaze = 0.0;
double left_gaze = M_PI/2.0;
double right_gaze = M_PI/2.0;

double zeroTo2PI(radian)
double radian;
{
	if (radian < 0)
	{
		radian += 2.0 * M_PI;
	}
	return radian;
}

double lessThanPI(radian)
double radian;
{
	if (radian > 0.0 && radian > M_PI) {
		radian -= 2.0 * M_PI;
	} else if (radian < 0.0 && radian < -M_PI) {
		radian += 2.0 * M_PI;
	}
	return radian;
}

int SEARCH(roger, time)
Robot* roger;
double time;
{
	// get a sample from sample_gaze_direction()
	// get a new sample only when it is converged, or at the first time
	// printf("%f\n", roger->ext_force[LEFT][0]);

			// printf("heading:%.4f  base_pos:%.4f  left_gaze: %.4f\n", zeroTo2PI(heading), zeroTo2PI(roger->base_position[THETA]), left_gaze);
	ticktock++;
	if (converged == TRUE) {
		heading_prev = heading;
		if (sample_gaze_direction(&heading) == TRUE) {

			// double a = heading_prev + 10.0;
			// double b = heading + 10.0;
			// if (a < b) {
			// 	double temp = b;
			// 	b = a;
			// 	a = temp;
			// }
			// while ((a-b) <= M_PI / 2.0 ) {
			// 	printf("Too small difference  head_prev:%.4f head:%.4f  diff:%.4f\n", heading_prev, heading, a-b);
			// 	// printf("heading:%f heading_prev:%f  heading - heading_prev: %f\n", heading, heading_prev,a-b);
			// 	heading_prev = heading;
			// 	sample_gaze_direction(&heading);
			// 	a = heading_prev + 10.0;
			// 	b = heading + 10.0;
			// 	if (a < b) {
			// 		double temp = b;
			// 		b = a;
			// 		a = temp;
			// 	}
			// }
			// 	printf("New better difference head_prev:%.4f head:%.4f  diff:%.4f\n", heading_prev, heading, a-b);



			// when heading is over M_PI, e.g., 3.1907
			while (heading > M_PI) {
				// printf("\t\t\tBigger than 3.141592\n");
				sample_gaze_direction(&heading);
			}

			// printf("New heading: %.4f\n", heading);
			converged = FALSE;

			time_prev = time;
			// printf("time:%f  ticktock:%d\n", time, ticktock);

			// left_gaze = right_gaze = zeroTo2PI(heading) - zeroTo2PI(roger->base_position[THETA]);
			// recommended_setpoints[0][LEFT] = lessThanPI(left_gaze);
			// recommended_setpoints[0][RIGHT] = lessThanPI(right_gaze);
			recommended_setpoints[0][THETA] = heading;

			return TRANSIENT;
		} else {
			converged = FALSE;
			// if sampling fails, then return 0: NO_REFERENCE
			return NO_REFERENCE;
		}
	}

	// double cover_min = roger->base_position[THETA] - 0.2;
	// double cover_max = roger->base_position[THETA] + 0.2;
	double cover_min = roger->base_position[THETA] - (M_PI/9.0);
	double cover_max = roger->base_position[THETA] + (M_PI/9.0);

	if (roger->base_setpoint[THETA] >= cover_min && roger->base_setpoint[THETA] <= cover_max) {
		// printf("CONVERGED! In Range, converged at %.4f\n", heading);
		converged = TRUE;
		return CONVERGED;
	}

	/** adjust gaze as eyes get closer to the heading **/
	left_gaze = right_gaze = zeroTo2PI(heading) - zeroTo2PI(roger->base_position[THETA]);
	recommended_setpoints[0][LEFT] = left_gaze;
	recommended_setpoints[0][RIGHT] = right_gaze;



		// printf("time:%.4f ticktock:%d\n", time, ticktock);
	// if (ticktock % 100 == 0) {
	// 		left_gaze = right_gaze = -left_gaze;
	// 		recommended_setpoints[0][LEFT] = left_gaze;
	// 		recommended_setpoints[0][RIGHT] = right_gaze;
	// 		ticktock = 0;
	// }

	// printf("time_prev:%f  time:%f  diff:%f\n", time_prev, time, time - time_prev);

	if (time - time_prev > 0.1) {
		printf("TOO LONG\n");
		converged = TRUE;
	}

	// printf("TRANS! base_pos:%.4f  head:%.4f head_prev:%.4f cov:min:%.4f max:%.4f\n", 
		// roger->base_position[THETA], heading, heading_prev, cover_min, cover_max);
	// roger->base_setpoint[THETA] = heading;

}

int TRACK(roger, time)
Robot* roger;
double time;
{
	double ul = -1.0;
	double ur = -1.0;
	// recommended_setpoints[1][THETA] = heading;

	if ( average_red_pixel(roger, &ul, &ur) == TRUE ) {
		// printf("Ball SEEN!\n");
		double eye_theta_error_l = atan2((ul - 63.5), FOCAL_LENGTH);
		double eye_theta_error_r = atan2((ur - 63.5), FOCAL_LENGTH);
		recommended_setpoints[1][LEFT] = roger->eye_theta[LEFT] + eye_theta_error_l;
		recommended_setpoints[1][RIGHT] = roger->eye_theta[RIGHT] + eye_theta_error_r;
		converged = TRUE;

		// if not foveated
		double eye_diff = fabs(roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]);
		// printf("eye_diff: %.4f\n", eye_diff);
		double tolerance = M_PI / 2.0;
		if (eye_diff > tolerance) {
			// base_position[THETA] + /2.0;
			recommended_setpoints[1][THETA] = roger->base_position[THETA] + (roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]) / 2.0;
			// printf("TRACK(): TRANSIENT\n");
			return TRANSIENT;
		} else {
			// printf("TRACK(): CONVERGED\n");
			return CONVERGED;
		}
	} else {
		// printf("TRACK(): NO_REFERENCE\n");
		return NO_REFERENCE;
	}
}

int SEARCHTRACK(roger, time)
Robot* roger;
double time;
{

	/***********************************************************************************************/
	/* internal_state=[ 0:SEARCH 1:TRACK ] */
	/***********************************************************************************************/
	int internal_state[2];
	internal_state[0] = SEARCH(roger, time); // assigns values to recommended_setpoints[0]
	internal_state[1] = TRACK(roger, time); // assigns values to recommended_setpoints[1]
	// for N=2
	int state = internal_state[1]*3 + internal_state[0];
	int return_state;
	switch (state) {
		// TRACK SEARCH
		case 0: // NO_REFERENCE - NO_REFERENCE
		// \\ choose action[i] 0 ,= i ,= NACTIONS
		// \\submit_setpoints(recommended_setpoints[i]);
		case 1: // NO_REFERENCE - TRANSIENT
		case 2: // NO_REFERENCE - CONVERGED
			roger->base_setpoint[THETA] = recommended_setpoints[0][THETA];
			roger->eyes_setpoint[LEFT] = recommended_setpoints[0][LEFT];
			roger->eyes_setpoint[RIGHT] = recommended_setpoints[0][RIGHT];
			// printf("Search:%d  Track:%d Case 1 2 3\n", internal_state[0], internal_state[1]);
			return_state = TRANSIENT;
		break;
	
		case 3: // TRANSIENT - NO_REFERENCE
		case 4: // TRANSIENT - TRANSIENT
		case 5: // TRANSIENT - CONVERGED
			roger->base_setpoint[THETA] = recommended_setpoints[1][THETA];
			roger->eyes_setpoint[LEFT] = recommended_setpoints[1][LEFT];
			roger->eyes_setpoint[RIGHT] = recommended_setpoints[1][RIGHT];
			// printf("Search:%d  Track:%d Case 3 4 5\n", internal_state[0], internal_state[1]);
			return_state = TRANSIENT;
		break;
	
		case 6: // CONVERGED - NO_REFERENCE
		case 7: // CONVERGED - TRANSIENT
		case 8: // CONVERGED - CONVERGED
			roger->base_setpoint[THETA] = recommended_setpoints[1][THETA];
			roger->eyes_setpoint[LEFT] = recommended_setpoints[1][LEFT];
			roger->eyes_setpoint[RIGHT] = recommended_setpoints[1][RIGHT];
			// printf("Search:%d  Track:%d Case 6 7 8\n", internal_state[0], internal_state[1]);
			return_state = CONVERGED;
		break;
		default:
		break;
	}
	return return_state;
}

void project4_control(roger, time)
Robot* roger;
double time;
{
	SEARCHTRACK(roger, time);
	// printf("heading:%.4f eye_heading-left:%.4f right%.4f\n", roger->eyes_setpoint[LEFT], roger->eyes_setpoint[RIGHT]);
	// printf("eye: (%.4f, %.4f)\n", roger->eyes_setpoint[LEFT], roger->eyes_setpoint[RIGHT]);

	double base_err = 0.0;
	base_err = roger->base_setpoint[THETA] - roger->base_position[THETA];
	// printf("base_set:%.4f base_pos:%.4f base_err:%.4f heading:%.4f\n",

	if (fileP4created) {
		fprintf(fileP4, "%f,%f\n", time, base_err);
	}


}

/************************************************************************/
void project4_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
// void project4_enter_params() 
void project4_enter_params(roger)
Robot* roger; 
{
	// success = sample_gaze_direction(&heading);
	// converged = FALSE;
	// sample_gaze_direction(&heading);
	// printf("heading:%f\n", heading);
  // printf("Project 4 enter_params called. \n");
  // double random = 0.0;
  // printf("random:"); fflush(stdout);
  // scanf("%lf", &random);

  /*** Project 4 FILE ***/
  if (!fileP4created) {
	  char file_prefix[] = "project4-SearchTrack/data/P4";
	  char file_concat[80];
	  sprintf(file_concat, "%s.txt", file_prefix);
	  fileP4 = fopen(file_concat, "w+");
	  fileP4created = TRUE;
	  fprintf(fileP4, "time,base_err\n");
  }
}

//function called when the 'visualize' button on the gui is pressed
void project4_visualize(roger)
Robot* roger;
{ }
