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

#ifndef BALL_RNDP4
#define BALL_RNDP4	-0.0
#endif

typedef struct _ballposP4 {
	double x[NBINS];
	double y[NBINS];
	double time[NBINS];
	int head;
	int tail;
	int size;
	int cnt;
} BallPosP4;

Observation obs;
BallPosP4 bps;

int bps_createdP4 = FALSE;

int init_BallPosP4(BallPosP4* bps) {
	for (int i=0; i<NBINS; i++) {
		bps->x[i] = BALL_RNDP4;
		bps->y[i] = BALL_RNDP4;
		bps->time[i] = BALL_RNDP4;
	}
	bps->head = 0;
	bps->tail = 0;
	bps->size = NBINS;
	bps->cnt = 0;
}

int record_BallPosP4(Robot* roger, double time, Observation* obs, BallPosP4* bps) 
{
	int head = bps->head;
	int tail = bps->tail;
	int size = bps->size;
	int cnt = bps->cnt;

	double x = obs->pos[X];
	double y = obs->pos[Y];
	if (time > 0.001) { // avoid record BALL_RNDP4 at the beginning
		if (x > MAX_X || x < MIN_X) { // if x is out of bounds, record trash value
			x = BALL_RNDP4;
		}
		if (y > MAX_Y || y < MIN_Y) { // if y is out of bounds, record trash value
			y = BALL_RNDP4;
		}
	}

	if (cnt == 0) { // if it is recording for the first time
		bps->head = 0;
		bps->tail = 0;
		bps->cnt = 1;
		bps->time[0] = time;
		bps->x[0] = x;
		bps->y[0] = y;
		return TRUE;
	}

	// get difference between current x,y and previous x,y
	// if difference bigger than epsilon, it is a noise, so record trash value
	double prev_x = (tail > 0) ? (bps->x[tail-1]) : (bps->x[NBINS-1]);
	double prev_y = (tail > 0) ? (bps->y[tail-1]) : (bps->y[NBINS-1]);
	double diff_x = fabs(x - prev_x);
	double diff_y = fabs(y - prev_y);
	double epsilon = 5.5;
	if (prev_x < BALL_RNDP4 && diff_x > epsilon) {
		x = BALL_RNDP4;
	}
	if (prev_y < BALL_RNDP4 && diff_y > epsilon) {
		y = BALL_RNDP4;
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
	if (cnt < NBINS) {
		bps->cnt++;
	}
	// printf("%.4f- recording Ball Position: t:%.4f x:%.4f y:%.4f at head:%d  tail:%d  cnt:%d  size:%d\n",
	// 				time, bps->time[tail], bps->x[tail], bps->y[tail], bps->head, bps->tail, bps->cnt, bps->size);
	return TRUE;
}

int velocity_BallP4(Robot* roger, double time, BallPosP4* bps, double* vel_x, double* vel_y)
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
			tail = NBINS-1;
		}
		x_last = bps->x[tail];
	}
	double x_prev = bps->x[head];
	while (x_prev > epsilon) {
		head++;
		if (head >= NBINS) {
			head = 0;
		}
		x_prev = bps->x[head];
	}
	double y_last = bps->y[tail];
	while (y_last > epsilon) {
		tail--;
		if (tail < 0) {
			tail = NBINS-1;
		}
		y_last = bps->y[tail];
	}
	double y_prev = bps->y[head];
	while (y_prev > epsilon) {
		head++;
		if (head >= NBINS) {
			head = 0;
		}
		y_prev = bps->y[head];
	}

	*vel_x = (bps->x[tail] - bps->x[head]) / (NBINS+0.00000000001);
	*vel_y = (bps->y[tail] - bps->y[head]) / (NBINS+0.00000000001);
}

int search_converged = TRUE;

double recommended_setpoints[2][3];
// SetPoint recommended_setpoints[3];

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

int SEARCH_P4(roger, time)
Robot* roger;
double time;
{
	// get a sample from sample_gaze_direction()
	// get a new sample only when it is converged, or at the first time
	ticktock++;
	if (search_converged == TRUE) {
		heading_prev = heading;
		if (sample_gaze_direction(&heading) == TRUE) {
			// when heading is over M_PI, e.g., 3.1907
			while (heading > M_PI) {
				sample_gaze_direction(&heading);
			}
			// printf("New heading: %.4f\n", heading);
			search_converged = FALSE;

			time_prev = time;
			// printf("time:%f  ticktock:%d\n", time, ticktock);

			// left_gaze = right_gaze = zeroTo2PI(heading) - zeroTo2PI(roger->base_position[THETA]);
			// recommended_setpoints[0][LEFT] = lessThanPI(left_gaze);
			// recommended_setpoints[0][RIGHT] = lessThanPI(right_gaze);
			recommended_setpoints[0][THETA] = heading;

			return TRANSIENT;
		} else {
			search_converged = FALSE;
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
		search_converged = TRUE;
		return CONVERGED;
	}

	/** adjust gaze as eyes get closer to the heading **/
	left_gaze = right_gaze = zeroTo2PI(heading) - zeroTo2PI(roger->base_position[THETA]);
	recommended_setpoints[0][LEFT] = left_gaze;
	recommended_setpoints[0][RIGHT] = right_gaze;

	// If it takes too long to look at the sample
	if (time - time_prev > 0.2) {
		printf("TOO LONG\n");
		search_converged = TRUE;
	}
}

int TRACK_P4(roger, time)
Robot* roger;
double time;
{
	double ul = -1.0;
	double ur = -1.0;

	if (!bps_createdP4) {
		init_BallPosP4(&bps);
		bps_createdP4 = TRUE;
	}
	
	if ( average_red_pixel(roger, &ul, &ur) == TRUE ) {
		// printf("Ball SEEN!\n");
		// stereo_observationP4(roger, &obs);
		// record_BallPosP4(roger, time, &obs, &bps);
		// double vel_x;
		// double vel_y;
		// velocity_BallP4(roger, time, &bps, &vel_x, &vel_y);
		// double t_x = (obs.pos[X] - roger->base_position[X] + R_BASE) / (vel_x + 0.00000000001);
		// double t_y = (obs.pos[Y] - roger->base_position[Y] + R_BASE) / (vel_y + 0.00000000001);
		// double arrive_x = obs.pos[X] + vel_x * t_x;
		// double arrive_y = obs.pos[Y] + vel_y * t_y;
		// printf("%3.3f-  \tvel_x:%3.3f \tvel_y:%3.3f \tt_x:%3.3f\n", time, vel_x, vel_y, t_x);

		// roger->base_setpoint[X] = arrive_x;
		// roger->base_setpoint[Y] = arrive_y;


		double eye_theta_error_l = atan2((ul - 63.5), FOCAL_LENGTH);
		double eye_theta_error_r = atan2((ur - 63.5), FOCAL_LENGTH);
		recommended_setpoints[1][LEFT] = roger->eye_theta[LEFT] + eye_theta_error_l;
		recommended_setpoints[1][RIGHT] = roger->eye_theta[RIGHT] + eye_theta_error_r;
		search_converged = TRUE;

		// if not foveated
		double eye_diff = fabs(roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]);
		// printf("eye_diff: %.4f\n", eye_diff);
		double epsilon = M_PI / 2.0;
		if (eye_diff > epsilon) {
			recommended_setpoints[1][THETA] = roger->base_position[THETA] + (roger->eye_theta[LEFT] + roger->eye_theta[RIGHT]) / 2.0;
			return TRANSIENT;
		} else {
			return CONVERGED;
		}
	} else {
		return NO_REFERENCE;
	}
}

int SEARCHTRACK_P4(roger, time)
Robot* roger;
double time;
{

	/***********************************************************************************************/
	/* internal_state=[ 0:SEARCH 1:TRACK ] */
	/***********************************************************************************************/
	int internal_state[2];
	internal_state[0] = SEARCH_P4(roger, time); // assigns values to recommended_setpoints[0]
	internal_state[1] = TRACK_P4(roger, time); // assigns values to recommended_setpoints[1]
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
	SEARCHTRACK_P4(roger, time);
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
