/*************************************************************************/
/* File:        project3.c                                               */
/* Description: User project #3 - empty project directory for project    */
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

FILE *fpv = NULL;
int file_created_v = 0;

int average_red_pixel(roger, ul, ur)
Robot* roger;
double *ul;
double *ur;
{
/** PESUDO CODE
	if (there is red detected in both images) {
		estimate image coordinates, ul and ur,
		of the center of the red segments on the
		left and right images and return(TRUE)
	}
	else return(FALSE);
**/

	// for each eyes
	for (int i=0; i<NEYES; i++) {

		// initialize left/right boundary
		int left_bound = -1, right_bound = -1;

		// search through all 128 pixles on both eyes
		for (int j=0; j<NPIXELS; j++) {

			// if red color is seen
			if (roger->image[i][j][0] == 255) {

				// if left boundary not set yet,
				if (left_bound < 0) {
					left_bound = j;
				}
				// keep track right boundary
				right_bound = j;
			} 
			/*** Stop searching early. Not working properly cause these must be an interrupt image between red segments. ***/
			// else {
			// 	// stop searching if both left and right boundary found
			// 	if (left_bound >= 0 && right_bound >= 0) {
			// 		break;
			// 	}
			//
		}

		// if there was left boundary
		if (left_bound >= 0) {
			// printf("left: %d  right: %d\n", left_bound, right_bound);
			if (i == 0) {
				*ul = (left_bound + right_bound) / 2.0;
			}
			else if (i == 1) {
				*ur = (left_bound + right_bound) / 2.0;
			}
		} 
		// printf("eye[%d]: left_bound:%d  right_bound:%d   ul:%f  ur:%f \n", i, left_bound, right_bound, *ul, *ur);
	}

	// red color is detected on both image plane, thus there are both ul and ur
	if (*ul >= 0.0 && *ur >= 0.0) {
		return TRUE;
	} else {
		return FALSE;
	}
}

void project3_control(roger, time)
Robot* roger;
double time;
{
	double ul = -1.0;
	double ur = -1.0;

	//	red is detected
	if ( average_red_pixel(roger, &ul, &ur) == TRUE ) {
	// new setpoint for LEFT EYE
	// if (ul >= 0.0) {
		double eye_theta_error_l = atan2((ul - 63.5), FOCAL_LENGTH);
		roger->eyes_setpoint[LEFT] = roger->eye_theta[LEFT] + eye_theta_error_l;
	// } 
	// else {
	// roger->eyes_setpoint[LEFT] = 0;
	// }

	// new setpoint RIGHT EYE
	// if (ur >= 0.0) {
		double eye_theta_error_r = atan2((ur - 63.5), FOCAL_LENGTH);
		roger->eyes_setpoint[RIGHT] = roger->eye_theta[RIGHT] + eye_theta_error_r;
	// } 
	// else {
	// roger->eyes_setpoint[RIGHT] = 0;
	// }
  printf("Time:%.3f EYE_Th[L,R]:(%2.3f,%2.3f) EYE_Set[L,R]:(%2.3f,%2.3f) T_error:(%f,%f)\n", 
		time, roger->eye_theta[LEFT], roger->eye_theta[RIGHT],
			    roger->eyes_setpoint[LEFT], roger->eyes_setpoint[RIGHT],
  				eye_theta_error_l, eye_theta_error_r);

		if (file_created_v) {
			fprintf(fpv, "%f,%f,%f\n", time, eye_theta_error_l, eye_theta_error_r);
		}

	}
	// else { // if red is not detected on both eyes
	// 	roger->eyes_setpoint[LEFT] = 0;
	// 	roger->eyes_setpoint[RIGHT] = 0;
	// }

}

/************************************************************************/
void project3_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project3_enter_params() 
{
  // printf("Project 4 enter_params called. \n");

  // char file_vis_prefix[] = "project3-Vision/data/vis";
  // char file_vis_concat[80];
  // sprintf(file_vis_concat, "%s.txt", file_vis_prefix);
  // fpv = fopen(file_vis_concat, "w+");
  // file_created_v = 1;
  // fprintf(fpv, "time,eye_theta_error_l,eye_theta_error_r\n");
}

//function called when the 'visualize' button on the gui is pressed
void project3_visualize(roger)
Robot* roger;
{ }
