/*************************************************************************/
/* File:        project5.c                                               */
/* Description: User project #5 - empty project directory for project    */
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

double scale = 0.001;

void stereoJJT(roger, gammaR, gammaL, cov_b)
Robot* roger;
// double* ur;
// double* ul;
double gammaR;
double gammaL;
double cov_b[2][2];
{
	// cov_b[0][0] = det*cos(gammaR)*cos(gammaR);
	// cov_b[0][1] = det*(-(cos(gammaL)*cos(gammaL)));
	// cov_b[1][0] = det*sin(gammaR)*cos(gammaR);
	// cov_b[1][1] = det*(-(sin(gammaL)*cos(gammaL)));

	double J[2][2];
	double det = 2.0*BASELINE / (sin(gammaR - gammaL)*sin(gammaR - gammaL));
	J[0][0] = det*cos(gammaR)*cos(gammaR);
	J[0][1] = det*(-(cos(gammaL)*cos(gammaL)));
	J[1][0] = det*sin(gammaR)*cos(gammaR);
	J[1][1] = det*(-(sin(gammaL)*cos(gammaL)));

	double JT[2][2];
	matrix_transpose(2, 2, J, JT);
	matrix_mult(2, 2, J, 2, JT, cov_b);
}

void stereo_observation(roger, obs)
Robot* roger;
Observation* obs;
// double time;
// double scale;
{
	// get current eye angle = verge angle
	double theta_L = roger->eye_theta[LEFT];
	double theta_R = roger->eye_theta[RIGHT];

	// get angular offset of p from the image center
	double ul;
	double ur;
	average_red_pixel(roger, &ul, &ur);
	double phiL = atan2(ul-63.5, FOCAL_LENGTH);
	double phiR = atan2(ur-63.5, FOCAL_LENGTH);

	// sum of two angles = theta + phi
	double gammaL = theta_L + phiL;
	double gammaR = theta_R + phiR;

	// double x = BASELINE * ( (cos(gammaR) * cos(gammaL)) / ((sin(gammaR - gammaL))) );
	// double y = (BASELINE / 2) + (BASELINE * ( (cos(gammaR) * sin(gammaL)) / ((sin(gammaR - gammaL))) ));

	// double lambdaR = (BASELINE * cos(gammaL)) / (sin(gammaR - gammaL));
	// double lambdaL = (BASELINE * cos(gammaR)) / (sin(gammaR - gammaL));
	// catch case where eyes are parallel and same pixels on both eyes (infinite distance)

	double lambdaR = 0.0;
	double lambdaL = 0.0;
	if ((gammaR - gammaL) == 0.0)	{
		lambdaR = 20.0;
		lambdaL = 20.0;  // that's 20 meters! (almost infinity)
	} 
	else {
		lambdaR = 2.0*BASELINE*cos(gammaL) / sin(gammaR - gammaL);
		lambdaL = 2.0*BASELINE*cos(gammaR) / sin(gammaR - gammaL);
	}
	// double x = lambdaL*cos(gammaL);
	// double y = BASELINE + lambdaL*sin(gammaL);


	// printf("t:%3.3f ul:%3.1f ur:%3.1f tL:%.3f tR:%.3f pL:%6.3f pR:%6.3f gL:%6.3f gR:%6.3f \n",
	// 	roger->simtime, ul, ur, theta_L, theta_R, phiL, phiR, gammaL, gammaR);

  // to draw the stereo result, it has to be in world coordinates
  // first: triangulate the (x,y) coordinate in the base frame (using lambdaL)
  // and make it a homogeneous position vector
	
	double pos_b[4];
	// if (gammaL >= 0.0) {
		pos_b[X] = lambdaL * cos(gammaL);
		pos_b[Y] = BASELINE + (lambdaL * sin(gammaL));
		pos_b[2] = 0.0;
		pos_b[3] = 1.0;
	// } 
	// else {
	// 	pos_b[X] = lambdaR * cos(gammaR);
	// 	pos_b[Y] = -BASELINE + (lambdaR * sin(gammaR));
	// 	pos_b[2] = 0.0;
	// 	pos_b[3] = 1.0;
	// }
	

	// printf("t:%3.3f x:%.3f y:%.3f  gammaR:%.3f  gammaL:%.3f\n", roger->simtime, pos_b[X], pos_b[Y], gammaR, gammaL);



	// printf("pos_b[4]: [%f, %f, %f, %f]\n", pos_b[X], pos_b[Y], pos_b[2], pos_b[3]);

	// convert into world frame
	double pos_w[4];
	double wTb[4][4];
  construct_wTb(roger->base_position, wTb);
  matrix_mult(4, 4, wTb, 1, pos_b, pos_w);
	// printf("pos_w[4][4]: [%f %f]\n", pos_w[0], pos_w[1]);

	// printf("pos_w[4][4]: [%f %f]\n", pos_w[0], pos_w[1], pos_w[2], pos_w[3]);
	// printf("wTb[4][4]: [%f %f %f %f, %f %f %f %f, %f %f %f %f, %f %f %f %f\n", 
	// 	wTb[0][0], wTb[0][1], wTb[0][2], wTb[0][3],
	// 	wTb[1][0], wTb[1][1], wTb[1][2], wTb[1][3],
	// 	wTb[2][0], wTb[2][1], wTb[2][2], wTb[2][3]);

	// printf("obs->pos[X]:%f\n", obs->pos[X]);
	// printf("pos_w[3][X]:%f\n", pos_w[3][X]);
	obs->pos[X] = pos_w[X];
  obs->pos[Y] = pos_w[Y];

 	// the covariance matrix of the Observation (JJT) must be rotated as well:
 	double wRb[2][2];
	wRb[0][0] = wTb[0][0]; wRb[0][1] = wTb[0][1];
	wRb[1][0] = wTb[1][0]; wRb[1][1] = wTb[1][1];
	// printf("wRb[2][2]: [[%f %f], [%f %f]]\n", wRb[0][0], wRb[0][1], wRb[1][0], wRb[1][1]);

	// compute observation cov (JJT)
	double cov_b[2][2];
	// stereoJJT(roger, ur, ul, cov_b);
	stereoJJT(roger, gammaR, gammaL, cov_b);
	// cov_b[0][0] = 0.3;
	// cov_b[0][1] = 0.2;
	// cov_b[1][0] = 0.5;
	// cov_b[1][1] = 0.9;

	// and rotate it into world coordinates ... [cov]_w = wRb [cov]_b wRb^T
	double bRw[2][2];
	double mat2X2[2][2];
	matrix_transpose(2, 2, wRb, bRw);
	matrix_mult(2, 2, wRb, 2, cov_b, mat2X2);
	matrix_mult(2, 2, mat2X2, 2, bRw, obs->cov);

	// and scale it experimentally, you can use enter_params( ) for scale
	obs->cov[0][0] *= scale;
	obs->cov[0][1] *= scale;
	obs->cov[1][0] *= scale;
	obs->cov[1][1] *= scale;

	// obs.cov[0][0] = 3;
	// obs.cov[0][1] = 4;
	// obs.cov[1][0] = 5;
	// obs.cov[1][1] = 6;
	obs->time = roger->simtime;
}


int obs_number = 0;
void project5_control(roger, time)
Robot* roger;
double time;
{ 
	// printf("Project 5()\n");
	// printf("stereo_observation() called from Project5_control()\n");
	// printf("%f %f\n%f %f %f %f\n%f\n", obs.pos[X], obs.pos[Y],
					// obs.cov[0][0], obs.cov[0][1], obs.cov[1][0], obs.cov[1][1], time);
	// roger->drawing_request.obs_number = obs_number;
	roger->drawing_request.obs_number = obs_number;
}

/************************************************************************/
void project5_reset(roger)
Robot* roger;
{ }

// prompt for and read user customized input values
void project5_enter_params() 
{
  // printf("Project 5: Scale=%6.4f\n", scale);
  // printf("Base Rotate: enter 'scale'\n"); fflush(stdout);
  // scanf("%lf", &scale);
  obs_number += 1;
}

// Observation obs_list[10000];
// int obs_count = 0;

//function called when the 'visualize' button on the gui is pressed
void project5_visualize(roger)
Robot* roger;
{ 
	Observation obs;
	stereo_observation(roger, &obs);
	draw_observation(roger, obs);

	// if (roger->simtime > obs_count / 5.0 && obs_count <= 10000) {
	// 	stereo_observation(roger, &obs_list[obs_count]);
	// 	obs_count++;
	// }
	// for (int i=0; i<obs_count; i++) {
	// 	draw_observation(roger, obs_list[i]);
	// }
}
