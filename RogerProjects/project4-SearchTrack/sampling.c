/*************************************************************************/
/* File:        sampling.c                                               */
/* Description: Methods for sampling from distributions                  */
/* Date:        10-30-2012                                               */
/*************************************************************************/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "roger.h"
#include "simulate.h"
#include "control.h"
#include "modes.h"
#include "models.h"

double time();
int init_random_seed = FALSE; // int init_random_seed = TRUE;

#define MAX_SAMPLES 10 // number of sample repetition before giving up
int sample_count = 0;

// pr.bin_size, pr.area, pr.dist[NHEADINGS]
// 2pi / 64bins = 0.0981747704
// 1.0 / 64bins = 0.015625
// 1 bin = 0.0981747704 radians = 5.625 degrees
// P(1 bin) = 0.015625
Pr_dist Pr_red_prior = 
  {(2.0*M_PI/NHEADINGS), 1.0,{1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,1.0/NH,
			      1.0/NH}};

int sample_gaze_direction(heading)
// int sample_gaze_direction(heading, random)
	double *heading;
	// double random;
{
	int i;
	// double sum;
	double sum, random;

	if (init_random_seed == TRUE) {
		srand(time(NULL));
		init_random_seed = FALSE;
	}
	// uniformly sample from -PI to PI
	random = ((double)rand()/(double)RAND_MAX)*Pr_red_prior.area;
	// printf("\nrandom:%.4f  RAND_MAX:%.4f\n", random, RAND_MAX);
	i=sum=0;
	while (sum < random) {
		sum += Pr_red_prior.dist[i++];
	}
	// printf("NHEADINGS:%d  NH:%d\n", NHEADINGS, NH);
	// for (int i = 0; i<NHEADINGS; i=i+8) {
	// 	printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n", 
	// 		Pr_red_prior.dist[i], Pr_red_prior.dist[i+1], Pr_red_prior.dist[i+2], Pr_red_prior.dist[i+3], Pr_red_prior.dist[i+4], Pr_red_prior.dist[i+5], Pr_red_prior.dist[i+6], Pr_red_prior.dist[i+7]);
	// }
	// *heading = ((double)i)*Pr_red_prior.bin_size - M_PI;
	*heading = ((double)i+0.5)*Pr_red_prior.bin_size - M_PI;
	// printf("%.4f * %.4f - %.4f = %.4f\n", (double)i+0.5, Pr_red_prior.bin_size, M_PI, *heading);
	// printf("i=%d, sum=%.4f, heading=%.4f\n", i, sum, *heading);

	if (Pr_red_prior.area < 0.02) {
		sample_count = 0;
		return FALSE;
	}
	return TRUE;
}
