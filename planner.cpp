/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include "plannerheader.hpp"
#include "constants.hpp"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

/*access to the map is shifted to account for 0-based indexing in the map, whereas
 * 1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

static void planner(double*	map, int x_size, int y_size, float robotposeX, float robotposeY,
	float robotposeTheta, float goalposeX, float goalposeY, float*** plan, int* planlength)
{

	return;
}

void mexFunction(int nlhs, mxArray *plhs[],
	int nrhs, const mxArray*prhs[] )
{
    /* Check for proper number of arguments */
	if (nrhs != 3) {
		mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
			"Three input arguments required.");
	} else if (nlhs != 1) {
		mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
			"One output argument required.");
	}

    /* get the dimensions of the map and the map matrix itself*/
	int x_size = mxGetM(MAP_IN);
	int y_size = mxGetN(MAP_IN);
	double* map = mxGetPr(MAP_IN);

    /* get the dimensions of the robotpose and the robotpose itself*/
	int robotpose_M = mxGetM(ROBOT_IN);
	int robotpose_N = mxGetN(ROBOT_IN);
	if(robotpose_M != 1 || robotpose_N != 3){
		mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
			"robotpose vector should be 1 by 3.");
	}
	double* robotposeV = mxGetPr(ROBOT_IN);
	float robotposeX = (float)robotposeV[0];
	float robotposeY = (float)robotposeV[1];
	float robotposeTheta = (float)robotposeV[2];

    /* get the dimensions of the goalpose and the goalpose itself*/
	int goalpose_M = mxGetM(GOAL_IN);
	int goalpose_N = mxGetN(GOAL_IN);
	if(goalpose_M != 1 || goalpose_N != 3){
		mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
			"goalpose vector should be 1 by 3.");
	}
	double* goalposeV = mxGetPr(GOAL_IN);
	float  goalposeX = (float)goalposeV[0];
	float  goalposeY = (float)goalposeV[1];

    /* Do the actual planning in a subroutine */
	float** plan = NULL;
	int planlength = 0;

	planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, &plan, &planlength);
	mexPrintf("planner returned plan of length=%d\n", planlength);

	/* Create return values */
	if(planlength > 0)
	{
		PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, 3, mxDOUBLE_CLASS, mxREAL); 
		double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
		int i;
		for(i = 0; i < planlength; i++)
		{
			plan_out[0*planlength + i] = plan[i][0];
			plan_out[1*planlength + i] = plan[i][1];
			plan_out[2*planlength + i] = plan[i][2];

		}
	}
	else
	{
		PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, 3, mxDOUBLE_CLASS, mxREAL); 
		double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values     
		plan_out[0] = robotposeX;
		plan_out[1] = robotposeY;
		plan_out[2] = robotposeTheta;

	}
	PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
	int* planlength_out = (int *)mxGetPr(PLANLENGTH_OUT);
	*planlength_out = planlength;
	return;
}