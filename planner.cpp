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

	std::uniform_real_distribution<float> uni_distribution(0.0,1.0);

	Node Start, Goal;

	Start.x = 1;
	Start.y = 1;
	Start.vx = 0;
	Start.vy = 0;

	// Goal.x = 25;
	// Goal.y = 10;
	// Goal.vx = 0;
	// Goal.vy = 0;

	// Goal.x = 10;
	// Goal.y = 25;
	// Goal.vx = 0;
	// Goal.vy = 0;

	Goal.x = 20;
	Goal.y = 31;
	Goal.vx = 0;
	Goal.vy = 0;

 //    Goal.x = 42;
	// Goal.y = 39;
	// Goal.vx = 0;
	// Goal.vy = 0;

	list<Node*> Path;

	Tree RRT_Star(&Start, &Goal, map, x_size, y_size);
	Node* Goal_Node = RRT_Star.get_Goal();

	while (Goal_Node->parent == NULL){
		RRT_Star.expand_tree(uni_distribution);
	}

	//RRT_Star.print_tree();

	Node* Parent_Node = Goal_Node->parent;
	Path.push_front(Goal_Node);
	while(Parent_Node != NULL){
		Path.push_front(Parent_Node);
		Parent_Node = Parent_Node->parent;
	}

	RRT_Star.print_node(RRT_Star.get_Goal());

	for(int k=0; k<10; k++){
		for(int i=0;i<100;i++){
			RRT_Star.expand_tree(uni_distribution);
		}
		mexPrintf("Tree Size is %d\n",RRT_Star.get_tree_size());
	}
	

	RRT_Star.print_node(RRT_Star.get_Goal());

	int path_size = Path.size();

	mexPrintf("Tree Size is %d\n",RRT_Star.get_tree_size());

	mexPrintf("Path Size is %d\n",path_size);

	*plan = (float**) malloc(path_size*sizeof(float*));

	int p = 0;
	for (list<Node*>::iterator it = Path.begin(); it != Path.end(); it++){
		(*plan)[p] = (float*) malloc(3*sizeof(float)); 
		(*plan)[p][0] = (*it)->x;
		(*plan)[p][1] = (*it)->y;
		(*plan)[p][2] = (*it)->vx;
		(*plan)[p][3] = (*it)->vy;
		(*plan)[p][4] = (*it)->optimal_time;
		p++;
	}    

	*planlength = path_size;

	return;
}

void mexFunction(int nlhs, mxArray *plhs[],
	int nrhs, const mxArray*prhs[] )
{
    /* Check for proper number of arguments */
	if (nrhs != 3) {
		mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
			"Three input arguments required.");
	} else if (nlhs != 2) {
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
		PLAN_OUT = mxCreateNumericMatrix( (mwSize)planlength, 5, mxDOUBLE_CLASS, mxREAL); 
		double* plan_out = mxGetPr(PLAN_OUT);        
        //copy the values
		int i;
		for(i = 0; i < planlength; i++)
		{
			plan_out[0*planlength + i] = plan[i][0];
			plan_out[1*planlength + i] = plan[i][1];
			plan_out[2*planlength + i] = plan[i][2];
			plan_out[3*planlength + i] = plan[i][3];
			plan_out[4*planlength + i] = plan[i][4];
		}
	}
	else
	{
		PLAN_OUT = mxCreateNumericMatrix( (mwSize)1, 5, mxDOUBLE_CLASS, mxREAL); 
		double* plan_out = mxGetPr(PLAN_OUT);
        //copy the values     
		plan_out[0] = robotposeX;
		plan_out[1] = robotposeY;
		plan_out[2] = robotposeTheta;
		plan_out[3] = robotposeTheta;
		plan_out[4] = robotposeTheta;

	}

	PLANLENGTH_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL); 
	int* planlength_out = (int *)mxGetPr(PLANLENGTH_OUT);
	*planlength_out = planlength;
	return;
}