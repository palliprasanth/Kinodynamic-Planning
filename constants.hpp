#ifndef CONSTANTS_H   
#define CONSTANTS_H

#define PI 3.141592654

// Map resolution in meters
#define RES 0.1

#define VOL_HYPER 4.935 //Volume of unit hyperball in 4D space
#define GAMMA 100

#define MAX_RAD 10 // Distance threshold for controller generation in meters
#define MAX_EDGE_COST 10 //Max cost threshold for an edge
#define T_MAX 7 // Max time for bisection root finding
#define T_DIFF 0.05 // Time Difference for trajectory computation

// State Space Limits
#define X_MIN 0
#define X_MAX 50

#define Y_MIN 0
#define Y_MAX 50

#define VX_MIN -2
#define VX_MAX 2

#define VY_MIN -2
#define VY_MAX 2

#endif