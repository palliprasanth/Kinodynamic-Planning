#include <math.h>
#include "mex.h"
#include <iostream>
#include <list>
#include "plannerheader2.hpp"
#include "constants.hpp"

float get_euclidian_distance(Node* node1, Node* node2){
	return pow(pow(node1->x - node2->x,2.0) + pow(node1->y - node2->y,2.0),0.5);
}

float deg2rad(float input){
	return (input*PI)/180.0;
}

float rad2deg(float input){
	return (input*180.0)/PI;
}

void wrap_to_pi(float* input){
	while(*input < -PI || *input >= PI){
		if (*input < -PI)
			*input += 2*PI;
		else
			*input -= 2*PI;
	}
}