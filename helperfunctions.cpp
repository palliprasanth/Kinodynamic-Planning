#include <math.h>
#include "mex.h"
#include <iostream>
#include <list>
#include "plannerheader.hpp"
#include "constants.hpp"

float get_euclidian_distance(Node* node1, Node* node2){
	return pow(pow(node1->x - node2->x,2.0) + pow(node1->y - node2->y,2.0),0.5);
}

