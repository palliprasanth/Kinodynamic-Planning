#include <math.h>
#include "mex.h"
#include <iostream>
#include <list>
#include <iterator>
#include "plannerheader.hpp"
#include "constants.hpp"

using namespace std;

Tree::Tree(){
	mexPrintf("Tree Created\n");
}

Tree::~Tree(){
	mexPrintf("Tree Destroyed\n");
}

void Tree::print_tree(){
	mexPrintf("Printing the Tree\n");
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		mexPrintf("node_id = %d\n",it->node_id);
		mexPrintf("parent_id = %d\n",it->parent->node_id);
		mexPrintf("Cost = %f\n",it->cost);
		mexPrintf("Number of children: %d\n",it->children.size());
		mexPrintf("x = %f\n",it->x);
		mexPrintf("y = %f\n",it->y);
		mexPrintf("theta = %f\n",it->theta);
		mexPrintf("\n");
	}
}