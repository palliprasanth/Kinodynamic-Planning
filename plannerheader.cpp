#include <math.h>
#include "mex.h"
#include <iostream>
#include <list>
#include <iterator>
#include "plannerheader.hpp"
#include "constants.hpp"

using namespace std;

Tree::Tree(Node* Start, Node* Goal){
	
	Node temp;

	temp.node_id = Vertices.size()+1;
	temp.x = Start->x;
	temp.y = Start->y;
	temp.vx = Start->vx;
	temp.vy = Start->vy;
	temp.cost = 0;
	temp.parent = NULL;

	Vertices.push_back(temp);

	Start_Node = &Vertices.back();

	temp.node_id = Vertices.size()+1;
	temp.x = Goal->x;
	temp.y = Goal->y;
	temp.vx = Goal->vx;
	temp.vy = Goal->vy;
	temp.cost = 0;
	temp.parent = NULL;

	Vertices.push_back(temp);

	Goal_Node = &Vertices.back();

	reached = false;
}

Tree::~Tree(){
	// mexPrintf("Tree Destroyed\n");
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
		//mexPrintf("theta = %f\n",it->theta);
		mexPrintf("\n");
	}
}

SystemDI::SystemDI(){
	//mexPrintf("System Created\n");
	r = 0.25;
}



float SystemDI::cost_of_path(Node* Start, Node* Goal, float tau){
	float value = 0;

	float x1,x2,x3,x4,y1,y2,y3,y4;

	x1 = Start->x; 
	x2 = Start->y;
	x3 = Start->vx; 
	x4 = Start->vy;

	y1 = Goal->x; 
	y2 = Goal->y;
	y3 = Goal->vx; 
	y4 = Goal->vy;

	value = tau + ((4*r*(x3 - y3))/tau - (6*r*(x1 - y1 + tau*x3))/pow(tau,2))*(x3 - y3) + ((4*r*(x4 - y4))/tau - (6*r*(x2 - y2 + tau*x4))/pow(tau,2))*(x4 - y4) - ((6*r*(x3 - y3))/pow(tau,2) - (12*r*(x1 - y1 + tau*x3))/pow(tau,3))*(x1 - y1 + tau*x3) - ((6*r*(x4 - y4))/pow(tau,2) - (12*r*(x2 - y2 + tau*x4))/pow(tau,3))*(x2 - y2 + tau*x4);

	return value;
}

float SystemDI::diff_cost_of_path(float x1, float x2, float x3, float x4, float y1, float y2, float y3, float y4, float tau){
	float value = 0;

	value = (12*r*y3*(2*x1 - 2*y1 + tau*x3 + tau*y3))/pow(tau,3) - (4*r*pow((3*x2 - 3*y2 + tau*x4 + 2*tau*y4),2))/pow(tau,4) - (4*r*pow((3*x1 - 3*y1 + tau*x3 + 2*tau*y3),2))/pow(tau,4) + (12*r*y4*(2*x2 - 2*y2 + tau*x4 + tau*y4))/pow(tau,3) + 1;

	return value;
}

bool SystemDI::optimal_arrival_time(Node* Start, Node* Goal, float* opt_time){

	float x1,x2,x3,x4,y1,y2,y3,y4;

	x1 = Start->x; 
	x2 = Start->y;
	x3 = Start->vx; 
	x4 = Start->vy;

	y1 = Goal->x; 
	y2 = Goal->y;
	y3 = Goal->vx; 
	y4 = Goal->vy;

	float t_init = 0.001;
	float t_final = 100;
	float error = 0.001;

	float fun_init, fun_mid, fun_final;

	float t_mid;

	if(diff_cost_of_path(x1, x2, x3, x4, y1, y2, y3, y4, t_init)*diff_cost_of_path(x1, x2, x3, x4, y1, y2, y3, y4, t_final) > 0){
		return false;
	}

	else{
		while (fabs(t_final-t_init)>=error){
			t_mid = (t_init + t_final)/2;
			fun_init = diff_cost_of_path(x1, x2, x3, x4, y1, y2, y3, y4, t_init);
			fun_mid = diff_cost_of_path(x1, x2, x3, x4, y1, y2, y3, y4, t_mid);
			fun_final = diff_cost_of_path(x1, x2, x3, x4, y1, y2, y3, y4, t_final);

			if (fun_mid==0){
				(*opt_time) = t_mid;
				return true;
			}

			if (fun_init*fun_mid>0){
				t_init=t_mid; 
			}
			else if (fun_init*fun_mid<0){    
				t_final=t_mid;    
			}    
		}
	}

	(*opt_time) = t_init;
	return true;

}

void SystemDI::compute_trajectory(Node* Start, Node* Goal, float t_star){
	float x1,x2,x3,x4,y1,y2,y3,y4;

	x1 = Start->x; 
	x2 = Start->y;
	x3 = Start->vx; 
	x4 = Start->vy;

	y1 = Goal->x; 
	y2 = Goal->y;
	y3 = Goal->vx; 
	y4 = Goal->vy;

	float t_delta = 0.05;
	float t = t_delta;

	Point2D temp;

	Edge temp_edge;
	// Creating the edge
	temp_edge.parent = Start;
	temp_edge.child = Goal;
	temp_edge.edge_cost = cost_of_path(Start, Goal, t_star);

	temp.x = Start->x;
	temp.y = Start->y;
	temp_edge.trajectory.push_back(temp);

    // Add Collision Check here
    // If collision happens break it
	while(t < t_star){
		temp.x = y1 + y3*(t - t_star) - (((4*r*(x3 - y3))/t_star - (6*r*(x1 - y1 + t_star*x3))/pow(t_star,2))*pow(t - t_star,2))/(2*r) - (((6*r*(x3 - y3))/pow(t_star,2) - (12*r*(x1 - y1 + t_star*x3))/pow(t_star,3))*pow(t - t_star,3))/(6*r);
		temp.y = y2 + y4*(t - t_star) - (((4*r*(x4 - y4))/t_star - (6*r*(x2 - y2 + t_star*x4))/pow(t_star,2))*pow(t - t_star,2))/(2*r) - (((6*r*(x4 - y4))/pow(t_star,2) - (12*r*(x2 - y2 + t_star*x4))/pow(t_star,3))*pow(t - t_star,3))/(6*r);
		temp_edge.trajectory.push_back(temp);
		t += t_delta;
	}

	temp.x = Goal->x;
	temp.y = Goal->y;
	temp_edge.trajectory.push_back(temp);

	// Adding the edge as a child of Start
	Start->children.push_back(temp_edge);
	return;
}

SystemDI::~SystemDI(){
	// mexPrintf("System Destroyed\n");
}