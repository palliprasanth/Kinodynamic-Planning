#include <math.h>
#include "mex.h"
#include <iostream>
#include <list>
#include <iterator>
#include <random>
#include <chrono>
#include "plannerheader.hpp"
#include "constants.hpp"

using namespace std;

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

Tree::Tree(Node* Start, Node* Goal, double* input_map, int xsize, int ysize){

	grid_map = input_map;
	x_size = xsize;
	y_size = ysize;
	
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

	Goal_Node.node_id = -1;
	Goal_Node.x = Goal->x;
	Goal_Node.y = Goal->y;
	Goal_Node.vx = Goal->vx;
	Goal_Node.vy = Goal->vy;
	Goal_Node.cost = 0;
	Goal_Node.parent = NULL;

	reached = false;

	// System Properties Initializing
	r = 0.25;
}

Tree::~Tree(){
	// mexPrintf("Tree Destroyed\n");
}

Node* Tree::get_Start(){
	return Start_Node;
}

Node* Tree::get_Goal(){
	return &Goal_Node;
}

void Tree::generate_sample_Node(){
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::uniform_real_distribution<double> uni_distribution(0.0,1.0);

	bool valid_sample = false;

	while (!valid_sample){
		Sample_Node.x = X_MIN + uni_distribution(generator)*(X_MAX-X_MIN);
		Sample_Node.y = Y_MIN + uni_distribution(generator)*(Y_MAX-Y_MIN);
		Sample_Node.vx = VX_MIN + uni_distribution(generator)*(VX_MAX-VX_MIN);
		Sample_Node.vy = VY_MIN + uni_distribution(generator)*(VY_MAX-VY_MIN);

		valid_sample = is_valid_Node(&Sample_Node);
	}

	//mexPrintf("%f, %f, %f, %f\n",Sample_Node.x,Sample_Node.y,Sample_Node.vx,Sample_Node.vy);
	return;
}

void Tree::compute_euclidean_neighbors(Node* Cur_Node){
	Euclidean_Neighbors.clear();
	for(list<Node>::iterator it = Vertices.begin();it != Vertices.end(); it++){
		if(get_euclidian_distance(Cur_Node,&(*it)) < MAX_RAD){
			Euclidean_Neighbors.push_back(&(*it));
		}
	}

	return;
}

void Tree::expand_tree(){
	generate_sample_Node();
	Node* best_parent = NULL;
	float best_node_cost = 10000.0; 
	float cur_edge_cost, cur_node_cost, best_time;
	float opt_time = 0;
	bool root_check;

	// First Rewiring Starts
	compute_euclidean_neighbors(&Sample_Node);
	for(list<Node*>::iterator it = Euclidean_Neighbors.begin();it != Euclidean_Neighbors.end(); it++){
		root_check = optimal_arrival_time(*it, &Sample_Node, &opt_time);
		if(root_check){
			cur_edge_cost = cost_of_path(*it, &Sample_Node, opt_time);
			if (cur_edge_cost < MAX_COST){
				cur_node_cost = (*it)->cost + cur_edge_cost;
				if (cur_node_cost < best_node_cost){
					if(compute_trajectory(*it, &Sample_Node, opt_time)){
						best_node_cost = cur_node_cost; 
						best_parent = (*it);
						best_time = opt_time;
					}
				}
			}
		}
	}

	if (best_parent != NULL){
		Node* sample_node_address;

		Sample_Node.node_id = Vertices.size()+1;
		Sample_Node.cost = best_node_cost;
		Sample_Node.parent = best_parent;

		Vertices.push_back(Sample_Node);

		sample_node_address = &Vertices.back();

		add_trajectory(best_parent, sample_node_address, best_time, best_node_cost);
	}

	// Second Rewiring Starts

	return;
}

bool Tree::is_valid_Node(Node* Cur_Node){
	int gridposx = (int)(Cur_Node->x / RES + 0.5);
	int gridposy = (int)(Cur_Node->y / RES + 0.5);

	if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
		return false;
	}
	if ((int)grid_map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
		return false;
	}
	return true;
}

bool Tree::is_valid_Node(Point2D* Cur_Point){
	int gridposx = (int)(Cur_Point->x / RES + 0.5);
	int gridposy = (int)(Cur_Point->y / RES + 0.5);

	if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
		return false;
	}
	if ((int)grid_map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
		return false;
	}
	return true;
}

float Tree::get_neighbourhood_distance(){
	float distance;
	int number_of_vertices = Vertices.size();
	distance = MIN(pow(((GAMMA/VOL_HYPER)*(log(number_of_vertices)/number_of_vertices)),(float)1/(float)4),number_of_vertices);
	return distance;
}

void Tree::propagate_costs(Node* Parent_node){
	// int num_of_children = Parent_node->children.size();
	
	// if (num_of_children == 0)
	// 	return;
	
	float distance;
	for (list<Edge>::iterator it = Parent_node->children.begin(); it != Parent_node->children.end(); it++){
		distance = it->edge_cost;
		(*it).child->cost = Parent_node->cost + distance;
		this->propagate_costs((*it).child);
	}
	return;
}

void Tree::print_node(Node* Cur_Node){
	mexPrintf("node_id = %d\n",Cur_Node->node_id);
	mexPrintf("parent_id = %d\n",Cur_Node->parent->node_id);
	mexPrintf("Cost = %f\n",Cur_Node->cost);
	mexPrintf("Number of children: %d\n",Cur_Node->children.size());
	mexPrintf("x = %f\n",Cur_Node->x);
	mexPrintf("y = %f\n",Cur_Node->y);
	mexPrintf("vx = %f\n",Cur_Node->vx);
	mexPrintf("vy = %f\n",Cur_Node->vy);
	mexPrintf("\n");
}

void Tree::print_tree(){
	mexPrintf("Printing the Tree\n");
	for (list<Node>::iterator it = Vertices.begin(); it != Vertices.end(); it++){
		print_node(&(*it));
	}
}

float Tree::cost_of_path(Node* Start, Node* Goal, float tau){
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

float Tree::diff_cost_of_path(float x1, float x2, float x3, float x4, float y1, float y2, float y3, float y4, float tau){
	float value = 0;

	value = (12*r*y3*(2*x1 - 2*y1 + tau*x3 + tau*y3))/pow(tau,3) - (4*r*pow((3*x2 - 3*y2 + tau*x4 + 2*tau*y4),2))/pow(tau,4) - (4*r*pow((3*x1 - 3*y1 + tau*x3 + 2*tau*y3),2))/pow(tau,4) + (12*r*y4*(2*x2 - 2*y2 + tau*x4 + tau*y4))/pow(tau,3) + 1;

	return value;
}

bool Tree::optimal_arrival_time(Node* Start, Node* Goal, float* opt_time){

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

bool Tree::compute_trajectory(Node* Start, Node* Goal, float t_star){
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

	Point2D temp_point;

    // Add Collision Check here, If collision happens break it
	while(t < t_star){
		temp_point.x = y1 + y3*(t - t_star) - (((4*r*(x3 - y3))/t_star - (6*r*(x1 - y1 + t_star*x3))/pow(t_star,2))*pow(t - t_star,2))/(2*r) - (((6*r*(x3 - y3))/pow(t_star,2) - (12*r*(x1 - y1 + t_star*x3))/pow(t_star,3))*pow(t - t_star,3))/(6*r);
		temp_point.y = y2 + y4*(t - t_star) - (((4*r*(x4 - y4))/t_star - (6*r*(x2 - y2 + t_star*x4))/pow(t_star,2))*pow(t - t_star,2))/(2*r) - (((6*r*(x4 - y4))/pow(t_star,2) - (12*r*(x2 - y2 + t_star*x4))/pow(t_star,3))*pow(t - t_star,3))/(6*r);
		if(!is_valid_Node(&temp_point)){
			return false;
		}
		t += t_delta;
	}
	return true;	
}

void Tree::add_trajectory(Node* Start, Node* Goal, float t_star, float edge_cost){
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

	Point2D temp_point;

	// Creating the edge
	Edge temp_edge;
	temp_edge.parent = Start;
	temp_edge.child = Goal;
	temp_edge.edge_cost = edge_cost;

	while(t < t_star){
		temp_point.x = y1 + y3*(t - t_star) - (((4*r*(x3 - y3))/t_star - (6*r*(x1 - y1 + t_star*x3))/pow(t_star,2))*pow(t - t_star,2))/(2*r) - (((6*r*(x3 - y3))/pow(t_star,2) - (12*r*(x1 - y1 + t_star*x3))/pow(t_star,3))*pow(t - t_star,3))/(6*r);
		temp_point.y = y2 + y4*(t - t_star) - (((4*r*(x4 - y4))/t_star - (6*r*(x2 - y2 + t_star*x4))/pow(t_star,2))*pow(t - t_star,2))/(2*r) - (((6*r*(x4 - y4))/pow(t_star,2) - (12*r*(x2 - y2 + t_star*x4))/pow(t_star,3))*pow(t - t_star,3))/(6*r);
		temp_edge.trajectory.push_back(temp_point);
		t += t_delta;
	}

	// Adding the edge as a child of Start
	Start->children.push_back(temp_edge);
	return;
}