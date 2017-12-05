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
	temp.time = Start->time;
	temp.cost = 0;
	temp.parent = NULL;

	Vertices.push_back(temp);

	Start_Node = &Vertices.back();

	Goal_Node.node_id = -1;
	Goal_Node.x = Goal->x;
	Goal_Node.y = Goal->y;
	Goal_Node.vx = Goal->vx;
	Goal_Node.vy = Goal->vy;
	Goal_Node.time = Goal->time;
	Goal_Node.cost = 10000.0;
	Goal_Node.parent = NULL;

	// reached = false;

	// System Properties Initializing
	r = 0.25;

	// Sampling properties
	seed = std::chrono::system_clock::now().time_since_epoch().count();
	generator.seed(seed);
	//uni_distribution.min(0);
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

int Tree::get_tree_size(){
	return Vertices.size();
}

void Tree::generate_sample_Node(std::uniform_real_distribution<float> uni_distribution){


	bool valid_sample = false;

	while (!valid_sample){
		Sample_Node.x = X_MIN + uni_distribution(generator)*(X_MAX-X_MIN);
		Sample_Node.y = Y_MIN + uni_distribution(generator)*(Y_MAX-Y_MIN);
		Sample_Node.vx = VX_MIN + uni_distribution(generator)*(VX_MAX-VX_MIN);
		Sample_Node.vy = VY_MIN + uni_distribution(generator)*(VY_MAX-VY_MIN);

		valid_sample = is_valid_Node(&Sample_Node);
	}
	//mexPrintf("Sample Node Generated\n");
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

void Tree::expand_tree(std::uniform_real_distribution<float> uni_distribution){
	//mexPrintf("Expanding Tree\n");
	generate_sample_Node(uni_distribution);
	Node* best_parent = NULL;
	float best_node_cost = 10000.0; 
	float cur_edge_cost, cur_node_cost, best_time, best_edge_cost;
	float opt_time = 0;
	bool root_check;
	Edge child_edge;
	Node* sample_node_address;

	// First Rewiring Starts
	compute_euclidean_neighbors(&Sample_Node);

	//mexPrintf("Number of neighbors: %d\n", Euclidean_Neighbors.size());
	for(list<Node*>::iterator it = Euclidean_Neighbors.begin();it != Euclidean_Neighbors.end(); it++){
		root_check = optimal_arrival_time(*it, &Sample_Node, &opt_time);
		if(root_check){
			cur_edge_cost = cost_of_path(*it, &Sample_Node, opt_time);
			if (cur_edge_cost < MAX_EDGE_COST){
				cur_node_cost = (*it)->cost + cur_edge_cost;
				if (cur_node_cost < best_node_cost){
					if(compute_trajectory(*it, &Sample_Node, opt_time)){
						best_node_cost = cur_node_cost;
						best_edge_cost = cur_edge_cost; 
						best_parent = (*it);
						best_time = opt_time;
					}
				}
			}
		}
	}

	if (best_parent != NULL){
		Sample_Node.node_id = Vertices.size()+1;
		Sample_Node.cost = best_node_cost;
		Sample_Node.time = best_parent->time + best_time;
		Sample_Node.parent = best_parent;
		Sample_Node.optimal_time = best_time;
		Vertices.push_back(Sample_Node);
		sample_node_address = &Vertices.back();

		// Add child info here
		child_edge.edge_cost = best_edge_cost;
		child_edge.child = sample_node_address;

		best_parent->children.push_back(child_edge);
	}
	else{
		return;
	}

	// Second Rewiring Starts
	for(list<Node*>::iterator it = Euclidean_Neighbors.begin();it != Euclidean_Neighbors.end(); it++){
		if((*it)->node_id != best_parent->node_id){
			root_check = optimal_arrival_time(&Sample_Node, *it, &opt_time);
			if(root_check){
				cur_edge_cost = cost_of_path(&Sample_Node, *it, opt_time);
				if (cur_edge_cost < MAX_EDGE_COST){
					if (Sample_Node.cost + cur_edge_cost < (*it)->cost){
						if(compute_trajectory(&Sample_Node, *it, opt_time)){
							// Do rewiring stuff here
							// Delete current node from the list of children of its parent
							delete_child(*it);
							(*it)->cost = Sample_Node.cost + cur_edge_cost;
							(*it)->time = Sample_Node.time + opt_time;
							(*it)->parent = sample_node_address;
							(*it)->optimal_time = opt_time;
							propagate_costs(*it);
						}
					}
				}
			}
		}
	}

	// Repeat the same for goal node	
	root_check = optimal_arrival_time(&Sample_Node, &Goal_Node, &opt_time);
	if(root_check){
		//mexPrintf("Root Check Goal\n");
		cur_edge_cost = cost_of_path(&Sample_Node, &Goal_Node, opt_time);
		//mexPrintf("Edge Cost: %f\n", cur_edge_cost);
		if (cur_edge_cost < MAX_EDGE_COST){
			if (Sample_Node.cost + cur_edge_cost < Goal_Node.cost){
				if(compute_trajectory(&Sample_Node, &Goal_Node, opt_time)){
					// Do rewiring stuff here
					// Delete current node from the list of children of its parent
					//mexPrintf("Goal Connected\n");
					delete_child(&Goal_Node);
					Goal_Node.cost = Sample_Node.cost + cur_edge_cost;
					Goal_Node.time = Sample_Node.time + opt_time;
					Goal_Node.parent = sample_node_address;
					Goal_Node.optimal_time = opt_time;
					propagate_costs(&Goal_Node);
				}
			}
		}
	}
	
	return;
}

bool Tree::is_valid_Node(Node* Cur_Node){
	int gridposx = (int)(Cur_Node->x / RES);
	int gridposy = (int)(Cur_Node->y / RES);

	if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
		return false;
	}
	if (grid_map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
		return false;
	}

	return true;
}

bool Tree::is_valid_Node(Point2D* Cur_Point){
	int gridposx = (int)(Cur_Point->x / RES);
	int gridposy = (int)(Cur_Point->y / RES);

	if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
		return false;
	}
	if (grid_map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
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

void Tree::propagate_costs(Node* Parent_Node){	
	float distance;
	for (list<Edge>::iterator it = Parent_Node->children.begin(); it != Parent_Node->children.end(); it++){
		distance = it->edge_cost;
		(*it).child->cost = Parent_Node->cost + distance;
		(*it).child->time = Parent_Node->time + (*it).child->optimal_time;
		this->propagate_costs((*it).child);
	}
	return;
}

void Tree::delete_child(Node* Child_Node){
	Node* parent = Child_Node->parent;
	if (parent != NULL ){
		for (list<Edge>::iterator it = parent->children.begin(); it != parent->children.end(); it++){
			if (it->child->node_id == Child_Node->node_id){
				parent->children.erase(it);
			}
		}
	}
	return;
}

void Tree::print_node(Node* Cur_Node){
	mexPrintf("node_id = %d\n",Cur_Node->node_id);
	if (Cur_Node->parent != NULL){
		mexPrintf("parent_id = %d\n",Cur_Node->parent->node_id);
	}
	mexPrintf("Cost = %f\n",Cur_Node->cost);
	mexPrintf("Time = %f\n", Cur_Node->time);
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
	print_node(&Goal_Node);
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
	float t_final = T_MAX;
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

	float t_delta = T_DIFF;
	float t = t_delta;

	Node temp_node;

    // Add Collision Check here, If collision happens break it
	while(t < t_star){
		temp_node.x = (2*pow(t,3)*x1 + pow(t_star,3)*x1 - 2*pow(t,3)*y1 - 3*pow(t,2)*t_star*x1 + t*pow(t_star,3)*x3 + pow(t,3)*t_star*x3 + 3*pow(t,2)*t_star*y1 + pow(t,3)*t_star*y3 - 2*pow(t,2)*pow(t_star,2)*x3 - pow(t,2)*pow(t_star,2)*y3)/pow(t_star,3);
		temp_node.y = (2*pow(t,3)*x2 + pow(t_star,3)*x2 - 2*pow(t,3)*y2 - 3*pow(t,2)*t_star*x2 + t*pow(t_star,3)*x4 + pow(t,3)*t_star*x4 + 3*pow(t,2)*t_star*y2 + pow(t,3)*t_star*y4 - 2*pow(t,2)*pow(t_star,2)*x4 - pow(t,2)*pow(t_star,2)*y4)/pow(t_star,3);
		temp_node.time = Start->time + t;
		if(!is_valid_Node(&temp_node)){
			return false;
		}
		t += t_delta;
	}
	return true;	
}