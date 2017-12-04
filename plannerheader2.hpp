#ifndef PLANNER_HEADER_2_H   
#define PLANNER_HEADER_2_H

#include <list>
#include <random>
#include "kdTree.hpp"

typedef struct Point2D Point2D;
typedef struct Edge Edge;
typedef struct Node Node;

using namespace std;

// Helper Function Prototypes
float get_euclidian_distance(Node*, Node*);
float deg2rad(float);
float rad2deg(float);
void wrap_to_pi(float*);

// Structures
struct Point2D{
	float x;
	float y;
};	

struct Edge{	
	Node* child;
	float edge_cost;
};

struct Node{
	int node_id;
	float x;
	float y;
	float vx;
	float vy;
	//float theta;
	// float velocity;
	// float curvature;
	// float time;
	float cost;
	Node* parent;
	float optimal_time;
	std::list<Edge> children;
};

// Classes
class Tree{
private:
	list<Node> Vertices;
	// bool reached;
	Node* Start_Node;
	Node Goal_Node;

	double* grid_map;
	int x_size;
	int y_size;

	Node Sample_Node;
	list<Node*> Euclidean_Neighbors;

	//System Parameters
	float r;

	//Sampling Parameters
	unsigned seed;
	std::default_random_engine generator;
	

public:
	// Constructors
	Tree(Node*, Node*, double*, int, int);

	// Deconstructors
	~Tree();

	// Accesor Methods
	Node* get_Start();
	Node* get_Goal();
	int get_tree_size();

	// Other Methods
	void generate_sample_Node(std::uniform_real_distribution<float>);
	void compute_euclidean_neighbors(Node*);
	void expand_tree(std::uniform_real_distribution<float>, kdTreeNode*);
	bool is_valid_Node(Node*);
	bool is_valid_Node(Point2D*);
	float get_neighbourhood_distance();
	void propagate_costs(Node*);
	void delete_child(Node*);
	void print_node(Node*);
	void print_tree();

	// System Methods
	float cost_of_path(Node*, Node*, float);
	float diff_cost_of_path(float, float, float, float, float, float, float, float, float);
	bool compute_trajectory(Node*, Node*, float);
	bool optimal_arrival_time(Node*, Node*, float*);
};


#endif 
