#ifndef PLANNER_HEADER_H   
#define PLANNER_HEADER_H

#include <list>

typedef struct Point2D Point2D;
typedef struct Edge Edge;
typedef struct Node Node;

using namespace std;

// Helper Function Prototypes
float get_euclidian_distance(Node*, Node*);
double deg2rad(double);
double rad2deg(double);
void wrap_to_pi(double*);

// Structures
struct Point2D{
	float x;
	float y;
};	

struct Edge{
	Node* parent;	
	Node* child;
	float edge_cost;
	list<Point2D> trajectory;
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
	std::list<Edge> children;
};

// Classes
class Tree{
private:
	list<Node> Vertices;
	bool reached;
public:
	Node* Start_Node;
	Node* Goal_Node;

	// Constructors
	Tree(Node*, Node*);

	// Deconstructors
	~Tree();

	// Methods
	//void propagate_costs(Node*);
	void print_tree();
};


class SystemDI{
private:
	float r;

public:
	// Constructors
	SystemDI();

	// Deconstructors
	~SystemDI();

	// Methods
	float cost_of_path(Node*, Node*, float);
	float diff_cost_of_path(float, float, float, float, float, float, float, float, float);
	void compute_trajectory(Node*, Node*, float);
	bool optimal_arrival_time(Node*, Node*, float*);
};

#endif 