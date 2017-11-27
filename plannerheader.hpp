#ifndef PLANNER_HEADER_H   
#define PLANNER_HEADER_H

#include <list>

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
	Node* Start_Node;
	Node Goal_Node;

	double* grid_map;
	int x_size;
	int y_size;

	Node Sample_Node;
	list<Node*> Euclidean_Neighbors;

	//System Parameters
	float r;

public:
	// Constructors
	Tree(Node*, Node*, double*, int, int);

	// Deconstructors
	~Tree();

	// Accesor Methods
	Node* get_Start();
	Node* get_Goal();

	// Other Methods
	void generate_sample_Node();
	void compute_euclidean_neighbors(Node*);
	void expand_tree();
	bool is_valid_Node(Node*);
	bool is_valid_Node(Point2D*);
	float get_neighbourhood_distance();
	void propagate_costs(Node*);
	void print_node(Node*);
	void print_tree();

	// System Methods
	float cost_of_path(Node*, Node*, float);
	float diff_cost_of_path(float, float, float, float, float, float, float, float, float);
	bool compute_trajectory(Node*, Node*, float);
	void add_trajectory(Node*, Node*, float, float);
	bool optimal_arrival_time(Node*, Node*, float*);
};


#endif 
