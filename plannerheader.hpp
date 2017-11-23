#ifndef PLANNER_HEADER_H   
#define PLANNER_HEADER_H

#include <list>

typedef struct Node Node;

using namespace std;

// Helper Function Prototypes
float get_euclidian_distance(Node*, Node*);
double deg2rad(double);
double rad2deg(double);
void wrap_to_pi(double*);

// Structures
struct Node{
	int node_id;
	float x;
	float y;
	float theta;
	// float velocity;
	// float curvature;
	// float time;
	float cost;
	Node* parent;
	std::list<Node*> children;
};

// Classes
class Tree{
private:
	list<Node> Vertices;
public:
	// Constructors
	Tree();

	// Deconstructors
	~Tree();

	// Methods
	//void propagate_costs(Node*);
	void print_tree();
};

#endif 