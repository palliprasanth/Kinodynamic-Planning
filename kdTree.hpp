#ifndef KDTREE_H
#define KDTREE_H

#include <list>
#include "plannerheader2.hpp"

typedef struct Node Node;
typedef struct kdTreeNode kdTreeNode;

using namespace std;

struct kdTreeNode{
    
    Node* data;
    float point[2];
    kdTreeNode* left;
    kdTreeNode* right;
};

class kdTree{    
private:
    
    int dim;
    
    
public:
    //constructor
    kdTree();
    
    //Deconstructor
    ~kdTree();
    
    //Methods
    kdTreeNode* createKDTreeNode(Node* );
    void insertKDTree(kdTreeNode* , Node*, unsigned int);
    void nearestNeighbours(kdTreeNode* ,Node* ,int );
    float getEuclidDist( Node*);
    std::list<Node*> kdTreeNeighbours;
    kdTreeNode* root;
    
};

#endif
