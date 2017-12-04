#ifndef KDTREE_H
#define KDTREE_H

#include <list>
#include "plannerheader2.hpp"

#define DIM 2

typedef struct Node Node;
typedef struct kdTreeNode kdTreeNode;

using namespace std;

struct kdTreeNode{
    
    Node* data;
    float point[2];
    kdTreeNode* left;
    kdTreeNode* right;
};


kdTreeNode* createKDTreeNode(Node* );
kdTreeNode* insertKDTree(kdTreeNode* , Node*, unsigned int);
void nearestNeighbours(kdTreeNode* ,Node* ,int,std::list<Node*>* );
float getEuclidDist( Node*);
void printNode(kdTreeNode*);
    


#endif
