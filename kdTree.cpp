#include <list>
#include "mex.h"
#include "plannerheader2.hpp"
#include <math.h>
#include "constants.hpp"

using namespace std;

void printNode(kdTreeNode* n){
    if(n==NULL)
        mexPrintf("Node is NULL \n"); 
    else{
        mexPrintf("X=%f , Y=%f , leftNode=%x, rightNode=%x \n", (n->point)[0], (n->point)[1], n->left, n->right); 
        mexPrintf("kdTree node data x=%f, y=%f, vx=%f,vy=%f \n", (n->data)->x, (n->data)->y, (n->data)->vx, (n->data)->vy);
    }
    return; 
}

kdTreeNode* createKDTreeNode(Node* newNode){
    
  //mexPrintf("New node to be inserted X = %f, Y= %f \n", newNode->x, newNode->y); 
  kdTreeNode* tmp = new kdTreeNode;
  
  tmp->data = newNode;
  (tmp->point)[0] = newNode->x;
  (tmp->point)[1] = newNode->y; 
  tmp->left = NULL;
  tmp->right = NULL;
  mexPrintf("printing new kdTree node inserted \n"); 
  printNode(tmp); 
  return tmp; 
}

kdTreeNode* insertKDTree(kdTreeNode* r,  Node* newNode, unsigned int depth){

  float newPoint[2];
  newPoint[0] = newNode->x;
  newPoint[1] = newNode->y; 
  
  
  if(r==NULL){
    //mexPrintf("NULL \n"); 
    r = createKDTreeNode(newNode);
    return r;
  }

  int currDim = depth % DIM;
  
  if(newPoint[currDim] < r->point[currDim]){
    r->left = insertKDTree(r->left, newNode, depth+1);
  }
  else{
    r->right = insertKDTree(r->right, newNode, depth+1);
  }

  return r;
}

float getEuclidDist(kdTreeNode* root,  Node* testPt){

  return pow(pow(root->point[0] - testPt->x,2.0) + pow(root->point[1] - testPt->y,2.0),0.5);
}

void nearestNeighbours( kdTreeNode* r, Node* testPt, int currDim, std::list<Node*>* neighbours){
  //Note before run this, make sure and clear the nearest neighbour list in the class

  float newPoint[2];
  newPoint[0] = testPt->x;
  newPoint[1] = testPt->y;
  
  if(r==NULL)
    return; 

  float dist = getEuclidDist(r, testPt);
  float dimDist = r->point[currDim] - newPoint[currDim];

  if(dist < MAX_RAD){
    //mexPrintf("printing node to be inserted into the nearest  neighbours list \n");
    //mexPrintf("dist=%f \n", dist);
    //printNode(r);
    (*neighbours).push_back(r->data); 
  }

  currDim = (currDim+1) % DIM;

  nearestNeighbours(dimDist > 0 ? r->left : r->right, testPt, currDim, neighbours);
  
  if(dimDist >= MAX_RAD)
    return;

  nearestNeighbours(dimDist > 0 ? r->right : r->left, testPt, currDim, neighbours);
}


