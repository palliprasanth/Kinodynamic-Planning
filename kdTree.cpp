#include <list>
#include "plannerheader2.hpp"
#include <math.h>
#include "constants.hpp"

using namespace std;

kdTreeNode* kdTree::createKDTreeNode(Node* newNode){
  kdTreeNode* tmp = new kdTreeNode;
  
  tmp->data = newNode;
  tmp->point[0] = newNode->x;
  tmp->point[1] = newNode->y; 
  tmp->left = NULL;
  tmp->right = NULL;
  return tmp; 
}

void kdTree::insertKDTree(kdTreeNode* root,  Node* newNode, unsigned int depth){

  float newPoint[2];
  newPoint[0] = newNode->x;
  newPoint[1] = newNode->y; 
  
  //kdTree is empty 
  if(root==NULL){
    root = createKDTreeNode(newNode);
  }

  int currDim = depth % dim;
  
  if(newPoint[currDim] < root->point[currDim]){
    insertKDTree(root->left, newNode, depth+1);
  }
  else{
    insertKDTree(root->right, newNode, depth+1);
  }

  return;
}

float kdTree::getEuclidDist( Node* testPt){

  return pow(pow(root->point[0] - testPt->x,2.0) + pow(root->point[1] - testPt->y,2.0),0.5);
}

void kdTree::nearestNeighbours( kdTreeNode* root, Node* testPt, int currDim){
  //Note before run this, make sure and clear the nearest neighbour list in the class

  float newPoint[2];
  newPoint[0] = testPt->x;
  newPoint[1] = testPt->y;
  
  if(root==NULL)
    return; 

  float dist = getEuclidDist(testPt);
  float dimDist = root->point[currDim] - newPoint[currDim];

  if(dist < MAX_RAD)
    kdTreeNeighbours.push_back(root->data); 

  currDim = (currDim+1) % dim;

  nearestNeighbours(dimDist > 0 ? root->left : root->right, testPt, currDim);
  
  if(dimDist >= MAX_RAD)
    return;

  nearestNeighbours(dimDist > 0 ? root->right : root->left, testPt, currDim);
}

kdTree::kdTree(){

  dim=2; 
  root = NULL;
}

kdTree::~kdTree(){
  
}
