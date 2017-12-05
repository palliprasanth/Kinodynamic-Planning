#include "mex.h"
#include "plannerheader2.hpp"
#include "kdTree.hpp"
#include <list>

using namespace std;
void test(){
    kdTreeNode* kdTrRoot = NULL; 
    Node* n1 = new Node; 
    n1->x = 5; 
    n1->y = 5; 
    kdTrRoot = insertKDTree(kdTrRoot, n1, 0); 
    mexPrintf("ROOT \n"); 
    printNode(kdTrRoot);
    
    Node* n2 = new Node; 
    n2->x = 1; 
    n2->y = 2; 
    kdTrRoot = insertKDTree(kdTrRoot, n2, 0);
    
    
    Node* n3 = new Node; 
    n3->x = 10; 
    n3->y = 26; 
    kdTrRoot = insertKDTree(kdTrRoot, n3, 0);
    
    
    mexPrintf("ROOT LEFT --->"); 
    printNode(kdTrRoot->left);
    mexPrintf("ROOT RIGHT --->"); 
    printNode(kdTrRoot->right);
    
    Node* testPt = new Node; 
    testPt->x = 0.5; 
    testPt->y = 0.5; 
    
    list<Node*> nn; 
    nearestNeighbours(kdTrRoot, testPt, 0, &nn); 
    
    for(list<Node*>::iterator it = nn.begin();it != nn.end(); it++){
		mexPrintf("Node X=%f, Y=%f \n", (*it)->x, (*it)->y);
    }
    
    return; 
}


void mexFunction(int nlhs, mxArray *plhs[],
	int nrhs, const mxArray*prhs[] ){
    test(); 
    return ;
}
