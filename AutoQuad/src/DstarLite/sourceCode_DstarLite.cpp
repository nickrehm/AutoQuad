/*
Copyright 2018, Michael Otte

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <vector>
using namespace std;


#include "heap.h"
#include "heap.cpp"

#include "graph.h"

// returns random number between 0 and 1
float rand_f()
{
  return (float)rand() / (float)RAND_MAX;
}

bool saveBlankEdgeFile(const char* Filename, const char* NewFilename) {
  FILE * pFile;

  int numEdges;

  // open the edge file
      pFile = fopen(Filename, "r");
      if (pFile==NULL) 
      {
        printf("unable to open file %s\n", Filename);
        return false;
      }

      // read the number of edges 
      if(fscanf(pFile, "%d\n", &numEdges) < 1)
      {
        printf("problem reading edge number from file\n"); 
        return false;
      }

      int node1_arr [numEdges];
      int node2_arr [numEdges];
      float cost_arr [numEdges];

      // now read the edges one at a time
      int startNodeID, endNodeID;
      float cost;
      for(int m = 0; m < numEdges; m++)
      {
        if(fscanf(pFile, "%d, %d, %f\n", &startNodeID, &endNodeID, &cost) < 2)
        {
          printf("problem reading the %d-th edge from file\n",m); 
          return false;
        }

        node1_arr[m] = startNodeID;
        node2_arr[m] = endNodeID;
        cost_arr[m] = cost;
      }

      // close the node file
      fclose(pFile);

      ///////////////////////////////

      pFile = fopen(NewFilename,"w");      
      if(pFile == NULL)
      {
        return false;
      }

      //ADD THE MF HEADER!!!!
      fprintf(pFile, "%d\n", numEdges);

      for(int i = 0; i<numEdges; i++)
      {
        fprintf(pFile, "%d, %d, %f\n",node1_arr[i], node2_arr[i], cost_arr[i]);
      }

      fclose(pFile);

      printf("saved updated edge data in %s\n", NewFilename);

      return 1;
}



double smallest(double a1, double a2) { //returns min of two values
  if (a1 > a2) {
    return a2;
  }
  else {
    return a1;
  }

}



float calculateKey1(Node* thisNode, Node* startNode, float km) {
    float key;
    float heuristic;

    heuristic = sqrt((thisNode->x - startNode->x)*(thisNode->x - startNode->x) + (thisNode->y - startNode->y)*(thisNode->y - startNode->y)); //distance to goal heuristic

    //heuristic = abs(thisNode->x - startNode->x) + abs(thisNode->y - startNode->y);

    key = smallest(thisNode->g, thisNode->rhs) + heuristic + km;
    thisNode->key = key;

    return key;
}

float calculateKey2(Node* thisNode) {
    float key;
    //float heuristic;

    //heuristic = sqrt((thisNode->x - startNode->x)*(thisNode->x - startNode->x) + (thisNode->y - startNode->y)*(thisNode->y - startNode->y)); //distance to goal heuristic

    //heuristic = abs(thisNode->x - startNode->x) + abs(thisNode->y - startNode->y);

    key = smallest(thisNode->g, thisNode->rhs);
    //thisNode->key = key;

    return key;
}




void updateNode(Node* thisNode, Node* startNode, Node* goalNode, Heap<Node> &H, float km) {
  double smallest_cost = LARGE;

  //thisNode->parentNode = NULL; //this removes node from search tree, so that it isn't re-added if edge-costs change

  if (thisNode->id != goalNode->id) {
    //thisNode->rhs = LARGE; //this isn't in the paper...
    for (int n = 0; n < thisNode->numOutgoingEdges; n++) { //sucessors
      Edge* thisEdge = thisNode->outgoingEdges[n];  //pointer to this edge
      Node* sucessor = thisEdge->endNode;  //pointer to the node on the other end of this edge
      //thisNode->rhs = smallest(thisNode->rhs, sucessor->g + thisEdge->edgeCost);
      if (smallest_cost > sucessor->g + thisEdge->edgeCost) { //this is to set the parent node to store the solutions, not included in paper
        smallest_cost = sucessor->g + thisEdge->edgeCost;
        thisNode->rhs = sucessor->g + thisEdge->edgeCost;
        thisNode->parentNode = sucessor;
      }
    }

    smallest_cost = LARGE;
    for (int n = 0; n < thisNode->numIncommingEdges; n++) { //predecessors.....used to assign parents, not part of d* Lite
      Edge* thisEdge = thisNode->incommingEdges[n];  //pointer to this edge
      Node* predecessor = thisEdge->startNode;  //pointer to the node on the other end of this edge
      if (smallest_cost > predecessor->g + thisEdge->edgeCost) { //this is to set the parent node to store the solutions, not included in paper
        smallest_cost = predecessor->g + thisEdge->edgeCost;

        //thisNode->parentNode = predecessor;
        //sucessor->parentNode = thisNode; //nope
      }
    }
  }

  if (thisNode->inHeap) {
    H.removeNodeFromHeap(thisNode);
  }

  if (thisNode->g != thisNode->rhs) { //inconsistent, add back to heap
    H.addToHeap(thisNode, calculateKey1(thisNode,startNode,km));
  }
}


bool compareKeys(float k1, float k2, float kp1, float kp2) {
  bool first = false;
  bool second = false;

  double eps = 0;//0.0000000000000001;

  if (k1 < (kp1 + eps)) {
    first = true;
  }
  if (k1 == kp1 && k2 < (kp2 + eps)) {
    second = true;
  }

  if (first || second) {
    return true;
  }
  else {
    return false;
  }
  
}


void computeShortestPath(Graph &G, Heap<Node> &H, Node* startNode, Node* goalNode, int startNodeIndex, int goalNodeIndex, float km) {
  //cout << "computing shortest path...";
  float eps = 0.01;
  int overflow = 0;

  //while ((H.topKey() < (calculateKey1(startNode,startNode,km) + eps)) || (startNode->rhs > startNode->g)) {
  //while (compareKeys(H.topKey(), calculateKey2(&G.nodes[H.topKeyIndex()]), calculateKey1(startNode,startNode,km)+eps, calculateKey2(startNode)+eps) || (startNode->rhs > startNode->g)) {
  while (true) {
    float kold [2] = {H.topKey(), calculateKey2(&G.nodes[H.topKeyIndex()])};
    Node* thisNode = H.popHeap();
    overflow = overflow + 1;

    //if (kold < calculateKey1(thisNode, startNode, km)) {
    if (compareKeys(kold[0],kold[1],calculateKey1(thisNode, startNode, km),calculateKey2(thisNode))) {
      H.addToHeap(thisNode, calculateKey1(thisNode,startNode,km));
    }

    else if (thisNode->g > thisNode->rhs) { 
      thisNode->g = thisNode->rhs;
      for(int n = 0; n < thisNode->numIncommingEdges; n++) { //predecessor
        Edge* thisEdge = thisNode->incommingEdges[n];  //pointer to this edge
        Node* predecessor = thisEdge->startNode;  //pointer to the node on the other end of this edge
        updateNode(predecessor,startNode,goalNode,H,km);
      }
    }

    else {
      thisNode->g = LARGE;
      for(int n = 0; n < thisNode->numIncommingEdges; n++) { //predecessor
        Edge* thisEdge = thisNode->incommingEdges[n];  //pointer to this edge
        Node* predecessor = thisEdge->startNode;  //pointer to the node on the other end of this edge
        updateNode(predecessor,startNode,goalNode,H,km);
      }
      updateNode(thisNode,startNode,goalNode,H,km); //thisNode is union with all predecessors
    }

    if (H.topHeap() == NULL ) {
      cout << "emptied heap in computeShortestPath :( ";
      break;
    }
    if (overflow > 100000) {
      break;
    }

  }
}


bool updateEdgeCostsandNodes(Graph &G, Heap<Node> &H, Node* startNode, Node* goalNode, float km, const char* nodeFilename, const char* edgeFilename, float x_obs, float y_obs, float r_obs) {
  FILE * pFile;
  int numNodes, numEdges;

      // open the node file
      pFile = fopen( nodeFilename , "r");
      if(pFile==NULL) 
      {
        printf("unable to open file %s\n", nodeFilename);
        return false;
      }

      // read the number of nodes 
      if(fscanf(pFile, "%d\n", &numNodes) < 1)
      {
        printf("problem reading node number from file\n"); 
        return false;
      }

      int id_arr [numNodes];
      float x_arr [numNodes];
      float y_arr [numNodes];

      // now read the nodes one at a time
      int id;
      float x, y;
      for(int n = 0; n < numNodes; n++)
      {
        if(fscanf(pFile, "%d, %f, %f\n", &id, &x, &y) < 3)
        {
          printf("problem reading the %d-th node from file\n",n); 
          return false;
        }
        //input the data
        id_arr[n] = id-1; //c++ 0-based indexing
        x_arr[n] = x;
        y_arr[n] = y;
      }

      // close the node file
      fclose(pFile);


  // open the edge file
      pFile = fopen(edgeFilename, "r");
      if (pFile==NULL) 
      {
        printf("unable to open file %s\n", edgeFilename);
        return false;
      }

      // read the number of edges 
      if(fscanf(pFile, "%d\n", &numEdges) < 1)
      {
        printf("problem reading edge number from file\n"); 
        return false;
      }

      int node1_arr [numEdges];
      int node2_arr [numEdges];
      float cost_arr [numEdges];
      

      // now read the edges one at a time
      int startNodeID, endNodeID;
      float cost;
      for(int m = 0; m < numEdges; m++)
      {
        if(fscanf(pFile, "%d, %d, %f\n", &startNodeID, &endNodeID, &cost) < 2)
        {
          printf("problem reading the %d-th edge from file\n",m); 
          return false;
        }

        node1_arr[m] = startNodeID;
        node2_arr[m] = endNodeID;
        cost_arr[m] = cost;
      }

      // close the node file
      fclose(pFile);

      /////////////////////////////////////////////////////////////////////////////////////
      //NOW WE DO THE BIG BOY SHIT
      //nodes: id_arr, x_arr, y_arr
      //edges: node1_arr, node2_arr, cost_arr
      //obstacle to be injected: x_obs, y_obs, r_obs

      vector<float> obstructed_node_ids;
      float dist_check;

      //loop through nodes arrays and store id if x and y coords are within the obstacle
      for (int i = 0; i<numNodes; i++) {
        dist_check = sqrt((x_obs - x_arr[i])*(x_obs - x_arr[i]) + (y_obs - y_arr[i])*(y_obs - y_arr[i]));
        if (dist_check <= r_obs) {
          obstructed_node_ids.push_back(id_arr[i]);
        }
      }

      //loop through edges arrays and any with the obstructed ids, set their cost to infinity
      for (int i = 0; i<numEdges; i++) { //loop through all edge nodes
        for (int j = 0; j<obstructed_node_ids.size(); j++) { //loop through all obstructed node ids
          if (node1_arr[i] == obstructed_node_ids[j] || node2_arr[i] == obstructed_node_ids[j]) {
            cost_arr[i] = 10000000.0; //'infinity'
          }
        }
      }

      //Now we spit that data shit back into a new edge.txt file (overwrite the original)
      pFile = fopen(edgeFilename,"w");      
      if(pFile == NULL)
      {
        return false;
      }

      //ADD THE MF HEADER!!!!
      fprintf(pFile, "%d\n", numEdges);

      for(int i = 0; i<numEdges; i++)
      {
        fprintf(pFile, "%d, %d, %f\n",node1_arr[i], node2_arr[i], cost_arr[i]);
      }

      fclose(pFile);

      printf("saved updated edge data in %s\n", edgeFilename);

      /////////////////////////////////////////////////////////////////////////////////////////
      //NOW UPDATE KEYS OF AFFECTED NODES IN HEAP AND INSERT THEM TO TOP
      //H.printHeap();

    //loop over obstructed nodes and update edge costs in G, update the node too
    for (int j = 0; j < obstructed_node_ids.size(); j++) {
      for(int i = 0; i < G.numEdges; i++) {
        if (G.edges[i].startNode->id == obstructed_node_ids[j] || G.edges[i].endNode->id == obstructed_node_ids[j]) {
          G.edges[i].edgeCost = LARGE;
          //G.edges[i].endNode->key = LARGE;
          //G.edges[i].startNode->key = LARGE;
          G.edges[i].endNode->parentNode = NULL;
          G.edges[i].endNode->g = LARGE;
          G.edges[i].endNode->rhs = LARGE;
          G.edges[i].startNode->parentNode = NULL;
          G.edges[i].startNode->g = LARGE;
          G.edges[i].startNode->rhs = LARGE;

          updateNode(G.edges[i].endNode, startNode, goalNode, H, km);
          updateNode(G.edges[i].startNode, startNode, goalNode, H, km);
        }
      }
      
      /*
      for(int k = 0; k < G.numNodes; k++) {
        if (G.nodes[k].id == obstructed_node_ids[j]) {
          updateNode(&G.nodes[k], startNode, goalNode, H); //need to pass startNode and goalNode into this function...
        }
      }
      */
      
    }

    //cout << "finished updating shit";

    return true;

}





/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main()
{
  int replan_counter = 0;
  //we want to find a path that goes from here to here
  int startNodeIndex = 12; //subtract 1 from desired 
  int goalNodeIndex = 952; //subtract 1 from desired

  saveBlankEdgeFile("edges_blank.txt", "edges.txt"); //makes a copy of blank edge file that we will update with addition of obstacles later

  //create graph
  Graph G; 
  G.readGraphFromFiles("nodes_blank.txt", "edges_blank.txt");

  //set rhs and g values in G to default values
  for (int i = 0; i < G.numNodes; i++) {
    Node* thisNode =  &G.nodes[i];
    thisNode->g = LARGE; //'infinity'
    thisNode->rhs = LARGE; //'infinity'
  }

  float km = 0; //key modifier

  //initialize heap
  Heap<Node> H(100); // this is the heap (start's with space for 100 items
                     // but will grow automatically as needed).

  //these are pointers to the start and end nodes
  Node* startNode = &G.nodes[startNodeIndex];
  Node* lastNode = startNode;
  Node* goalNode = &G.nodes[goalNodeIndex];
  goalNode->rhs = 0;

  H.addToHeap(goalNode, calculateKey1(goalNode,startNode,km));

  //COMPUTE INITIAL SOLUTION
  computeShortestPath(G,H,startNode,goalNode,startNodeIndex,goalNodeIndex,km);
  
  G.saveSearchTreeToFile("search_tree1.txt");
  G.savePathToFile("output_path1.txt", startNode);

  
  //this is the D* Lite loop that moves us along the path, checking for obstacles and replanning
  Node* thisNode = startNode; //uncomment for reverse search rooted at goal
  while(startNode->id != goalNode->id) {	//update this to stop when we traverse to goal successfully
      

  		//move the start node to the next position along the solution path
  		thisNode = thisNode->parentNode;
  		Node* startNode = &G.nodes[thisNode->id];
  		Node* goalNode = &G.nodes[goalNodeIndex];
  		startNodeIndex = startNode->id;
  		cout << startNode->id;
    
  		//get user input for obstacle injection
    	int inject;
    	float x_obs, y_obs,r_obs;

    	cout << " inject obstacle?????? 1=yes 0=no ";
    	cin >> inject;

		 //get obstacle data and then update all the shit
    	if (inject==1) { 
    		cout << "Enter obstacle x-coord: ";
    		cin >> x_obs;
    		cout << "Enter obstacle y-coord: ";
    		cin >> y_obs;
    		cout << "Enter obstacle radius: ";
    		cin >> r_obs;

			//update the edge costs to affected nodes
			float h = sqrt((lastNode->x - startNode->x)*(lastNode->x - startNode->x) + (lastNode->y - startNode->y)*(lastNode->y - startNode->y));
			km = km + h;
			lastNode = startNode;
			updateEdgeCostsandNodes(G, H, startNode, goalNode, km, "nodes_blank.txt", "edges.txt", x_obs, y_obs, r_obs); //note: use the other edge file as this one gets modified

			//re-compute shortest path
			computeShortestPath(G,H,startNode,goalNode,startNodeIndex,goalNodeIndex,km);
			replan_counter = replan_counter + 1;

			if (replan_counter == 1) {
			  G.saveSearchTreeToFile("search_tree2.txt");
			  G.savePathToFile("output_path2.txt", startNode);
			}
			if (replan_counter == 2) {
			  G.saveSearchTreeToFile("search_tree3.txt");
			  G.savePathToFile("output_path3.txt", startNode);
			}
			if (replan_counter == 3) {
			  G.saveSearchTreeToFile("search_tree4.txt");
			  G.savePathToFile("output_path4.txt", startNode);
			}
    	}
    }
    
      
  return 0;
}






