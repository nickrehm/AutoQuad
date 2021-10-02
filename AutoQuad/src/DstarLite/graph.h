/*  Copyright Michael Otte, 2018
 * 
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *  Basic random graph with nodes and edges
 *
 *  Note: that this reads nodes from file with 1-based indexing and 
 *  switches node IDs to use 0-based indexing. In other words,
 *  nodes from a file have their IDs reduced by 1 within this code.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>



struct Node;        // NOTE: we'll define this in detail later, but we need it now


// make the basic edge data stucture 
struct Edge 
{ 
  Node* startNode;     // the edge starts at this node
  Node* endNode;       // the edge goes to this node

  double edgeCost;  // going along the edge cost this much
}; 
 

// this is a basic graph node for a 2D graph
struct Node
{
  int id;     // this is the id of the node
              // it is alos the position in the node array in the
              // graph data structure where this node is stored

  double x;   // physical x location
  double y;   // physical y location


  // NOTE, in an undirect graph outgoingEdges and incommingEdges are the same
  // while in a directed graph they are not

  int numOutgoingEdges;  // the number of outgoing edges

  Edge** outgoingEdges;  // these are edges to {neighbors accessible from this node}
                         // outgoingEdges[j] is a pointer to the j-th one
 
  int numIncommingEdges; // the number of outgoing edges

  Edge** incommingEdges; // these are edges from {nodes which this node can be accessed}
                         // incommingEdges[k] is a pointer the k-th one


  Node* parentNode;      // used for graph search, this is a pointer to this node's parent
                         // node in the search tree

  int status;            // used for grah search, 0 = not visited yet
                         //                       1 = in open list (i.e., in heap)
                         //                       2 = in closed list

  // the following fields are assumed by the heap data structure that I've been using

  bool inHeap;           // init to false and set to true iff the node is in the heap
  int heapIndex;         // enables quick access to the node in the heap

  double costToStart; //added by NJR 2/12/21
  double costToGoal; //added by NJR 4/1/21
  double key; //added NJR 4/5/21
  double g;
  double rhs;

};




// this is the graph data structure
class Graph 
{ 
  public:
    int numNodes;    // the number of nodes in the graph

    Node* nodes;     // an array that contains all of the nodes
                     // nodes[i] contains the node with id i

    int numEdges;    // the number of edges in the graph

    Edge* edges;     // an array that contains all of the edges
                     // edge[j] contains the j-th edge


    // default constructor
    Graph()
    {
      printf("building a default graph\n");
      numNodes = 0;
      nodes = NULL;
      numEdges = 0;
      edges = NULL;
    }

    // default destructor
    ~Graph()
    {
      printf("deleting a graph\n");
      if(nodes != NULL)
      {
        for(int n = 0; n < numNodes; n++)
        {
          if(nodes[n].outgoingEdges != NULL)
          {
            free(nodes[n].outgoingEdges);
          }
          nodes[n].numOutgoingEdges = 0;

          if(nodes[n].incommingEdges != NULL)
          {
            free(nodes[n].incommingEdges);
          }
          nodes[n].numIncommingEdges = 0;
        }
        free(nodes);
      }
      numNodes = 0;
      nodes = NULL;

      if(edges != NULL)
      {
        free(edges);
      }
      numEdges = 0;
      edges = NULL;
    }

    // copy constructor
    //     Graph(const Graph &H)

 
    // loads the graph from the files 
    // (e.g., generated using generate_weighted_random_graph.m)
    // returns true on success
    bool readGraphFromFiles(const char* nodeFilename, const char* edgeFilename)
    {

      FILE * pFile;

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

      // allocate space for the nodes 
      nodes = (Node*)calloc(numNodes, sizeof(Node));

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
        // allocate the node 
        nodes[n].id = id-1;   // NOTE: switches to c++ 0-based indexing!!!
        nodes[n].x = x;
        nodes[n].y = y;

        nodes[n].numOutgoingEdges = 0;  // dummy value for now, reset lower down
        nodes[n].outgoingEdges = NULL;  // dummy value for now, reset lower down
        nodes[n].numIncommingEdges = 0; // dummy value for now, reset lower down
        nodes[n].incommingEdges = NULL; // dummy value for now, reset lower down

        nodes[n].parentNode = NULL;     // used for graph search
        nodes[n].status = 0;            // everything starts not visited

        nodes[n].inHeap = false;        // used by heap
        nodes[n].heapIndex = -1;        // used by heap


        //printf("read node: %d, (%f, %f) from file\n", nodes[n].id, nodes[n].x, nodes[n].y);
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

      // allocate space for the edges 
      edges = (Edge*)calloc(numEdges, sizeof(Edge));

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
        // allocate the edge 
        // NOTE: switches to c++ 0-based indexing!!!
        // also note that edges store pointers to nodes that they are the edge for
        edges[m].startNode = &nodes[startNodeID-1];
        edges[m].endNode = &nodes[endNodeID-1];

        // cost from file
        edges[m].edgeCost = cost;

        // uncomment to use euclidian distance as cost instead
        //double x1 = nodes[startNodeID-1].x;
        //double y1 = nodes[startNodeID-1].y;
        //double x2 = nodes[endNodeID-1].x;
        //double y2 = nodes[endNodeID-1].y;
        //sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));

        //printf("read edge: (%d, %d) from file\n", edges[m].startNode->id, edges[m].endNode->id);
      }

      // close the node file
      fclose(pFile);


      // now we need to go through the nodes and set up the neighbor
      // lists for each node. In this code we are using edges arrays
      // for this part. we will use three passes, on the first
      // pass we figure out how many incomming and outgoing edges
      // each node has, next we allocte all required space,
      // finally, we actually set the edges up


      // we'll also init some temp variables to help us
      // remember how many edges have been set so far in the final pass
      int* outgoingEdgesCount = (int*)calloc(numNodes, sizeof(int));
      int* incommingEdgeCount = (int*)calloc(numNodes, sizeof(int));
      for(int n = 0; n < numNodes; n++)
      {
        outgoingEdgesCount[n] = 0;
        incommingEdgeCount[n] = 0; 
        nodes[n].numOutgoingEdges = 0;   // set this to 0 while we're at it
        nodes[n].numIncommingEdges = 0;  // set this to 0 while we're at it
      }   

      // first pass: go through all edges to see how many incomming 
      // and outgoing edges each node has, and recored these values      
      for(int m = 0; m < numEdges; m++)
      {
        outgoingEdgesCount[edges[m].startNode->id]++;
        incommingEdgeCount[edges[m].endNode->id]++;
      }      

      // second: go through all nodes and allocate space for their
      // neighbor lists, 
      for(int n = 0; n < numNodes; n++)
      {
        nodes[n].outgoingEdges = (Edge**)calloc(outgoingEdgesCount[n], sizeof(Edge*));
        nodes[n].incommingEdges = (Edge**)calloc(incommingEdgeCount[n], sizeof(Edge*));
      }

      // third: go through all the edges again and record pointers to them 
      // in the neighbor lists
      for(int m = 0; m < numEdges; m++)
      {
        // use local pointers to the start and end nodes to make 
        // things things easier to read
        Node* startNodePtr = edges[m].startNode; 
        Node* endNodePtr = edges[m].endNode; 

        //printf("start node: %d\n", startNodePtr->id);
        //printf("end node: %d\n", endNodePtr->id);

        startNodePtr->outgoingEdges[startNodePtr->numOutgoingEdges] = &edges[m];
        endNodePtr->incommingEdges[endNodePtr->numIncommingEdges] = &edges[m];

        startNodePtr->numOutgoingEdges++;
        endNodePtr->numIncommingEdges++;
      } 

      // cleanup
      free(outgoingEdgesCount);
      free(incommingEdgeCount);


      printf("done reading graph from file\n");
      return true;
    }


    // prints out the graph data on the command line
    void printGraph()
    {
      for(int n = 0; n < numNodes; n++)
      {
        printf("node, id: %d, location: (%f, %f)\n", nodes[n].id, nodes[n].x, nodes[n].y);
        printf("  %d outgoing edges:\n", nodes[n].numOutgoingEdges);
        for(int i = 0; i < nodes[n].numOutgoingEdges; i++)
        {
          printf("    from %d (%f, %f) to %d (%f, %f) at cost: %f\n", 
                  nodes[n].outgoingEdges[i]->startNode->id, 
                  nodes[n].outgoingEdges[i]->startNode->x, 
                  nodes[n].outgoingEdges[i]->startNode->y, 
                  nodes[n].outgoingEdges[i]->endNode->id, 
                  nodes[n].outgoingEdges[i]->endNode->x,
                  nodes[n].outgoingEdges[i]->endNode->y,  
                  nodes[n].outgoingEdges[i]->edgeCost);
        }

        printf("  %d incomming edges:\n", nodes[n].numIncommingEdges);
        for(int i = 0; i < nodes[n].numIncommingEdges; i++)
        {
          printf("    from %d (%f, %f) to %d (%f, %f) at cost: %f\n", 
                  nodes[n].incommingEdges[i]->startNode->id, 
                  nodes[n].incommingEdges[i]->startNode->x, 
                  nodes[n].incommingEdges[i]->startNode->y, 
                  nodes[n].incommingEdges[i]->endNode->id, 
                  nodes[n].incommingEdges[i]->endNode->x, 
                  nodes[n].incommingEdges[i]->endNode->y, 
                  nodes[n].incommingEdges[i]->edgeCost);
        }
      }
    }



    // saves the path to a file, extracts it backward from the goal node
    // note that 1 is added to the indicies so that the result is compatible
    // with any 1-indexed input files that may have been used
    // returns true on success
    bool savePathToFile(const char* pathFile, Node* goalNode)
    {
      FILE * pFile = fopen(pathFile,"w");      
      if(pFile == NULL)
      {
        return false;
      }

      Node* thisNode = goalNode;
      while(thisNode != NULL)
      {
        // format is id, x, y    
        // NOTE: incrimenting ids by 1 to convert to 1-based-indexing 
        fprintf(pFile, "%d, %f, %f\n",thisNode->id+1, thisNode->x, thisNode->y);
        thisNode = thisNode->parentNode;
      }

      fclose(pFile);
      printf("saved path in %s\n", pathFile);

      return true;
    } 


    // saves the search tree to a file
    // note that 1 is added to the indicies so that the result is compatible
    // with any 1-indexed input files that may have been used
    // returns true on success
    bool saveSearchTreeToFile(const char* searchTreeFile)
    {
      FILE * pFile = fopen(searchTreeFile,"w");      
      if(pFile == NULL)
      {
        return false;
      }

      for(int n = 0; n < numNodes; n++)
      {
        Node* thisNode = &nodes[n];
        
        if(thisNode->parentNode != NULL)
        {
          //cout << thisNode->parentNode->id;
          //cout << " ";

          // format is id1, x1, y1, id2, x2, y2    
          // NOTE: incrimenting ids by 1 to convert to 1-based-indexing 
          fprintf(pFile, "%d, %f, %f, %d, %f, %f\n",
                  thisNode->id+1, thisNode->x, thisNode->y, 
                  thisNode->parentNode->id+1, thisNode->parentNode->x, thisNode->parentNode->y);
        }
      }
      fclose(pFile);
      printf("saved search tree in %s\n", searchTreeFile);

      return true;
    }


};








