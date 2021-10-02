/*  Michael Otte, University of Colorado, 9-22-08
 *
Copyright 2008 and 2018, Michael Otte

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *
 *
 *  basic binary heap implimentation
 *
 *  NOTE: Modified in 2018 to make into a more general c++ class
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <limits>
#include <cstring>

using namespace std;

// sets up the memory for the heap
template <class T> 
void Heap<T>::buildHeap(int heapCapacity)
{

  heapCost = (double*)calloc(heapCapacity, sizeof(double));
  heapNode = (T**)calloc(heapCapacity, sizeof(T*));
  indexOfLast = -1;
  parentOfLast = -1;
  tempCost = LARGE;
  tempNode = NULL;

  capacity = heapCapacity;
  int i;
  for(i = 0; i < heapCapacity; i++)
    heapCost[i] = LARGE;
}






// compares a node n with its parent, and switches them if the parent's
// cost is more than the node's cost. Repeats if a switch happens.
// returns the index that the node ended up at
template <class T>  
int Heap<T>::bubbleUp(int n)
{
  tempInd = (n-1)/2;
  while(n != 0 & heapCost[tempInd] > heapCost[n])
  {
     // swap costs 
     tempCost = heapCost[tempInd]; 
     heapCost[tempInd] = heapCost[n];
     heapCost[n] = tempCost;
     
     // swap graph node pointers
     tempNode = heapNode[tempInd];
     heapNode[tempInd] = heapNode[n];
     heapNode[n] = tempNode;
     
     // update graph node heap index values
     heapNode[tempInd]->heapIndex = tempInd;
     heapNode[n]->heapIndex = n;
     
     // get new node and parent indicies
     n = tempInd;
     tempInd = (n-1)/2;
  } 
  return n;  
}

// compares a node n with its children, and switches them if a child's cost
// is less than the node's cost. Repeats if a switch happens.
template <class T>  
void Heap<T>::bubbleDown(int n)
{
  // find child with smallest value
  if(heapCost[2*n+1] < heapCost[2*n+2])
    tempInd = 2*n+1;
  else
    tempInd = 2*n+2; 
   
  while(n <= parentOfLast & heapCost[tempInd] < heapCost[n])
  {
     // swap costs 
     tempCost = heapCost[tempInd]; 
     heapCost[tempInd] = heapCost[n];
     heapCost[n] = tempCost;
     
     // swap graph node pointers
     tempNode = heapNode[tempInd];
     heapNode[tempInd] = heapNode[n];
     heapNode[n] = tempNode;  
      
     // update graph node heap index values
     heapNode[tempInd]->heapIndex = tempInd;
     heapNode[n]->heapIndex = n;
     
     // get new node and child indicies
     n = tempInd;
     if(heapCost[2*n+1] < heapCost[2*n+2])
       tempInd = 2*n+1;
     else
       tempInd = 2*n+2;   
  }   
}

// add thisNode to the heap, with the key value
template <class T> 
void Heap<T>::addToHeap(T* thisNode, float keyValue)
{
  if(indexOfLast == capacity-1)
  {
    increaseHeapSize();
  }

  if(thisNode->inHeap == false)
  {
    indexOfLast++;
    parentOfLast = (indexOfLast-1)/2;
    heapNode[indexOfLast] = thisNode;
    heapCost[indexOfLast] = keyValue;
    thisNode->heapIndex = indexOfLast;
    bubbleUp(indexOfLast);     
    thisNode->inHeap = true;
  }
}

// returns a pointer to the node that is on the top of the heap
template <class T> 
T* Heap<T>::topHeap()
{
  if(indexOfLast >= 0)
  {
    return heapNode[0];
  }
  return NULL;
}


template <class T> 
float Heap<T>::topKey() //returns smallest key in heap
{
  double min_cost = LARGE;
  /*
  if (indexOfLast == NULL) { //return infinity if heap is empty
    return LARGE;
  }
  */
  for(int i = 0; i <= indexOfLast; i++) {
    if (heapCost[i] < min_cost) {
      min_cost = heapCost[i];
    }
  }

  return min_cost;
}

template <class T> 
int Heap<T>::topKeyIndex() //returns index of node with smallest key in heap
{
  double min_cost = LARGE;
  int idx_mincost;
  /*
  if (indexOfLast == NULL) { //return infinity if heap is empty
    return LARGE;
  }
  */
  for(int i = 0; i <= indexOfLast; i++) {
    if (heapCost[i] < min_cost) {
      min_cost = heapCost[i];
      idx_mincost = heapNode[i]->id;
    }
  }

  return idx_mincost;
}




// removes the top valued node from the heap and returns a pointer to it
template <class T>  
T* Heap<T>::popHeap()
{
  T* oldTopNode = heapNode[0];
  heapNode[0] = heapNode[indexOfLast];
  heapCost[0] = heapCost[indexOfLast];
  heapNode[0]->heapIndex = 0;
  heapNode[indexOfLast] = NULL;
  heapCost[indexOfLast] = LARGE;
  indexOfLast--;
  parentOfLast = (indexOfLast-1)/2;
  bubbleDown(0);
  oldTopNode->inHeap = false;
  oldTopNode->heapIndex = -1;
  return oldTopNode;
}


// removes the particular node from the heap
// (even if that node is internal to the heap)
// and then rebalances the heap, returns a pointer
// to the node that has been removed
template <class T>  
T* Heap<T>::removeNodeFromHeap(T* thisNode)
{
  if(!thisNode->inHeap)
  {
    return NULL;
  }

  int ind = thisNode->heapIndex;

  heapNode[ind] = heapNode[indexOfLast];
  heapCost[ind] = heapCost[indexOfLast];
  heapNode[ind]->heapIndex = ind;

  heapNode[indexOfLast] = NULL;
  heapCost[indexOfLast] = LARGE;
  indexOfLast--;
  parentOfLast = (indexOfLast-1)/2;

  ind = bubbleUp(ind);
  bubbleDown(ind);

  thisNode->inHeap = false;
  thisNode->heapIndex = -1;
  return thisNode;
}


// updates the position of the particular node within the heap
// to reflect the new key value
// (even if that node is internal to the heap)
// and then rebalances the heap)
// NOTE: this will insert the node if it is not in the heap already
template <class T>  
void Heap<T>::updateNodeInHeap(T* thisNode, float keyValue)
{
  // we'll just do the easy way
  if(thisNode->inHeap)
  {
    removeNodeFromHeap(thisNode);
  }
  addToHeap(thisNode, keyValue);


}



// prints the heap values on the command line
// note this will break if T does not have field "id"
template <class T> 
void Heap<T>::printHeap()
{
  printf("heap costs:\n");

  int i,p; 
  char ch[10];
  for(i = 0, p = 0; i <= indexOfLast; i++)
  {    
    printf("%f ", heapCost[i]);
    if(i == p)
    {
       printf("\n");
       p = p+2^p;
    }
  }
  printf("\n\n");
  

  printf("heap node ids:\n");
  for(i = 0, p = 0; i <= indexOfLast; i++)
  {    
    printf("(%d) ", heapNode[i]->id);
    if(i == p)
    {
       printf("\n");
       p = p+2^p;
    }
  }
  printf("\n\n");
}


// returns 1 if heap is good, 0 if bad, also prints a command line message
template <class T> 
int Heap<T>::checkHeap()
{
  int i;
  for(i = 0; i <= indexOfLast; i++)
  {
    if(heapCost[i] < heapCost[(i-1)/2] || heapNode[i]->heapIndex != i)
    {
      printf("There is a problem with the heap \n");
      getchar();
      return false;
    }
  } 
  printf("The heap is OK \n");
  return true;  
}

// cleans up and deletes the memory used by the heap
// note this is called automatically be the destructor
template <class T> 
void Heap<T>::deleteHeap()
{
  free(heapCost);
  heapCost = NULL;
  free(heapNode);
  heapNode = NULL;
  indexOfLast = -1;
  parentOfLast = -1;
  tempInd = -1;
  tempNode = NULL;
  tempCost = LARGE;
}


// increases the heap size by a factor of two
// this should only be used for debugging as it will really slow down the code
template <class T> 
void Heap<T>::increaseHeapSize()
{

  //printf("growing heap\n");
  int oldCapacity = capacity;
  capacity = capacity * 2;
  if(capacity < 100)
  {
    capacity = 100;
  }
  
  double* newHeapCost = (double*)calloc(capacity, sizeof(double));
  T** newHeapNode = (T**)calloc(capacity, sizeof(T*)); 

  std::memcpy(newHeapCost, heapCost, oldCapacity*sizeof(double));
  std::memcpy(newHeapNode, heapNode, oldCapacity*sizeof(T*));

  free(heapCost);
  free(heapNode);

  heapCost = newHeapCost;
  heapNode = newHeapNode;

  //printf("heap can now hold up to %d items\n", capacity);
}

