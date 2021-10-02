/*  Michael Otte, University of Colorado, 9-22-08
 *
Copyright 2008 and 2018, Michael Otte

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


 *  basic binary heap implimentation 
 *
 *  NOTE: Modified in 2018 to make into a more general c++ class
 *  
 *  NOTE: the data type of the elements stored in the heap
 *        is assumed to have the following fields initialized as follows:
 *                 bool inHeap = false
 *                 int heapIndex = -1
 *                                    
 *  For sorting speed, the heap is stored in two parallel arrays, one with 
 *  cost, and the other containing the associated node pointers.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <limits>
#include <cstring>

// make things easier (LARGE really just needs to be any number
// >> than any graph cost we might encounter, but this is the best)
double LARGE = std::numeric_limits<double>::infinity();

using namespace std;

template <class T> 
class Heap 
{ 
 //  private: 
   public: 
       double* heapCost;     // cost values
       T** heapNode;         // pointers to the corresponding map nodes
                             // (or whatever is stored in the heap)

       int parentOfLast;     // stores the index of the parent of the last node
       int tempInd;          // used to help swap nodes
       double tempCost;      // used to help swap nodes
       T*  tempNode;         // used to help swap nodes

       int capacity;         // the number of things this heap can store without
                             // being increased in size
//   public: 

       int indexOfLast;      // the index of the last node in the heap array

//   private: 

       // sets up the memory for the heap
       // assumes a reserve size of heapCapacity
       void buildHeap(int heapCapacity);

       // compares a node n with its parent, and switches them if the parent's
       // cost is more than the node's cost. Repeats if a switch happens.
       // returns the index that the node ended up at
       int bubbleUp(int n);

       // compares a node n with its children, and switches them if a child's cost
       // is less than the node's cost. Repeats if a switch happens.
       void bubbleDown(int n);

       // cleans up and deletes the memory used by the heap
       void deleteHeap();

       // increases the heap size by a factor of two
       void increaseHeapSize();

//   public: 


     // default constructor
     Heap()
     {
       printf("building a heap\n");
       buildHeap(100);
     }

     // constructor that should be used most of the time
     Heap(int heapCapacity)
     {
       printf("building a heap with %d elements\n", heapCapacity);
       buildHeap(heapCapacity);
     }


     // destructor
     ~Heap()
     {
       printf("deleting a heap\n");
       deleteHeap();
     }


     // copy constructor
     //     Heap(const Heap &H)




     // add thisNode to the heap
     void addToHeap(T* thisNode, float keyValue);

     // returns a pointer to the node that is on the top of the heap
     T* topHeap();

     float topKey(); //returns lowest key of any node in the queue
     int topKeyIndex(); //returns index of lowest key of any node in the queue

     // removes the top valued node from the heap and returns a pointer to it
     T* popHeap();


     // removes the particular node from the heap
     // (even if that node is internal to the heap)
     // and then rebalances the heap, returns a pointer
     // to the node that has been removed
     T* removeNodeFromHeap(T* thisNode);

     // updates the position of the particular node within the heap
     // to reflect the new key value
     // (even if that node is internal to the heap)
     // and then rebalances the heap
     // NOTE: this will insert the node if it is not in the heap already
     void updateNodeInHeap(T* thisNode, float keyValue);


     // prints the heap values on the command line
     void printHeap();

     // returns 1 if heap is good, 0 if bad, also prints a command line message
     // this should only be used for debugging as it will really slow down the code
     int checkHeap();

};




