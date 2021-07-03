// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
  private:
 	 static constexpr int X{ 0 }, Y{ 1 }, Z{ 2 };


   void recursiveInsert(Node **node, const std::vector<float> point, const int id, const int coord)
   {
        if (*node == NULL)
        {
            *node = new Node(point, id);
            return;
        }
         
        if ( point.at(coord) < (*node)->point.at(coord) )
        {
            recursiveInsert(&((*node)->left), point, id, ((coord + 1) % 3));
        }
        else
        {
            recursiveInsert(&((*node)->right), point, id, ((coord + 1) % 3));
        }
   }
  
  void recursiveSearch(Node *node, const std::vector<float> &target, std::vector<int> &ids,const float tolerance, const int coord)
  {
        if (node == nullptr) 
        { 
          return; 
        }

    	// define withinBox
        float boundaries_xl = (target.at(X) - tolerance);
        float boundaries_xr = (target.at(X) + tolerance);
        float boundaries_yh = (target.at(Y) + tolerance);
        float boundaries_yl = (target.at(Y) - tolerance); 
        float boundaries_z1 = (target.at(Z) - tolerance); 
        float boundaries_z2 = (target.at(Z) + tolerance);
 
      // for 3d
 	 if ((node->point.at(X) >= boundaries_xl && node->point.at(X) <= boundaries_xr) && (node->point.at(Y)>= boundaries_yl && node->point.at(Y) <= boundaries_yh) && (node->point.at(Z) >= boundaries_z1 && node->point.at(Z) <= boundaries_z2))
      { 
              // for 3d
			float distance = sqrt((node->point.at(X) - target.at(X))*(node->point.at(X)-target.at(X)) + (node->point.at(Y)-target.at(Y))*(node->point.at(Y)-target.at(Y)) + (node->point.at(Z)-target.at(Z)) * (node->point.at(Z)-target.at(Z)));
       
          if(distance <= tolerance)
          {
            ids.push_back(node->id);
          }
       }

        if ((target.at(coord) - tolerance) < node->point.at(coord))
            recursiveSearch(node->left, target, ids, tolerance, ((coord + 1) % 3));

        if ((target.at(coord) + tolerance) > node->point.at(coord))
            recursiveSearch(node->right, target, ids, tolerance, ((coord + 1) % 3));
    }
  
  
  public:
	Node* root;

	KdTree()
	: root(NULL)
	{}
  
  
  	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
      	 recursiveInsert(&root, point, id, X);
		// the function should create a new node and place correctly with in the root 
	}
  
  
  	// return a list of point ids in the tree that are within distance of target
    std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		//Node *node{ root };
        std::vector<int> ids;
        recursiveSearch(root, target, ids, distanceTol, X);
        
        return ids;
	}
};





