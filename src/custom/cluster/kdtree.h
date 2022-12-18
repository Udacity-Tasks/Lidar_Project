/* \author Aaron Brown */
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
	Node* root;

	KdTree()
	: root(NULL)
	{}
	
	void insertHelper(Node *&node, int depth, std::vector<float> point, int id)
	{
		// Tree is empty
		if(node == NULL)
		{
			node = new Node(point, id);
			return;
		}
		
		// calculate current dim
		uint cd = depth % 3;

		if(point[cd] < (node->point[cd]))
		{
			insertHelper(node->left, depth+1, point, id);
		}
		else
		{
			insertHelper(node->right, depth+1, point, id);
		}
		
	}
	

	/*
	~KdTree()
	{
		delete root;
	}
	*/
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(root, 0, point, id);

	}

	
	std::vector<int> searchHelper(std::vector<float> target, Node *&node, int depth, float distanceTol, std::vector<int> &ids)
	{
		int cd = depth % 3;
		
		if(node!=NULL)
		{
			if (node->point[0]<=(target[0]+distanceTol) && node->point[0]>=(target[0]-distanceTol) && 
				node->point[1]<=(target[1]+distanceTol) && node->point[1]>=(target[1]-distanceTol) &&
				node->point[2]<=(target[2]+distanceTol) && node->point[2]>=(target[2]-distanceTol))
			{
				float distance = distanceToTarget(node->point, target);

				if(distance<=distanceTol)
				{
					ids.push_back(node->id);
				}
			}

			if (node->point[cd]<=(target[cd]+distanceTol) && node->point[cd]>=(target[cd]-distanceTol))
			{
				searchHelper(target, node->left, depth+1, distanceTol, ids);
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
			
			else if(node->point[cd] < (target[cd]-distanceTol))
			{
				searchHelper(target, node->right, depth+1, distanceTol, ids); 
			}

			else
			{
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
		}

		return ids;
	}
	

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}

	// Euclidean distance
	float distanceToTarget(std::vector<float> node, std::vector<float> target)
	{
		float distance = 0.0;
		for(size_t i=0; i<node.size(); i++)
		{
			distance += ((node[i]-target[i])*(node[i]-target[i]));
		}
		double distSqrt = sqrt(distance);
		return distSqrt;
	}
	
};




