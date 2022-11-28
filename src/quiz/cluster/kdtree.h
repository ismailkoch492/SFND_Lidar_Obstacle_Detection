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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			// Calculate the current dim
			uint cd = depth % 2;

			if( point[cd] < ((*node)->point[cd]) )
				insertHelper( &((*node)->left), depth+1, point, id );
			else
				insertHelper( &((*node)->right), depth+1, point, id );
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insertHelper(&root, 0, point, id);

	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
	{
		if(node != NULL)
		{
			bool cond[4];
			for(int i = 0; i < 2; i++)
			{
				cond[2 * i] = node->point[i] >= (target[i] - distanceTol);
				cond[2 * i + 1] = node->point[i] <= (target[i] + distanceTol);
			}
			if( (cond[0] && cond[1]) && (cond[2] && cond[3]) )
			{
				float d_xy[] = {node->point[0] - target[0], node->point[1] - target[1]};
				float distance = sqrt( d_xy[0] * d_xy[0] + d_xy[1] * d_xy[1]);
				if (distance <= distanceTol)
					ids.push_back(node->id);
			}
			if( (target[depth%2] - distanceTol) < node->point[depth%2] )
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			if( (target[depth%2] + distanceTol) > node->point[depth%2] )
				searchHelper(target, node->right, depth+1, distanceTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}
	

};




