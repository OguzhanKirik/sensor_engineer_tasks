// Project template by Aaron Brown at Udacity, Inc.
// Assignment completed by Dylan Walker Brown
// See LICENSE file for details

#ifndef KDTREE_H
#define KDTREE_H
#include "render/render.h"
// implementaion for 3d
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree() : root(NULL) {}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint cd = depth % 3;  // Use 3 dimensions
			if (point[cd] < ((*node)->point[cd]))
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);
	}

// 	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int>& ids)
// 	{
// 		if (node != NULL)
// 		{
// 			if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) &&
// 				(node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) &&
// 				(node->point[2] >= (target[2] - distanceTol) && node->point[2] <= (target[2] + distanceTol)))
// 			{
// 				float distance = sqrt(
// 					pow((node->point[0] - target[0]), 2) +
// 					pow((node->point[1] - target[1]), 2) +
// 					pow((node->point[2] - target[2]), 2));

// 				if (distance <= distanceTol)
// 					ids.push_back(node->id);
// 			}

// 			uint cd = depth % 3;
// 			if ((target[cd] - distanceTol) < node->point[cd])
// 		`		searchHelper(target, node->left, depth + 1, distanceTol, ids);
// 			if ((target[cd] + distanceTol) > node->point[cd])
// 				searchHelper(target, node->right, depth + 1, distanceTol, ids);
// 		}
// 	}
void searchHelper(const std::vector<float>& target, Node* node, int depth, float distanceTolSquared, std::vector<int>& ids)
{
	if (node == nullptr) return;

	bool insideBox = true;
	for (int i = 0; i < 3; ++i) {
		if (node->point[i] < (target[i] - sqrt(distanceTolSquared)) || node->point[i] > (target[i] + sqrt(distanceTolSquared))) {
			insideBox = false;
			break;
		}
	}

	if (insideBox) {
		float distSquared = 0.0f;
		for (int i = 0; i < 3; ++i) {
			float diff = node->point[i] - target[i];
			distSquared += diff * diff;
		}
		if (distSquared <= distanceTolSquared)
			ids.push_back(node->id);
	}

	uint cd = depth % 3;
	if (target[cd] - sqrt(distanceTolSquared) < node->point[cd])
		searchHelper(target, node->left, depth + 1, distanceTolSquared, ids);
	if (target[cd] + sqrt(distanceTolSquared) > node->point[cd])
		searchHelper(target, node->right, depth + 1, distanceTolSquared, ids);
}

// 	std::vector<int> search(std::vector<float> target, float distanceTol)
// 	{
// 		std::vector<int> ids;
// 		searchHelper(target, root, 0, distanceTol, ids);
// 		return ids;
// 	}
  
  
  std::vector<int> search(const std::vector<float>& target, float distanceTol)
{
	std::vector<int> ids;
	float distanceTolSquared = distanceTol * distanceTol;
	searchHelper(target, root, 0, distanceTolSquared, ids);
	return ids;
}

};

#endif