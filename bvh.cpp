#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

bool compareX(Object* i1, Object* i2)
{
	return (i1->getCentroid().getAxisValue(0) < i2->getCentroid().getAxisValue(0));
}
bool compareY(Object* i1, Object* i2)
{
	return (i1->getCentroid().getAxisValue(1) < i2->getCentroid().getAxisValue(1));
}
bool compareZ(Object* i1, Object* i2)
{
	return (i1->getCentroid().getAxisValue(2) < i2->getCentroid().getAxisValue(2));
}


void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	   //PUT YOUR CODE HERE

		if (right_index - left_index <= 2) {
			node->makeLeaf(left_index, right_index - left_index);
			return;
		}
		AABB b = node->getAABB();
		Vector min = b.min;
		Vector max = b.max;
		float x0 = min.getAxisValue(0);
		float y0 = min.getAxisValue(1);
		float z0 = min.getAxisValue(2);
		float x1 = max.getAxisValue(0);
		float y1 = max.getAxisValue(1);
		float z1 = max.getAxisValue(2);

		float xRange = x1 - x0;
		float yRange = y1 - y0;
		float zRange = z1 - z0;

		Vector longestRange = Vector();
		if (xRange > yRange) {
			if (xRange > zRange) {
				longestRange = Vector(xRange,0,0);
			}
			else {
				longestRange = Vector(0,0,zRange);
			}
		}
		else {
			if (yRange > zRange) {
				longestRange = Vector(0,yRange,0);
			}
			else {
				longestRange = Vector(0, 0, zRange);
			}
		}
		/*
		if (longestRange.y != 0) {
			sort(objects.at(left_index), objects.at(right_index), compareY);
		}
		if (longestRange.x != 0) {
			sort(objects.at(left_index), objects.at(right_index), compareX);
		}
		if (longestRange.z != 0) {
			sort(objects.at(left_index), objects.at(right_index), compareZ);
		}
		Vector center = b.centroid();
		int split_index;
		if (longestRange.x != 0) {
			split_index = center.x;
		}
		else if (longestRange.y != 0) {
			split_index = center.y;
		}
		else {
			split_index = center.z;
		}
		*/
		int split_index = 0;
		node->makeNode(nodes.size());
		node->makeNode(nodes.size()+1);

		BVHNode* left = new BVHNode();
		
		BVHNode* right = new BVHNode();

		AABB bboxLeft = AABB();
		AABB bboxRight = AABB();
		for (int i = left_index; i < split_index; i++) {
			AABB bbox = objects.at(i)->GetBoundingBox();
			bboxLeft.extend(bbox);

		}
		for (int i = split_index; i < right_index; i++) {
			AABB bbox = objects.at(i)->GetBoundingBox();
			bboxRight.extend(bbox);
		}
		left->setAABB(bboxLeft);
		right->setAABB(bboxRight);
		nodes.push_back(left);
		nodes.push_back(right);

		build_recursive(left_index, split_index, left);
		build_recursive(split_index, right_index, right);


		//right_index, left_index and split_index refer to the indices in the objects vector
	   // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	    // node.index can have a index of objects vector or a index of nodes vector
			
		
	}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
			float tmp;
			float tmin = FLT_MAX;  //contains the closest primitive intersection
			bool hit = false;

			BVHNode* currentNode = nodes[0];

			//PUT YOUR CODE HERE
			
			return(false);
	}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
			float tmp;

			double length = ray.direction.length(); //distance between light and intersection point
			ray.direction.normalize();

			//PUT YOUR CODE HERE

			return(false);
	}		
