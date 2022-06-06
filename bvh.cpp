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



void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	   //PUT YOUR CODE HERE

		if (right_index - left_index <= Threshold) {
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

		Vector minBB = Vector(FLT_MAX, FLT_MAX, FLT_MAX), maxBB = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		float xRange = x1 - x0;
		float yRange = y1 - y0;
		float zRange = z1 - z0;

		int dim;

		
		if (xRange > yRange) {
			if (xRange > zRange) {
				dim = 0;
			}
			else {	
				dim = 2;
			}
		}
		else {
			if (yRange > zRange) {
				dim = 1;
			}
			else {
				dim = 2;
			}
		}
		Comparator a;
		a.dimension = dim;
		sort(objects.begin() + left_index, objects.begin() + right_index, a);


		float middle = (b.max.getAxisValue(dim) + b.min.getAxisValue(dim)) * 0.5;
		
		int split_index;

		//certificar que nenhum ramo fica vazio
		if (objects[left_index]->getCentroid().getAxisValue(dim) > middle || objects[right_index - 1]->getCentroid().getAxisValue(dim) <= middle) {
			middle = 0;
			for (split_index = left_index; split_index < right_index; split_index++) {
				middle += objects[split_index]->getCentroid().getAxisValue(dim);
			}
			middle /= (right_index - left_index);
		}

		if (objects[left_index]->getCentroid().getAxisValue(dim) > middle || objects[right_index - 1]->getCentroid().getAxisValue(dim) <= middle) {
			split_index = left_index + Threshold;
		}
		else {
			for (split_index = left_index; split_index < right_index; split_index++) {
				if (objects[split_index]->getCentroid().getAxisValue(dim) > middle) {
					break;
				}
			}
		}
		
		//int split_index = 0;

		
		BVHNode* left = new BVHNode();
		
		BVHNode* right = new BVHNode();

		AABB bboxLeft = AABB(minBB,maxBB);
		AABB bboxRight = AABB(minBB,maxBB);
		for (int i = left_index; i < split_index; i++) {
			AABB bbox = objects[i]->GetBoundingBox();
			bboxLeft.extend(bbox);

		}
		for (int i = split_index; i < right_index; i++) {
			AABB bbox = objects[i]->GetBoundingBox();
			bboxRight.extend(bbox);
		}
		left->setAABB(bboxLeft);
		right->setAABB(bboxRight);

		node->makeNode(nodes.size()) ;
		//node->makeNode(nodes.size() + 1);
		

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
			Object* closest_obj = NULL;
			float distance;

			if (currentNode->getAABB().intercepts(ray, distance)) {
				while (true) {
					if (!currentNode->isLeaf()) {
						float t1;
						float t2;

						BVHNode* left = nodes[currentNode->getIndex()];
						BVHNode* right = nodes[currentNode->getIndex()+1];

						//faco ja isto assim em vez de estar constantemente a chamar as funcoes nos ifs que nao deve haver problema
						bool leftH = left->getAABB().intercepts(ray, t1);
						bool rightH = right->getAABB().intercepts(ray, t2);
						//acho q isto e necessario pelo q vi no horario de duvidas?
						if (left->getAABB().isInside(ray.origin)) {
							t1 = 0;
						} 
						if (right->getAABB().isInside(ray.origin)) {
							t2 = 0;
						} 

						if (leftH && rightH) {
							if (t1 > t2) {
								StackItem one = StackItem(left, t1);
								hit_stack.push(one);
								currentNode = right;
							}
							else {
								StackItem two = StackItem(right, t2);
								hit_stack.push(two);
								currentNode = left;
							}
							continue;
						}
						else if (leftH) {
							currentNode = left;
							continue;
						}
						else if (rightH) {
							currentNode = right;
							continue;
						}
					}
					else {
						int ind = currentNode->getNObjs();
						int ind2 = currentNode->getIndex();
						float temp;
						
						for (int i = ind2; i < (ind2 + ind); i++) {
							if (objects[i]->intercepts(ray, temp) && temp < tmin) {
								tmin = temp;
								*hit_obj = objects[i];
								hit = true;
							}
						}
					}
					bool newT = false;
					while (!hit_stack.empty()) {
						StackItem head = hit_stack.top();
						hit_stack.pop();

						if (head.t < tmin) {
							currentNode = head.ptr;
							newT = true;
							break;
						}
					}
					//tem que ser assim porque o continue faz com q eu trabalhe apenas no nested while
					if (newT) {
						continue;
					}
					if (hit_stack.empty()) {
						if (hit) {
							hit_point = ray.direction * tmin + ray.origin;
						}
						return hit;
					}

				}
			}
			
			//PUT YOUR CODE HERE
			
			return(false);
	}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
	float tmp;
	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;

	BVHNode* currentNode = nodes[0];
	Object* closest_obj = NULL;
	float distance;

	if (currentNode->getAABB().intercepts(ray, distance)) {
		while (true) {
			if (!currentNode->isLeaf()) {
				float t1;
				float t2;

				BVHNode* left = nodes[currentNode->getIndex()];
				BVHNode* right = nodes[currentNode->getIndex() + 1];

				//faco ja isto assim em vez de estar constantemente a chamar as funcoes nos ifs que nao deve haver problema
				bool leftH = left->getAABB().intercepts(ray, t1);
				bool rightH = right->getAABB().intercepts(ray, t2);
				//acho q isto e necessario pelo q vi no horario de duvidas?
				if (left->getAABB().isInside(ray.origin)) {
					t1 = 0;
				}
				if (right->getAABB().isInside(ray.origin)) {
					t2 = 0;
				}
				if (leftH && rightH) {
					if (t1 > t2) {
						StackItem one = StackItem(left, t1);
						hit_stack.push(one);
						currentNode = right;
					}
					else {
						StackItem two = StackItem(right, t2);
						hit_stack.push(two);
						currentNode = left;
					}
					continue;
				}
				else if (leftH) {
					currentNode = left;
					continue;
				}
				else if (rightH) {
					currentNode = right;
					continue;
				}
			}
			else {
				int ind = currentNode->getNObjs();
				int ind2 = currentNode->getIndex();
				float temp;

				for (int i = ind2; i < (ind2 + ind); i++) {
					if (objects[i]->intercepts(ray, temp) && temp < tmin) {
						return true;
					}
				}
			}
			bool newT = false;
			while (!hit_stack.empty()) {
				StackItem head = hit_stack.top();
				hit_stack.pop();

				if (head.t < tmin) {
					currentNode = head.ptr;
					newT = true;
					break;
				}
			}
			//tem que ser assim porque o continue faz com q eu trabalhe apenas no nested while
			if (newT) {
				continue;
			}
			if (hit_stack.empty()) {
				return false;
			}

		}
	}

	//PUT YOUR CODE HERE

	return(false);
	}		
