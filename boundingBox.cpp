#ifndef AABB_H
#define AABB_H

#include "boundingBox.h"
#include "macros.h"

//-------------------------------------------------------------------- - default constructor
AABB::AABB(void) 
{
	min = Vector(-1.0f, -1.0f, -1.0f);
	max = Vector(1.0f, 1.0f, 1.0f);
}

// --------------------------------------------------------------------- constructor
AABB::AABB(const Vector& v0, const Vector& v1)
{
	min = v0; max = v1;
}

// --------------------------------------------------------------------- copy constructor
AABB::AABB(const AABB& bbox) 
{
	min = bbox.min; max = bbox.max;
}

// --------------------------------------------------------------------- assignment operator
AABB AABB::operator= (const AABB& rhs) {
	if (this == &rhs)
		return (*this);
	min = rhs.min;
	max = rhs.max;
	return (*this);
}

// --------------------------------------------------------------------- destructor
AABB::~AABB() {}

// --------------------------------------------------------------------- inside
// used to test if a ray starts inside a bbox

bool AABB::isInside(const Vector& p) 
{
	return ((p.x > min.x && p.x < max.x) && (p.y > min.y && p.y < max.y) && (p.z > min.z && p.z < max.z));
}

// --------------------------------------------------------------------- compute centroid
Vector AABB::centroid(void) {
	return (min + max) / 2;
}

// --------------------------------------------------------------------- extend AABB
void AABB::extend(AABB box) {
	if (min.x > box.min.x) min.x = box.min.x;
	if (min.y > box.min.y) min.y = box.min.y;
	if (min.z > box.min.z) min.z = box.min.z;

	if (max.x < box.max.x) max.x = box.max.x;
	if (max.y < box.max.y) max.y = box.max.y;
	if (max.z < box.max.z) max.z = box.max.z;
}

// --------------------------------------------------------------------- AABB intersection

bool AABB::intercepts(const Ray& ray, float& t)
{
	double t0, t1;

	float ox = ray.origin.x;
	float oy = ray.origin.y;
	float oz = ray.origin.z;
	float dx = ray.direction.x;
	float dy = ray.direction.y;
	float dz = ray.direction.z;

	float x0 = min.x;
	float y0 = min.y;
	float z0 = min.z;
	float x1 = max.x;
	float y1 = max.y;
	float z1 = max.z;

	float tx_min, ty_min, tz_min;
	float tx_max, ty_max, tz_max;

	float a = 1.0 / dx;
	if (a >= 0) {
		tx_min = (x0 - ox) * a;
		tx_max = (x1 - ox) * a;
	}
	else {
		tx_min = (x1 - ox) * a;
		tx_max = (x0 - ox) * a;
	}

	float b = 1.0 / dy;
	if (b >= 0) {
		ty_min = (y0 - oy) * b;
		ty_max = (y1 - oy) * b;
	}
	else {
		ty_min = (y1 - oy) * b;
		ty_max = (y0 - oy) * b;
	}

	float c = 1.0 / dz;
	if (c >= 0) {
		tz_min = (z0 - oz) * c;
		tz_max = (z1 - oz) * c;
	}
	else {
		tz_min = (z1 - oz) * c;
		tz_max = (z0 - oz) * c;
	}

	float tE, tL; //entering and leaving t values Vector face_in, face_out; // normals
	Vector face_in, face_out, Normal;


	// find largest tE, entering t value
	if (tx_min > ty_min) {
		tE = tx_min;
		face_in = (a >= 0.0) ? Vector(-1, 0, 0) : Vector(1, 0, 0);
	} else {
		tE = ty_min;
		face_in = (b >= 0.0) ? Vector(0, -1, 0) : Vector(0, 1, 0);
	}

	if (tz_min > tE) {
		tE = tz_min;
		face_in = (c >= 0.0) ? Vector(0, 0, -1) : Vector(0, 0, 1);
	}

	// find smallest tL, leaving t value
	if (tx_max < ty_max) {
		tL = tx_max;
		face_out = (a >= 0.0) ? Vector(1, 0, 0) : Vector(-1, 0, 0);
	} else {
		tL = ty_max;
		face_out = (b >= 0.0) ? Vector(0, 1, 0) : Vector(0, -1, 0);
	}

	if (tz_max < tL) {
		tL = tz_max;
		face_out = (c >= 0.0) ? Vector(0, 0, 1) : Vector(0, 0, -1);
	}

	if (tE < tL && tL > 0) { // condition for a hit
		if (tE > 0) {
			t = tE; // ray hits outside surface
			Normal = face_in;
		} else {
			t = tL; // ray hits inside surface
			Normal = face_out;
		}
		return (true);
	}
	else return (false);
}
#endif