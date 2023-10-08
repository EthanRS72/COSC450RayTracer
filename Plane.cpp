#include "Plane.h"

#include "utility.h"

Plane::Plane() : Object() {

}

Plane::Plane(const Plane& plane) : Object(plane) {

}

Plane::~Plane() {

}

Plane& Plane::operator=(const Plane& plane) {
	if (this != &plane) {
		Object::operator=(plane);
	}
	return *this;
}

std::vector<RayIntersection> Plane::intersect(const Ray& ray) const {

	std::vector<RayIntersection> result;

	Ray inverseRay = transform.applyInverse(ray);

	//point and direction on the z-axis
	double z0 = inverseRay.point(2);
	double dz = inverseRay.direction(2);

	if (!isZero(dz)) {
		// solve for hit point on the z-axis
		double t = -z0 / dz;
		RayIntersection hit;
		hit.point = inverseRay.point + t * inverseRay.direction;
		if (t > 0) {
			//check if the point lies within the valid x,y boundaries
			if (hit.point(0) <= 1 && hit.point(0) >= -1 && hit.point(1) <= 1 && hit.point(1) >= -1) {
				//if so hit point is valid so set and transform values as required
				hit.material = material;
				Normal n = Normal(0, 0, 1);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(n);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}

		
	}
	return result;
}