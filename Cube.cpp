/* $Rev: 250 $ */
#include "Cube.h"

#include "utility.h"

Cube::Cube() : Object() {

}

Cube::Cube(const Cube& cube) : Object(cube) {

}

Cube::~Cube() {

}

Cube& Cube::operator=(const Cube& cube) {
	if (this != &cube) {
		Object::operator=(cube);
	}
	return *this;
}

std::vector<RayIntersection> Cube::intersect(const Ray& ray) const {

	std::vector<RayIntersection> result;
	
	/**************************************************
	 * Code for Ray-Cube intersection test goes here. *
	 **************************************************/
    /*Define ray information*/
	Ray inverseRay = transform.applyInverse(ray);
	double x0 = inverseRay.point(0);
	double dx = inverseRay.direction(0);
	double y0 = inverseRay.point(1);
	double dy = inverseRay.direction(1);
	double z0 = inverseRay.point(2);
	double dz = inverseRay.direction(2);
	RayIntersection hit;

	//for all sides of the cube just treat them as planes
	/*Check validity of z and compute if valid*/
	if (!isZero(dz)) {
		double t = (1-z0) / dz;
		if (t > 0) {
			hit.point = inverseRay.point + t * inverseRay.direction;
			if (hit.point(0) <= 1 && hit.point(0) >= -1 && hit.point(1) <= 1 && hit.point(1) >= -1) {
				hit.material = material;
				Normal n = Normal(0, 0, 1);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(n);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}


        /*negative z*/
		t = (-1 - z0) / dz;
		if (t > 0) {
			hit.point = inverseRay.point + t * inverseRay.direction;
			if (hit.point(0) <= 1 && hit.point(0) >= -1 && hit.point(1) <= 1 && hit.point(1) >= -1) {
				hit.material = material;
				Normal n = Normal(0, 0, -1);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(n);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
	}


	/*Check validity of y and compute if valid*/
	if (!isZero(dy)) {
		double t = (1 - y0) / dy;
		if (t > 0) {
			hit.point = inverseRay.point + t * inverseRay.direction;
			if (hit.point(0) <= 1 && hit.point(0) >= -1 && hit.point(2) <= 1 && hit.point(2) >= -1) {
				hit.material = material;
				Normal n = Normal(0, 1, 0);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(n);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}


		/*negative y*/
		t = (-1 - y0) / dy;
		if (t > 0) {
			hit.point = inverseRay.point + t * inverseRay.direction;
			if (hit.point(0) <= 1 && hit.point(0) >= -1 && hit.point(2) <= 1 && hit.point(2) >= -1) {
				hit.material = material;
				Normal n = Normal(0, -1, 0);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(n);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
	}

	/*Check validity of x and compute if valid*/
	if (!isZero(dx)) {
		double t = (1 - x0) / dx;
		if (t > 0) {
			hit.point = inverseRay.point + t * inverseRay.direction;
			if (hit.point(1) <= 1 && hit.point(1) >= -1 && hit.point(2) <= 1 && hit.point(2) >= -1) {
				hit.material = material;
				Normal n = Normal(1, 0, 0);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(n);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}


		/*negative x*/
		t = (-1 - x0) / dx;
		if (t > 0) {
			hit.point = inverseRay.point + t * inverseRay.direction;
			if (hit.point(1) <= 1 && hit.point(1) >= -1 && hit.point(2) <= 1 && hit.point(2) >= -1) {
				hit.material = material;
				Normal n = Normal(-1, 0, 0);
				hit.point = transform.apply(hit.point);
				hit.normal = transform.apply(n);
				hit.distance = (hit.point - ray.point).norm();
				result.push_back(hit);
			}
		}
	}

	return result;
}
