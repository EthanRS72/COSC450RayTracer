#include "Cylinder.h"

#include "utility.h"

Cylinder::Cylinder() : Object() {

}

Cylinder::Cylinder(const Cylinder& cylinder) : Object(cylinder) {

}

Cylinder::~Cylinder() {

}

Cylinder& Cylinder::operator=(const Cylinder& cylinder) {
	if (this != &cylinder) {
		Object::operator=(cylinder);
	}
	return *this;
}

std::vector<RayIntersection> Cylinder::intersect(const Ray& ray) const {

	std::vector<RayIntersection> result;

	/******************************************************
	 * Code for Ray-Cylinder intersection test goes here. *
	 ******************************************************/
	//following: https://www.cl.cam.ac.uk/teaching/1999/AGraphHCI/SMAG/node2.html#SECTION00023100000000000000
	Ray inverseRay = transform.applyInverse(ray);

	const Point& e = inverseRay.point;//E
	const Direction& d = inverseRay.direction; //D

	double ex = inverseRay.point(0);
	double dx = inverseRay.direction(0);
	double ey = inverseRay.point(1);
	double dy = inverseRay.direction(1);
	double ez = inverseRay.point(2);
	double dz = inverseRay.direction(2);

	double a = (dx * dx) + (dy * dy); //Dx^2 + Dy^2
	double b = (2.0 * ex * dx) + (2.0 * ey * dy); //2 * Ex * Dx + 2 * Ey * Dy
	double c = (ex * ex) + (ey * ey) - 1.0; //Ex^2 + Ey^2 - 1

	std::vector<double> roots = solveQuadratic(a, b, c);

	for (double t : roots) {
		if (t > epsilon) {
			RayIntersection hit;
			Point h = Point(e + t * d);
			Normal n;
			double zhit = ez + t * dz;
			double yhit = ey + t * dy;
			double xhit = ex + t * dx;

			//body of cylinder
			if (zhit > -1.0 && zhit < 1.0) {
				if ((xhit >= -1.0 && xhit <= 1.0) && (yhit >= -1.0 && yhit <= 1.0)) {
					n = Normal(xhit, yhit, 0);

					// Apply the transform
					hit.point = transform.apply(h);
					hit.normal = transform.apply(n);

					// Ensure the normal points towards the ray
					if (hit.normal.dot(ray.direction) > 0) {
						hit.normal = -hit.normal;
					}

					// Other hit information
					hit.distance = (hit.point - ray.point).norm();
					hit.material = material;

					result.push_back(hit);
				}
			}

			//top cap
			if (!isZero(dz)) {
				double t = (1.0 - ez) / dz;
				hit.point = inverseRay.point + t * inverseRay.direction;

				if (t > 0) {
					// Check if the intersection point is within the cylinder's bounds
					double x = hit.point(0);
					double y = hit.point(1);
					// Within the unit circle
					if (x * x + y * y <= 1.0) {

						Normal n = Normal(0, 0, 1);
						hit.point = transform.apply(hit.point);
						hit.normal = transform.apply(n);

						hit.distance = (hit.point - ray.point).norm();
						hit.material = material;
						result.push_back(hit);
					}
				}
			}

			//bottom cap
			if (!isZero(dz)) {
				double t = (-1.0 - ez) / dz;
				hit.point = inverseRay.point + t * inverseRay.direction;

				if (t > 0) {
					// Check if the intersection point is within the cylinder's bounds
					double x = hit.point(0);
					double y = hit.point(1);
					// Within the unit circle
					if (x * x + y * y <= 1.0) {  
						
						Normal n = Normal(0, 0, -1);
						hit.point = transform.apply(hit.point);
						hit.normal = transform.apply(n);

						hit.distance = (hit.point - ray.point).norm();
						hit.material = material;
						result.push_back(hit);
					}
				}
			}
			
		}
	}
	return result;
}

