#include "Sphere.h"

#include "utility.h"

Sphere::Sphere() : Object() {

}

Sphere::Sphere(const Sphere& sphere) : Object(sphere) {

}

Sphere::~Sphere() {

}

Sphere& Sphere::operator=(const Sphere& sphere) {
	if (this != &sphere) {
		Object::operator=(sphere);
	}
	return *this;
}

std::vector<RayIntersection> Sphere::intersect(const Ray& ray) const {

	std::vector<RayIntersection> result;

	Ray inverseRay = transform.applyInverse(ray);

	// Intersection is of the form at^2 + bt + c, where t = distance along the ray

	const Point& p = inverseRay.point;//D
	const Direction& d = inverseRay.direction; //E
	double a = d.dot(d); //D*D
	double b = 2 * d.dot(p); //2E * D
	double c = p.dot(p) - 1; //E*E-1

	std::vector<double> roots = solveQuadratic(a, b, c);
	for (double t : roots) {
		if (t > epsilon) {
			RayIntersection hit;
			// Hit point on standard sphere
			Point h = Point(p + t * d);
			Normal n = Normal(p + t * d);

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

	return result;
}    