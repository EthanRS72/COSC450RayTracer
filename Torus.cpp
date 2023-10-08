#include "Torus.h"

#include "utility.h"

#include <cmath>


Torus::Torus(double radius) : Object(), radius_(radius) {

	std::cout << "Radius " << radius_ << std::endl;
}

Torus::Torus(const Torus& torus) : Object(torus), radius_(torus.radius_) {

}

Torus::~Torus() {

}

Torus& Torus::operator=(const Torus& torus) {
	if (this != &torus) {
		Object::operator=(torus);
		radius_ = torus.radius_;
	}
	return *this;
}

std::vector<RayIntersection> Torus::intersect(const Ray& ray) const {
	std::vector<RayIntersection> result;

	/***************************************************
	 * Code for Ray-Torus intersection test goes here. *
	 ***************************************************/
	//following: https://www.sciencedirect.com/science/article/pii/B9780080507545500542
	Ray inverseRay = transform.applyInverse(ray);

	const Point& p = inverseRay.point;//E
	const Direction& d = inverseRay.direction; //D

	double px = inverseRay.point(0);
	double dx = inverseRay.direction(0);
	double py = inverseRay.point(1);
	double dy = inverseRay.direction(1);
	double pz = inverseRay.point(2);
	double dz = inverseRay.direction(2);

	//torus params mention in paper 

	double P = (1.0 * 1.0) / std::pow(radius_,2);
	double a0 = 4 * std::pow(1.0 - radius_, 2);
	double b0 = std::pow(1.0 ,2) - (1.0 * 1.0);
	double a = std::sqrt(a0);
	double b = std::sqrt(b0);
	//normalised params for quartic
	double f = 1.0 - (dy * dy);
	double g = f + (P * dy * dy);
	double l = 2.0 * (px * dx + pz * dz);
	double t = (px * px) + (pz * pz);
	double q = a0 / (g * g);
	double m = (l + 2.0 * P * py * dy) / g;
	double u = (t + P * py * py + b0) / g;

	//Torus equations for quartic
	double c4 = std::pow(dx * dx + P * dy * dy + dz * dz, 2);
	double c3 = 4.0 * (px * dx + P * py * dy + pz * dz) * (dx * dx + P * dy * dy + dz * dz);
	double c2 = 4.0 * std::pow(px * dx + P * py * dy + pz * dz, 2) + 2.0 * (px * px + P * py * py + pz * pz + b0) * 
		(dx * dx + P * dy * dy + dz * dz) - a0 * (dx * dx + dz * dz);
	double c1 = 4.0 * (px * dx + P * py * dy + pz * dz) * (px * px + P * dy * dy + pz * pz + b0) - 2 * a0 *
		(px * dx + pz * dz);
	double c0 = std::pow(px * px + P * py * py + pz * pz + b0, 2) - a0 * (px * px + pz * pz);

	//generalise form which didn't work
	/*double c4 = 1.0;
	double c3 = 2.0 * m;
	double c2 = (m * m) + (2.0 * u) - (q * f);
	double c1 = (2.0 * m * u) - (q * l);
	double c0 = (u * u) - (q * t);*/

	std::vector<double> roots = solveQuartic(c4, c3, c2, c1, c0);

	for (double t : roots) {
		if (t > epsilon) {
			RayIntersection hit;
			
			Point h = Point(p + t * d);
			Normal n = Normal(p + t * d);

			//equations for normal which looked less like a torus
			/*double d_n = std::sqrt((h(0 * h(0)) + (h(1) * h(1))));
			double f_n = (2.0 * (d_n - 1.0)) / (d_n * a * a);
			double nx = h(0) * f_n;
			double ny = (2.0 * h(1)) / (b * b);
			double nz = h(2) * f_n;
			Normal n = Normal(nx, ny, nz)*/

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
