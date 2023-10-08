#include "Scene.h"

#include "Colour.h"
#include "ImageDisplay.h"
#include "utility.h"

// For demos

Scene::Scene() : backgroundColour(0,0,0), ambientLight(0,0,0), maxRayDepth(3), renderWidth(800), renderHeight(600), filename("render.png"), camera_(), objects_(), lights_() {

}

Scene::~Scene() {

}

void Scene::render() const {
	ImageDisplay display("Render", renderWidth, renderHeight);

	const double w = double(renderWidth);
	const double h = double(renderHeight);

	for (unsigned int v = 0; v < renderHeight; ++v) {
		for (unsigned int u = 0; u < renderWidth; ++u) {
			double cu = -1 + (u + 0.5)*(2.0 / w);
			double cv = -h/w + (v + 0.5)*(2.0 / w);
			Ray ray = camera_->castRay(cu, cv);
			display.set(u, v, computeColour(ray, maxRayDepth));
		}
		display.refresh();
	}

	display.save(filename);
	display.pause(5);
}

RayIntersection Scene::intersect(const Ray& ray) const {
	RayIntersection firstHit;
	firstHit.distance = infinity;
	std::vector<RayIntersection> hits;
	
	for (const auto& obj: objects_) {
		for (const auto& hit: obj->intersect(ray)) {
			if (hit.distance > epsilon && hit.distance < firstHit.distance) {
				firstHit = hit;
			}
		}
	}
	return firstHit;
}

Colour Scene::computeColour(const Ray& ray, unsigned int rayDepth) const {
	RayIntersection hitPoint = intersect(ray);
	if (hitPoint.distance == infinity) {
		return backgroundColour;
	}

	Colour hitColour(0, 0, 0);
		
	/******************************************************************
	 * Code for better lighting, shadows, and reflections goes below. *
	 ******************************************************************/

	for (const auto & light : lights_) {
		// Compute the influence of this light on the appearance of the hit object .
		if (light->getDistanceToLight(hitPoint.point) < 0) {
			// An ambient light, ignore shadows and add appropriate colour
			hitColour += light->getIlluminationAt(hitPoint.point) * hitPoint.material.ambientColour;
		} else {
			// Not an ambient light
			// Code for diffuse + specular lighting and shadows goes here
			
			//shadows
			//setup shadow ray starting at the hit point and heading towards the light
			Ray shadowRay;
			shadowRay.point = hitPoint.point;
			shadowRay.direction = light->getLightDirection(shadowRay.point) * -1;

			//cast shadow ray through the scene towards the light and check for intersections
			RayIntersection shadowCheck = intersect(shadowRay);

			//check if intersection occurs between point and light and if not add light
			if (!(shadowCheck.distance < hitPoint.distance)) {

				//compute li
				Vector li = light->getLightDirection(hitPoint.point) * -1;

				//unit vector norm
				Vector nHat = hitPoint.normal / hitPoint.normal.norm(); 

				//unit vector l
				Vector liHat = li / li.norm();

				// compute dot of norm n and norm l
				double diffdotprod = nHat.dot(liHat);

				//compute riHat
				Vector riHat = 2 * (diffdotprod)*nHat - liHat;

				//compute v
				Vector v = ray.point - hitPoint.point;

				//compute vhat
				Vector vHat = v / v.norm();

				//compute dot product of norm ri and norm v
				double specdotprod = riHat.dot(vHat);

				//diffuse -> Id,i * kd * (n.l) 
				//check dot and compute Id,i * kd * (n.l)
				if (diffdotprod > 0) {
					hitColour += light->getIlluminationAt(hitPoint.point) * hitPoint.material.diffuseColour * diffdotprod;
				}

				//specular -> Is,i * ks * (r.v)^a
				//check dot and compute Is,i * ks * (r.v)^a
				if (specdotprod > 0) {
					hitColour += light->getIlluminationAt(hitPoint.point) * hitPoint.material.specularColour * 
						std::pow(specdotprod, hitPoint.material.specularExponent);
				}
			}
		}
	}

	// Code for reflections goes here
	if (rayDepth > 0 && hitPoint.material.mirrorColour.red > 0 ||
		hitPoint.material.mirrorColour.green > 0 || hitPoint.material.mirrorColour.blue > 0) {

		//create new ray
		Ray reflectedRay;

		//recalculate nHat and vHat for the scope
		Vector nHat = hitPoint.normal / hitPoint.normal.norm();
		Vector v = ray.point - hitPoint.point;
		Vector vHat = v / v.norm();

		//set ray point and direction
		reflectedRay.point = hitPoint.point;
		reflectedRay.direction = 2 * (nHat.dot(vHat)) * nHat - vHat;

		//get mirror colour
		Colour mirrorCol = hitPoint.material.mirrorColour;

		//update hit colour with the mirror colour
		hitColour = (Colour(1, 1, 1) - mirrorCol) * hitColour + mirrorCol * computeColour(reflectedRay, rayDepth - 1);
	}

	hitColour.clip();
	return hitColour;
}

bool Scene::hasCamera() const {
	return bool(camera_);
}
