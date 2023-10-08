#pragma once

#ifndef TORUS_H_INCLUDED
#define TORUS_H_INCLUDED

#include "Object.h"

/**
 * \file
 * \brief Torus class header file.
 */


 /**
  * \brief Class for Torus objects.
  *
  * This class provides an Object which is a Torus centred at the origin with outer radius 1
  * and a user-specified inner radius in the range \f$(0,1)\f$, as shown below.
  *
  * \image html torus.png
  * 
  * For a torus as shown in the figure, with a main radius (the centre line of the donut) of 1,
  * and an inner radius (the thickness of the donut) of \f$r\f$,
  * the surface is defined by the quartic (order 4) equation
  * \f$(x^2 + y^2 + z^2 + 1 + r^2)^2 = 4(x^2 + y^2)\f$.
  * 
  * Note that the Torus provided in the skeleton code is not complete and returns no
  * RayIntersection%s from intersect(). This method needs to be implemented correctly
  * as part of the assignment.
  *
  */
class Torus : public Object {

public:

	/** \brief Torus default constructor.
	 *
	 * A newly constructed Torus is centred at the origin, and has a main radius of 1, 
	 * algined to the \f$X-Y\f$ plane. The inner radius is specified in the constructor.
	 * It may be moved, rotated, and scaled through its transform member.
	 * 
	 * \param radius The thickness of the torus.
	 */
	Torus(double radius = 0.5);

	/** \brief Torus copy constructor.
	 * \param torus The Torus to copy.
	 */
	Torus(const Torus& torus);

	/** \brief Torus destructor. */
	~Torus();

	/** \brief Torus assignment operator.
	 *
	 * \param torus The Torus to assign to \c this.
	 * \return A reference to \c this to allow for chaining of assignment.
	 */
	Torus& operator=(const Torus& torus);

	/** \brief Torus-Ray intersection computation.
	 *
	 * \todo The Torus-Ray intersection needs to be implemented as part of the assignment. 
	 *
	 * The intersection of a Ray with an Torus involves solving a fourth-order (quartic) equation.
	 *
	 * \param ray The Ray to intersect with this Torus.
	 * \return A list (std::vector) of intersections, which may be empty.
	 */
	std::vector<RayIntersection> intersect(const Ray& ray) const;

private:

	double radius_; //!< Thickness of the torus

};

#endif // TORUS_H_INCLUDED