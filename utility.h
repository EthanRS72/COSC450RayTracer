#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

/** \file
 * \brief General utility functions
 */

const double epsilon = 1e-12; //!< Small number to check for near zero
const double infinity = std::numeric_limits<double>::max(); //!< Very large number, bigger than any sensible distance
const double pi = 3.141592653589793; //!< \f$\pi\f$, obviously. Note that M_PI is NOT part of standard C/C++, so is not portable

/**
 * \brief Check if a value is (near) zero
 * 
 * With floating poitn values computations rarely return exact zeros, but numbers
 * small enough to be 'practically zero' can cause issues in computation. This
 * function checks if the input is within some small tolerance of zero.
 * 
 * \param x The value to check
 * \return true if x is within a small tolerance of zero
 */
inline bool isZero(double x) {
	return (x > -epsilon) && (x < epsilon);
}

/**
 * \brief Convert degrees to radians.
 *
 * C++ mathematical functions expect angles in radians, but many people think
 * in degrees. The high level Transform functions expect parameters in degrees
 * and this function, along with deg2rad(), converts between the two.
 *
 * \param deg An angle measured in degrees.
 * \return The angle measured in radians.
 */
inline double deg2rad(double deg) {
	return deg * pi / 180;
}

/**
 * \brief Convert radians to degrees.
 *
 * C++ mathematical functions expect angles in radians, but many people think
 * in degrees. The high level Transform functions expect parameters in degrees
 * and this function, along with rad2deg(), converts between the two.
 *
 * \param rad An angle measured in radians.
 * \return The angle measured in degrees.
 */
inline double rad2deg(double rad) {
	return rad * 180 / pi;
}

/**
 * \brief Return the sign of a number.
 *
 * This function returns +1 for positive numbers, -1 for negative numbers
 * and 0 for zero values. Since rounding can cause non-zero values, very small
 * numbers (less than \c epsilon) are returned as being 0.
 *
 * \param val The value to check the sign of.
 * \return 0, +1, or -1 depending on the sign of \c val.
 */
inline int sign(double val) {
	if (std::abs(val) < epsilon) return 0;
	if (val < 0) return -1;
	return 1;
}

/**
 * \brief Convert a string to upper case.
 *
 * This function returns a copy of the input string with all lower case
 * characters converted to upper case.
 *
 * \param str The string to convert.
 * \return The string converted to upper case.
 **/
inline std::string toUpper(const std::string& str) {
	std::string tmp = str;
	std::transform(tmp.begin(), tmp.end(), tmp.begin(), toupper);
	return tmp;
}


/**
 * \brief Solve a linear equation \f$ax + b = 0\f$
 * 
 * While the solution is generally \f$\-b/a\f$, this handles cases where \f$a \approx 0\f$,
 * returning no solutions in that case.
 * 
 * \param a The coefficient of \f$x\f$
 * \param b The constant term
 * \return A vector of zero or one solutions
 */
inline std::vector<double> solveLinear(double a, double b) {
	std::vector<double> result;
	if (std::abs(a) > epsilon) {
		result.push_back(-b / a);
	}
	return result;
}

/**
 * \brief Solve a quadratic equation \f$ax^2 + bx + c = 0\f$
 *
 * While the solution is given by the quadratic formula, this accounts for the fact that 
 * there may be zero, one, or two solutions, and for degenerate cases. Only real-valued 
 * solutions are returned.
 * 
 * \param a The coefficient of \f$x^2\f$
 * \param b The coefficient of \f$x\f$
 * \param c The constant term
 * \return A vector of up to two solutions
 */
std::vector<double> solveQuadratic(double a, double b, double c);

/**
 * \brief Solve a cubic equation \f$ax^3 + bx^2 + cx +d = 0\f$
 *
 * Analytic solver for cubic equations.Only real-valued solutions are returned.
 *
 * \param a The coefficient of \f$x^3\f$
 * \param b The coefficient of \f$x^2\f$
 * \param c The coefficient of \f$x\f$
 * \param d The constant term
 * \return A vector of up to three solutions
 */
std::vector<double> solveCubic(double a, double b, double c, double d);

/**
 * \brief Solve a quartic equation \f$ax^4 + bx^32 + cx^2 + dx + e = 0\f$
 *
 * Analytic solver for quartic equations.Only real-valued solutions are returned.
 *
 * \param a The coefficient of \f$x^4\f$
 * \param b The coefficient of \f$x^3\f$
 * \param c The coefficient of \f$x^2\f$
 * \param d The coefficient of \f$x\f$
 * \param e The constant term
 * \return A vector of up to four solutions
 */std::vector<double> solveQuartic(double a, double b, double c, double d, double e);
