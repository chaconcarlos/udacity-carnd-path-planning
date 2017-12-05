#ifndef CARDND_PATHPLANNING_MATH_H_
#define CARDND_PATHPLANNING_MATH_H_

/* CLASS DECLARATION *********************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Converts a value from degrees to radians.
 *
 * @param degrees The degrees.
 *
 * @return The value of the degrees in radians.
 */
double degreesToRadians(const double degrees);

/**
 * @brief Converts a value from radians to degrees.
 *
 * @param radians The radians.
 *
 * @return The value of the radians in degrees.
 */
double radiansToDegrees(const double radians);

/**
 * @brief Calculates the euclidean distance between 2 points.
 *
 * @param x1 The x of the first point.
 * @param y1 The y of the first point.
 * @param x2 The x of the second point.
 * @param y2 The y of the second point.
 *
 * @return The euclidean distance between the given points.
 */
double getDistance(const double x1, const double y1, const double x2, const double y2);

} /* namespace PathPlanning */
} /* namespace CardND */

#endif /* CARDND_PATHPLANNING_MATH_H_ */
