#ifndef CARDND_PATHPLANNING_NAVIGATION_H_
#define CARDND_PATHPLANNING_NAVIGATION_H_

/* INCLUDES ******************************************************************/

#include <stddef.h>
#include <vector>

/* CLASS DECLARATION *********************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Gets the index of the closest waypoint from a given point.
 *
 * @param x          The x value of the origin point.
 * @param y          The y value of the origin point.
 * @param waypointsX The map points x's values.
 * @param waypointsY The map points y's values.
 *
 * @return The index of the closest waypoint.
 */
size_t getClosestWaypoint(
  const double x,
  const double y,
  const std::vector<double>& waypointsX,
  const std::vector<double>& waypointsY);

/**
 * @brief Gets the next waypoint from a given point.
 *
 * @param x          The x value of the origin point.
 * @param y          The y value of the origin point.
 * @param theta      The heading of the origin point.
 * @param waypointsX The map points x's values.
 * @param waypointsY The map points y's values.
 *
 * @return The index of the next waypoint.
 */
size_t
NextWaypoint(
  const double x,
  const double y,
  const double theta,
  const std::vector<double>& waypointsX,
  const std::vector<double>& waypointsY);

/**
 * @brief Transform from Cartesian x,y coordinates to Frenet s,d coordinates.
 *
 * @param x          The x value of the cartesian point.
 * @param y          The y value of the cartesian point.
 * @param theta      The heading of the cartesian point.
 * @param waypointsX The map points x's values.
 * @param waypointsY The map points y's values.
 *
 * @return std::vector with the s and d coordinates.
 */
std::vector<double>
toFrenet(
  const double x,
  const double y,
  const double theta,
  const std::vector<double>& waypointsX,
  const std::vector<double>& waypointsY);

/**
 * @brief Transform from Frenet (s,d) coordinates to Cartesian (x,y).
 *
 * @param s      The distance along the road.
 * @param d      The distance from the center of the lane.
 * @param maps_s The map's s values.
 * @param maps_x The map's x values.
 * @param maps_y The map's y values.
 *
 * @return std::vector with the x and y coordinates.
 */
std::vector<double>
toCartesian(
  const double s,
  const double d,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y);

} /* namespace PathPlanning */
} /* namespace CardND */

#endif /* CARDND_PATHPLANNING_NAVIGATION_H_ */
