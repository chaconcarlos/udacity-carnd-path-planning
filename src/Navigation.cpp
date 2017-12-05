/* INCLUDES ******************************************************************/

#include "Navigation.h"

#include <math.h>

#include "Math.h"

/* DEFINITIONS ***************************************************************/

static constexpr double PI                        = M_PI;
static constexpr double INITIAL_WAYPOINT_DISTANCE = 100000;

/* CLASS IMPLEMENTATION ******************************************************/

namespace CardND
{
namespace PathPlanning
{

size_t
getClosestWaypoint(
  const double x,
  const double y,
  const std::vector<double>& waypointsX,
  const std::vector<double>& waypointsY)
{
  size_t closestWaypoint = 0;
  double closestDistance = INITIAL_WAYPOINT_DISTANCE;

  for(size_t i = 0; i < waypointsX.size(); ++i)
  {
    const double waypointX = waypointsX[i];
    const double waypointY = waypointsY[i];
    const double distance  = getDistance(x, y, waypointX, waypointY);

    if(distance < closestDistance)
    {
      closestDistance = distance;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

size_t
NextWaypoint(
  const double x,
  const double y,
  const double theta,
  const std::vector<double>& waypointsX,
  const std::vector<double>& waypointsY)
{
  size_t       closestWaypoint = getClosestWaypoint(x, y, waypointsX, waypointsY);
  const double waypointX       = waypointsX[closestWaypoint];
  const double waypointY       = waypointsY[closestWaypoint];
  const double heading         = atan2((waypointY - y), (waypointX - x));
  double       angle           = fabs(theta - heading);

  angle = std::min(2 * PI - angle, angle);

  if(angle > PI / 4)
  {
    closestWaypoint++;

    if (closestWaypoint == waypointsX.size())
      closestWaypoint = 0;
  }

  return closestWaypoint;
}

std::vector<double>
toFrenet(
  const double x,
  const double y,
  const double theta,
  const std::vector<double>& waypointsX,
  const std::vector<double>& waypointsY)
{
  const size_t next_wp = NextWaypoint(x, y, theta, waypointsX, waypointsY);
  size_t       prev_wp = next_wp - 1;

  if(next_wp == 0)
    prev_wp  = waypointsX.size() - 1;

  const double n_x = waypointsX[next_wp] - waypointsX[prev_wp];
  const double n_y = waypointsY[next_wp] - waypointsY[prev_wp];
  const double x_x = x - waypointsX[prev_wp];
  const double x_y = y - waypointsY[prev_wp];

  // find the projection of x onto n
  const double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  const double proj_x    = proj_norm * n_x;
  const double proj_y    = proj_norm * n_y;
  double       frenet_d  = getDistance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  const double center_x    = 1000 - waypointsX[prev_wp];
  const double center_y    = 2000 - waypointsY[prev_wp];
  const double centerToPos = getDistance(center_x, center_y, x_x, x_y);
  const double centerToRef = getDistance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef)
    frenet_d *= -1;

  // calculate s value
  double frenet_s = 0;

  for(size_t i = 0; i < prev_wp; ++i)
    frenet_s += getDistance(waypointsX[i], waypointsY[i], waypointsX[i + 1], waypointsY[i + 1]);

  frenet_s += getDistance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

std::vector<double>
toCartesian(
  const double s,
  const double d,
  const std::vector<double>& maps_s,
  const std::vector<double>& maps_x,
  const std::vector<double>& maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp + 1] && prev_wp < (int)(maps_s.size() - 1))
    prev_wp++;

  const int    wp2     = (prev_wp + 1) % maps_x.size();
  const double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));

  // the x,y,s along the segment
  const double seg_s        = s - maps_s[prev_wp];
  const double seg_x        = maps_x[prev_wp] + seg_s * cos(heading);
  const double seg_y        = maps_y[prev_wp] + seg_s * sin(heading);
  const double perp_heading = heading - PI / 2;
  const double x            = seg_x + d * cos(perp_heading);
  const double y            = seg_y + d * sin(perp_heading);

  return {x, y};
}

} /* namespace PathPlanning */
} /* namespace CardND */
