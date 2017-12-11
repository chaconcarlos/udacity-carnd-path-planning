/* INCLUDES ******************************************************************/

#include "TrajectoryPlanner.h"

#include <stddef.h>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <vector>

#include "Math.h"
#include "Navigation.h"
#include "spline.h"

/* DEFINITIONS ***************************************************************/

static const bool   ACTIVATE_LOGS                = false;
static const int    DEFAULT_TRAJECTORY_POINTS    = 50;
static const int    DEFAULT_REFERENCE_LANE       = 1;
static const double DEFAULT_POINTS_INTERVAL      = 0.02;
static const double MAX_SPEED_OFFSET             = 0.5;
static const double COLLISION_DISTANCE           = 30.0;
static const double PASSING_COLLISION_DISTANCE   = 10.0;
static const double TRAJECTORY_DISTANCE          = 30.0;
static const double DEFAULT_ACCELERATION         = .224;
static const double COST_COLLISION               = 10000.00;
static const double COST_CHANGE_LANES            = 1000.00;
static const double COST_COLLISION_DISTANCE      = 1000.00;
static const double MILES_CONVERSION_FACTOR      = 2.24;

/* STRUCT DECLARATIONS **************************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Describes a car motion.
 */
struct Kinematics
{
  double score        = std::numeric_limits<double>::max();
  double speed        = 0;
  double acceleration = 0;
  int    lane         = 0;
};

} /* namespace PathPlanning */
} /* namespace CardND */

/* STATIC DECLARATIONS *******************************************************/

/**
 * @brief Log stream. Should be flushed after every execution of the planner.
 */
static std::stringstream g_logStream;

/* STATIC FUNCTIONS **********************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Gets the elapsed time given a start timepoint.
 *
 * @param start The start timepoint.
 *
 * @return A string with the time elapsed.
 */
static std::string
getElapsedTime(const std::chrono::time_point<std::chrono::steady_clock>& start)
{
  auto              end = std::chrono::steady_clock::now();
  std::stringstream stream;

  stream << std::chrono::duration_cast<std::chrono::hours>(end - start).count()        << ":";
  stream << std::chrono::duration_cast<std::chrono::minutes>(end - start).count()      << ":";
  stream << std::chrono::duration_cast<std::chrono::seconds>(end - start).count()      << ":";
  stream << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;

  return stream.str();
}

/**
 * @brief Returns a map of detected vehicles sorted by the lane the are driving on.
 *
 * @param road             The road information.
 * @param detectedVehicles The vehicles.
 *
 * @return A map of detected vehicles sorted by the lane the are driving on.
 */
static std::map< int, std::vector<Vehicle> >
sortByLane(const Road& road, const std::map<size_t, Vehicle>& detectedVehicles)
{
  std::map< int, std::vector<Vehicle> > result;

  std::map<size_t, Vehicle>::const_iterator vehicle = detectedVehicles.begin();

  for (; vehicle != detectedVehicles.end(); ++vehicle)
  {
    for (int i = 0; i < road.getLaneCount(); ++i)
    {
      const double laneD          = 2 + 4 * i;
      const double laneLowerLimit = laneD - 2;
      const double laneUpperLimit = laneD + 2;
      const double vehicleD       = vehicle->second.getD();

      if (vehicleD > laneLowerLimit && vehicleD < laneUpperLimit)
        result[i].push_back(vehicle->second);
    }
  }

  return result;
}

static Kinematics
getLaneKinematics(
  const int lane,
  const int currentLane,
  const double maxSpeed,
  const double egoVehicleS,
  const double pathSize,
  const std::vector<Vehicle>& vehiclesInlane)
{
  const bool isSameLane = lane == currentLane;
  Kinematics currentKinematics;

  g_logStream << "  Checking lane " << lane << "..." << std::endl;

  currentKinematics.score        = 0;
  currentKinematics.speed        = maxSpeed;
  currentKinematics.acceleration = DEFAULT_ACCELERATION;
  currentKinematics.lane         = lane;

  if (!isSameLane)
   currentKinematics.score += COST_CHANGE_LANES;

  double minDistance = std::numeric_limits<double>::max();

  std::vector<Vehicle>::const_iterator vehicle = vehiclesInlane.begin();

  for (; vehicle != vehiclesInlane.end(); ++vehicle)
  {
   g_logStream << "    Analyzing vehicle on s: " << vehicle->getS() << ", speed:" << vehicle->getSpeed() << " m/s " <<std::endl;

   const double finalPathDistance     = static_cast<double>(pathSize) * DEFAULT_POINTS_INTERVAL * vehicle->getSpeed();
   const double s                     = vehicle->getS() + finalPathDistance;
   const bool   isAhead               = s > egoVehicleS;
   double       collisionDistance     = COLLISION_DISTANCE;
   double       distanceToEgo         = abs(s - egoVehicleS);
   double       collisionCost         = COST_COLLISION;
   double       collisionDistanceCost = COST_COLLISION_DISTANCE;
   bool         isOnCollision         = false;

   if (distanceToEgo == 0)
     distanceToEgo = std::numeric_limits<double>::epsilon();

   if (!isAhead && isSameLane)
   {
     collisionCost         = 0;
     collisionDistanceCost = 0;
   }
   else if (!isAhead)
   {
     collisionDistance = PASSING_COLLISION_DISTANCE;
   }
   else if (distanceToEgo < minDistance)
   {
     minDistance = distanceToEgo;
   }

   if (distanceToEgo < collisionDistance)
   {
     isOnCollision            = true;
     currentKinematics.score += collisionCost;
     currentKinematics.score += collisionDistance / distanceToEgo * collisionDistanceCost;
   }

   g_logStream << "    Distance to ego: " << distanceToEgo << ", is ahead? " << isAhead << " is on Collision? " << isOnCollision << std::endl;

   if (isAhead && isOnCollision)
   {
     currentKinematics.speed        = vehicle->getSpeed() * MILES_CONVERSION_FACTOR;
     currentKinematics.acceleration = DEFAULT_ACCELERATION * -1;
   }
  }

  currentKinematics.score += (1.0 / minDistance) * 30.0;

  g_logStream << "    Minimum vehicle distance: " << minDistance             << std::endl;
  g_logStream << "    Final lane score: "         << currentKinematics.score << std::endl;

  return currentKinematics;
}

/**
 * @brief Gets the best motion parameters for a vehicle, given its location, surrounding vehicles, road, and location.
 *
 * @param egoVehicleS      The s coordinate of the vehicle.
 * @param pathSize         The path size the car will follow.
 * @param currentLane      The current lane the vehicle is driving on.
 * @param road             The road information.
 * @param detectedVehicles The surrounding vehicles data.
 *
 * @return The best motion parameters for a vehicle, given its location, surrounding vehicles, road, and location.
 */
static Kinematics
getBestKinematics(
  const double egoVehicleS,
  const int pathSize,
  const int currentLane,
  const Road& road,
  const std::map< int, std::vector<Vehicle> >& detectedVehicles)
{
  Kinematics       bestKinematics;
  std::vector<int> possibleLanes;
  const double     maxSpeed = road.getMaxSpeed() - MAX_SPEED_OFFSET;

  possibleLanes.push_back(currentLane - 1);
  possibleLanes.push_back(currentLane);
  possibleLanes.push_back(currentLane + 1);

  for (int i = 0; i < possibleLanes.size(); ++i)
  {
    int        lane        = possibleLanes[i];
    const bool isValidLane = lane >= 0 && lane <= road.getLaneCount() - 1;
    Kinematics currentKinematics;

    if (!isValidLane)
      continue;

    const std::map< int, std::vector<Vehicle> >::const_iterator vehicles = detectedVehicles.find(lane);

    if (detectedVehicles.find(lane) != detectedVehicles.end())
      currentKinematics = getLaneKinematics(lane, currentLane, maxSpeed, egoVehicleS, pathSize, vehicles->second);
    else
      currentKinematics = getLaneKinematics(lane, currentLane, maxSpeed, egoVehicleS, pathSize, std::vector<Vehicle>());

    if (currentKinematics.score < bestKinematics.score)
      bestKinematics = currentKinematics;
  }

  g_logStream << "  Best Kinematics - score: " << bestKinematics.score << ", lane: " << bestKinematics.lane << ", acceleration: " << bestKinematics.acceleration << std::endl;

  return bestKinematics;
}

} /* namespace PathPlanning */
} /* namespace CardND */

/* CLASS IMPLEMENTATION ******************************************************/

namespace CardND
{
namespace PathPlanning
{

TrajectoryPlanner::TrajectoryPlanner(const Road& road)
: m_maxTrajectoryPoints(DEFAULT_TRAJECTORY_POINTS)
, m_targetLane(DEFAULT_REFERENCE_LANE)
, m_targetSpeed(0)
, m_pointsInterval(DEFAULT_POINTS_INTERVAL)
, m_referenceSpeed(0)
, m_road(road)
{
  m_referenceSpeed = m_road.getMaxSpeed() - MAX_SPEED_OFFSET;
}

TrajectoryPlanner::~TrajectoryPlanner()
{
}

void
TrajectoryPlanner::setMaxTrajectoryPoints(const int max)
{
  m_maxTrajectoryPoints = max;
}

void
TrajectoryPlanner::setPointsInterval(const double interval)
{
  m_pointsInterval = interval;
}

Trajectory
TrajectoryPlanner::generateTrajectory(const Vehicle& vehicle)
{
  g_logStream.str(std::string());

  static int                                                analisysCount = 0;
  static std::chrono::time_point<std::chrono::steady_clock> startClock    = std::chrono::steady_clock::now();

  g_logStream << getElapsedTime(startClock) << " - " << analisysCount << " - Starting analysis..." << std::endl;

  analisysCount++;

  const std::vector<double> previousPathXs    = vehicle.getPreviousPathXs();
  const std::vector<double> previousPathYs    = vehicle.getPreviousPathYs();
  const int                 previousPathSize  = previousPathXs.size();
  double                    currentcarS       = vehicle.getS();
  bool                      possibleCollision = false;
  Trajectory                trajectory;

  g_logStream << "  Ego vehicle is at s: " << currentcarS << " in lane: " << m_targetLane << " at simulator speed: " << vehicle.getSpeed() << " my speed: " << m_targetSpeed << std::endl;
  g_logStream << "  Points left from previous trajectory: " << previousPathSize << std::endl;

  std::map< int, std::vector<Vehicle> > detectedVehiclesByLane = sortByLane(m_road, vehicle.getDetectedVehicles());

  if (previousPathSize > 0)
    currentcarS = vehicle.getFinalS();

  const Kinematics kinematics = getBestKinematics(currentcarS, previousPathSize, m_targetLane, m_road, detectedVehiclesByLane);

  if (m_targetLane != kinematics.lane)
    g_logStream << "  ======= CHANGING LANES ======" << std::endl;

  m_targetLane = kinematics.lane;

  // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
  // Later we will interpolate these waypoints with a spline
  std::vector<double> ptsx;
  std::vector<double> ptsy;

  // reference x, y, yaw states
  // either we will reference the starting point as where the car is or at the previous path's end point.
  double refX   = vehicle.getX();
  double refY   = vehicle.getY();
  double refYaw = degreesToRadians(vehicle.getYaw());

  // If previous size is almost empty, use the car as starting reference.
  if (previousPathSize < 2)
  {
    // Use two points that make the path tangent to the car.
    const double prev_car_x = vehicle.getX() - cos(vehicle.getYaw());
    const double prev_car_y = vehicle.getY() - sin(vehicle.getYaw());

    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);

    ptsx.push_back(vehicle.getX());
    ptsy.push_back(vehicle.getY());
  }
  else
  {
   // Use the previous path's end point as starting reference.
    const double ref_x_prev = previousPathXs[previousPathSize - 2];
    const double ref_y_prev = previousPathYs[previousPathSize - 2];

    refX   = previousPathXs[previousPathSize - 1];
    refY   = previousPathYs[previousPathSize - 1];
    refYaw = atan2(refY - ref_y_prev, refX - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsy.push_back(ref_y_prev);
    ptsx.push_back(refX);
    ptsy.push_back(refY);
  }

  const std::vector<double> mapXs = m_road.getWaypointsX();
  const std::vector<double> mapYs = m_road.getWaypointsY();
  const std::vector<double> mapS  = m_road.getWaypointsS();

  const double laneD = 2 + 4 * m_targetLane;

  std::vector<double> next_wp0 = toCartesian(currentcarS + 30, laneD, mapS, mapXs, mapYs);
  std::vector<double> next_wp1 = toCartesian(currentcarS + 60, laneD, mapS, mapXs, mapYs);
  std::vector<double> next_wp2 = toCartesian(currentcarS + 90, laneD, mapS, mapXs, mapYs);

  ptsx.push_back(next_wp0[0]);
  ptsy.push_back(next_wp0[1]);
  ptsx.push_back(next_wp1[0]);
  ptsy.push_back(next_wp1[1]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); ++i)
  {
    double shift_x = ptsx[i] - refX;
    double shift_y = ptsy[i] - refY;

    ptsx[i] = shift_x * cos(0 - refYaw) - shift_y * sin(0 - refYaw);
    ptsy[i] = shift_x * sin(0 - refYaw) + shift_y * cos(0 - refYaw);
  }

  tk::spline spline;

  spline.set_points(ptsx, ptsy);

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  //Start with all of the previous path points from the last time.
  for (int i = 0; i < previousPathSize; ++i)
  {
    next_x_vals.push_back(previousPathXs[i]);
    next_y_vals.push_back(previousPathYs[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity.
  double target_x    = TRAJECTORY_DISTANCE;
  double target_y    = spline(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
  double x_add_on    = 0;

  for (int i = 1; i <= m_maxTrajectoryPoints - previousPathSize; ++i)
  {
    const double newSpeed = m_targetSpeed + kinematics.acceleration;

    if (newSpeed > m_referenceSpeed && kinematics.acceleration > 0)
      m_targetSpeed = m_referenceSpeed;
    else if (newSpeed < kinematics.speed && kinematics.acceleration < 0)
      m_targetSpeed = kinematics.speed;
    else
      m_targetSpeed = newSpeed;

    const double n       = target_dist / (DEFAULT_POINTS_INTERVAL * m_targetSpeed / MILES_CONVERSION_FACTOR);
    double       x_point = x_add_on + target_x / n;
    double       y_point = spline(x_point);

    x_add_on = x_point;

    const double x_ref = x_point;
    const double y_ref = y_point;

    // rotate back to normal after rotating it earlier.
    x_point = x_ref * cos(refYaw) - y_ref * sin(refYaw);
    y_point = x_ref * sin(refYaw) + y_ref * cos(refYaw);

    x_point += refX;
    y_point += refY;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  trajectory.trajectoryX = next_x_vals;
  trajectory.trajectoryY = next_y_vals;

  if (ACTIVATE_LOGS)
    std::cout << g_logStream.str();

  return trajectory;
}

} /* namespace PathPlanning */
} /* namespace CardND */
