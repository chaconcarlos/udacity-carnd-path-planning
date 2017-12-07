/* INCLUDES ******************************************************************/

#include "spline.h"

#include "Math.h"
#include "Navigation.h"
#include "TrajectoryPlanner.h"

/* DEFINITIONS ***************************************************************/

static const int    DEFAULT_TRAJECTORY_POINTS = 50;
static const int    DEFAULT_REFERENCE_LANE    = 1;
static const double DEFAULT_POINTS_INTERVAL   = 0.02;
static const double MAX_SPEED_OFFSET          = 0.5;
static const double COLLISION_DISTANCE        = 30.0;
static const double TRAJECTORY_DISTANCE       = 30.0;
static const double ON_COLLISION_ACCELERATION = .224;

static const std::string STATE_INITIAL               = "II";
static const std::string STATE_KEEP_LANE             = "KL";
static const std::string STATE_PREPARE_LANE_CHANGE_R = "PLCR";
static const std::string STATE_PREPARE_LANE_CHANGE_L = "PLCL";
static const std::string STATE_LANE_CHANCE_L         = "LCL";
static const std::string STATE_LANE_CHANCE_R         = "LCR";

/* STATIC FUNCTIONS **********************************************************/

namespace CardND
{
namespace PathPlanning
{

static bool
checkCollision(
  const double egoVehicleS,
  const int pathSize,
  const int targetLane,
  const std::map<size_t, Vehicle> detectedVehicles)
{
  const double laneD             = 2 + 4 * targetLane;
  const double laneLowerLimit    = laneD - 2;
  const double laneUpperLimit    = laneD + 2;
  bool         possibleCollision = false;

  std::map<size_t, Vehicle>::const_iterator vehicle = detectedVehicles.begin();

  for (; vehicle != detectedVehicles.end(); ++vehicle)
  {
    const double d     = vehicle->second.getD();
    const double speed = vehicle->second.getSpeed();

    if (d > laneLowerLimit && d < laneUpperLimit)
    {
      const double finalPathDistance = static_cast<double>(pathSize) * DEFAULT_POINTS_INTERVAL * speed;
      const double s                 = vehicle->second.getS() + finalPathDistance;
      const bool   isOnSamePath      = s > egoVehicleS;

      if (isOnSamePath && (s - egoVehicleS) < COLLISION_DISTANCE)
      {
        possibleCollision = true;
        break;
      }
    }
  }

  return possibleCollision;
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
, m_currentLane(DEFAULT_REFERENCE_LANE)
, m_currentSpeed(0)
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
  const std::vector<double> previousPathXs    = vehicle.getPreviousPathXs();
  const std::vector<double> previousPathYs    = vehicle.getPreviousPathYs();
  const int                 previousPathSize  = previousPathXs.size();
  double                    currentcarS       = vehicle.getS();
  bool                      possibleCollision = false;
  Trajectory                trajectory;

  if (previousPathSize > 0)
    currentcarS = vehicle.getFinalS();

  possibleCollision = checkCollision(currentcarS, previousPathSize, m_currentLane, vehicle.getDetectedVehicles());

  if (possibleCollision)
    m_currentSpeed -= ON_COLLISION_ACCELERATION;
  else if (m_currentSpeed < m_referenceSpeed)
    m_currentSpeed += ON_COLLISION_ACCELERATION;

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

  const double laneD = 2 + 4 * m_currentLane;

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
    const double n       = target_dist / (DEFAULT_POINTS_INTERVAL * m_currentSpeed / 2.24); // 2.24 from the conversion to m/s
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

  return trajectory;
}

} /* namespace PathPlanning */
} /* namespace CardND */
