#ifndef CARDND_PATHPLANNING_TRAJECTORYPLANNER_H_
#define CARDND_PATHPLANNING_TRAJECTORYPLANNER_H_

/* INCLUDES ******************************************************************/

#include "Road.h"
#include "Vehicle.h"

/* STRUCT DECLARATION ********************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Struct that represents a trajectory.
 */
struct Trajectory
{
  std::vector<double> trajectoryX;
  std::vector<double> trajectoryY;
};

} /* namespace PathPlanning */
} /* namespace CardND */

/* CLASS DECLARATION *********************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Builds a trajectory point for point for a given vehicle.
 */
class TrajectoryPlanner
{
  public:

    /**
     * @brief Initializes an instance of the TrajectoryPlanner class.
     *
     * @param road The data of the the road.
     */
    TrajectoryPlanner(const Road& road);

    /**
     * @brief Finalizes an instance of the TrajectoryPlanner class.
     */
    virtual ~TrajectoryPlanner();

  public:

    /**
     * @brief Sets the maximum number of trajectory points to generate for the ego vehicle.
     *
     * @param max The maximum number of trajectory points to generate for the ego vehicle. The default is 50.
     */
    void setMaxTrajectoryPoints(const int max);

    /**
     * @brief Sets the interval points to generate the trajectory.
     *
     * @param interval The interval.
     */
    void setPointsInterval(const double interval);

    /**
     * @brief Generates a trajectory for a given vehicle.
     *
     * @param vehicle The vehicle.
     *
     * @return The trajectory for the vehicle.
     */
    Trajectory generateTrajectory(const Vehicle& vehicle);

  private:

    int    m_maxTrajectoryPoints;
    int    m_targetLane;
    double m_targetSpeed;
    double m_pointsInterval;
    double m_referenceSpeed;
    Road   m_road;
};

} /* namespace PathPlanning */
} /* namespace CardND */

#endif /* CARDND_PATHPLANNING_TRAJECTORYPLANNER_H_ */
