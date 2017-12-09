#ifndef CARDND_PATHPLANNING_ROAD_H_
#define CARDND_PATHPLANNING_ROAD_H_

/* INCLUDES ******************************************************************/

#include <vector>

/* CLASS DECLARATION *********************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Represents the data of the road.
 */
class Road
{
  public:

    /**
     * @brief Initializes an instance of the Road class.
     *
     * @param waypointsX The X coordinates of the waypoints of the road.
     * @param waypointsY The Y coordinates of the waypoints of the road.
     * @param waypointsS The s coordinates of the waypoints in the road.
     * @param maxS       The max s coordinate for the road.
     * @param maxSpeed   The max speed for the road.
     * @param laneCount  The lane count for the road.
     */
    Road(
      const std::vector<double>& waypointsX,
      const std::vector<double>& waypointsY,
      const std::vector<double>& waypointsS,
      const double maxS,
      const double maxSpeed,
      const int    laneCount);

    /**
     * @brief Finalizes an instance of the Road class.
     */
    virtual ~Road();

  public:

    /**
     * @brief Gets the lane count for this road.
     *
     * @return The lane count for this road.
     */
    int getLaneCount() const;

    /**
     * @brief Gets the max speed of the road in miles/hour.
     *
     * @return The max speed of the road in miles/hour
     */
    double getMaxSpeed() const;

    /**
     * @brief Gets the max s coordinate for this road.
     *
     * @return The max s coordinate for this road.
     */
    double getMapMaxS() const;

    /**
     * @brief Gets the waypoints x coordinates for the road.
     *
     * @return The waypoints x coordinates for the road.
     */
    std::vector<double> getWaypointsX() const;

    /**
     * @brief Gets the waypoints x coordinates for the road.
     *
     * @return The waypoints x coordinates for the road.
     */
    std::vector<double> getWaypointsY() const;

    /**
     * @brief Gets the waypoints' s coordinates for this road.
     *
     * @return The waypoints' s coordinates for this road.
     */
    std::vector<double> getWaypointsS() const;

  private:

    int                  m_laneCount;
    double               m_maxSpeed;
    double               m_mapMaxS;
    std::vector<double>  m_waypointsX;
    std::vector<double>  m_waypointsY;
    std::vector<double>  m_waypointsS;
};

} /* namespace PathPlanning */
} /* namespace CardND */

#endif /* CARDND_PATHPLANNING_ROAD_H_ */
