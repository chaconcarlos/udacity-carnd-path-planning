/* INCLUDES ******************************************************************/

#include "Road.h"

/* CLASS IMPLEMENTATION ******************************************************/

namespace CardND
{
namespace PathPlanning
{

Road::Road(
    const std::vector<double>& waypointsX,
    const std::vector<double>& waypointsY,
    const std::vector<double>& waypointsS,
    const double maxS,
    const double maxSpeed,
    const int laneCount)
: m_laneCount(laneCount)
, m_maxSpeed(maxSpeed)
, m_mapMaxS(maxS)
, m_waypointsX(waypointsX)
, m_waypointsY(waypointsY)
, m_waypointsS(waypointsS)
{
}

Road::~Road()
{
}

double
Road::getMaxSpeed() const
{
  return m_maxSpeed;
}

double
Road::getMapMaxS() const
{
  return m_mapMaxS;
}

int
Road::getLaneCount() const
{
  return m_laneCount;
}

std::vector<double>
Road::getWaypointsX() const
{
  return m_waypointsX;
}

std::vector<double>
Road::getWaypointsY() const
{
  return m_waypointsY;
}

std::vector<double>
Road::getWaypointsS() const
{
  return m_waypointsS;
}

} /* namespace PathPlanning */
} /* namespace CardND */
