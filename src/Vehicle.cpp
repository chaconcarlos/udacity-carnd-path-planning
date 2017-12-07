/* INCLUDES ******************************************************************/

#include "Vehicle.h"

/* USINGS ********************************************************************/

using namespace nlohmann;

/* DEFINITIONS ***************************************************************/

static const int DATA_PACKAGE_VEHICLE_INDEX = 1;

/* CLASS IMPLEMENTATION ******************************************************/

namespace CardND
{
namespace PathPlanning
{

Vehicle::Vehicle()
: m_x(0)
, m_y(0)
, m_s(0)
, m_d(0)
, m_yaw(0)
, m_speed(0)
, m_endPathS(0)
, m_endPathD(0)
{
}

Vehicle::~Vehicle()
{
}

double
Vehicle::getX() const
{
  return m_x;
}

double
Vehicle::getY() const
{
  return m_y;
}

double
Vehicle::getS() const
{
  return m_s;
}

double
Vehicle::getD() const
{
  return m_d;
}

double
Vehicle::getYaw() const
{
  return m_yaw;
}

double
Vehicle::getSpeed() const
{
  return m_speed;
}

double
Vehicle::getFinalS() const
{
  return m_endPathS;
}

double
Vehicle::getFinalD() const
{
  return m_endPathD;
}

std::vector<double>
Vehicle::getPreviousPathXs() const
{
  return m_previousPathXs;
}

std::vector<double>
Vehicle::getPreviousPathYs() const
{
  return m_previousPathYs;
}

std::map<size_t, Vehicle>
Vehicle::getDetectedVehicles() const
{
  return m_detectedVehicles;
}

} /* namespace PathPlanning */
} /* namespace CardND */

/* IMPLEMENTATIONS ***********************************************************/

namespace CardND
{
namespace PathPlanning
{

Vehicle*
buildVehicleFromMessage(const nlohmann::json& message)
{
  const json dataObject = message[DATA_PACKAGE_VEHICLE_INDEX];
  Vehicle*   data       = new Vehicle();

  data->m_x     = dataObject["x"];
  data->m_y     = dataObject["y"];
  data->m_s     = dataObject["s"];
  data->m_d     = dataObject["d"];
  data->m_yaw   = dataObject["yaw"];
  data->m_speed = dataObject["speed"];

  // Previous path data given to the Planner
  data->m_previousPathXs = dataObject["previous_path_x"].get< std::vector<double> >();
  data->m_previousPathYs = dataObject["previous_path_y"].get< std::vector<double> >();

  // Previous path's end s and d values
  data->m_endPathS = dataObject["end_path_s"];
  data->m_endPathD = dataObject["end_path_d"];

  //[ id, x, y, vx, vy, s, d]
  const auto   sensorInformation = message[1]["sensor_fusion"];
  const size_t detectedCarsCount = sensorInformation.size();

  for (size_t i = 0; i < detectedCarsCount; ++i)
  {
    Vehicle detected;

    const size_t id = sensorInformation[i][0];
    const double vx = sensorInformation[i][3];
    const double vy = sensorInformation[i][4];

    detected.m_x        = sensorInformation[i][1];
    detected.m_y        = sensorInformation[i][2];
    detected.m_s        = sensorInformation[i][5];
    detected.m_d        = sensorInformation[i][6];
    detected.m_speed    = sqrt(pow(vx, 2) + pow(vy, 2));

    data->m_detectedVehicles.insert(std::make_pair(id, detected));
  }

  return data;
}

} /* namespace PathPlanning */
} /* namespace CardND */
