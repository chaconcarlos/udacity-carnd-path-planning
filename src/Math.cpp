/* INCLUDES ******************************************************************/

#include "Math.h"

/* INCLUDES ******************************************************************/

#include <math.h>

/* DEFINITIONS ***************************************************************/

static constexpr double PI = M_PI;

/* CLASS IMPLEMENTATION ******************************************************/

namespace CardND
{
namespace PathPlanning
{

double
degreesToRadians(const double degrees)
{
  return degrees * PI / 180;
}

double
radiansToDegrees(const double radians)
{
  return radians * 180 / PI;
}

double
getDistance(const double x1, const double y1, const double x2, const double y2)
{
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

} /* namespace PathPlanning */
} /* namespace CardND */
