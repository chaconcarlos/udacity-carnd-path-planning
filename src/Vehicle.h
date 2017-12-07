#ifndef CARDND_PATHPLANNING_VEHICLE_H_
#define CARDND_PATHPLANNING_VEHICLE_H_

/* INCLUDES ******************************************************************/

#include <map>
#include <vector>

#include "Json/json.hpp"

/* CLASS DECLARATION *********************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Represents a vehicle.
 */
class Vehicle
{
  friend Vehicle* buildVehicleFromMessage(const nlohmann::json& message);

  public:

    /**
     * @brief Initializes an instance of the Vehicle class.
     */
    Vehicle();

    /**
     * @brief Finalizes an instance of the Vehicle class.
     */
    virtual ~Vehicle();

  public:

    /**
     * @brief Gets the X coordinate.
     *
     * @return The X coordinate.
     */
    double getX() const;

    /**
     * @brief Gets the Y coordinate.
     *
     * @return The Y coordinate.
     */
    double getY() const;

    /**
     * @brief Gets the vehicle s coordinate.
     *
     * @return The vehicle s coordinate.
     */
    double getS() const;

    /**
     * @brief Gets the vehicle d coordinate.
     *
     * @return The vehicle d coordinate.
     */
    double getD() const;

    /**
     * @brief Gets the vehicle yaw.
     *
     * @return The vehicle yaw.
     */
    double getYaw() const;

    /**
     * @brief Gets the vehicle speed.
     *
     * @return The vehicle speed.
     */
    double getSpeed() const;

    /**
     * @brief Gets the final s coordinate of the current trajectory.
     *
     * @return The final s coordinate of the current trajectory.
     */
    double getFinalS() const;

    /**
     * @brief Gets the final d coordinate of the current trajectory.
     *
     * @return The final d coordinate of the current trajectory.
     */
    double getFinalD() const;

    /**
     * @brief Gets the x coordinates list of the points in the previous trajectory.
     *
     * @return The x coordinates list of the points in the previous trajectory.
     */
    std::vector<double> getPreviousPathXs() const;

    /**
     * @brief Gets the y coordinates list of the points in the previous trajectory.
     *
     * @return The y coordinates list of the points in the previous trajectory.
     */
    std::vector<double> getPreviousPathYs() const;

    /**
     * @brief Gets the detected vehicles around the current vehicle.
     *
     * @return The map of detected vehicles around the current vehicle.
     */
    std::map<size_t, Vehicle> getDetectedVehicles() const;

  private:

    double                    m_x;
    double                    m_y;
    double                    m_s;
    double                    m_d;
    double                    m_yaw;
    double                    m_speed;
    double                    m_endPathS;
    double                    m_endPathD;
    std::vector<double>       m_previousPathXs;
    std::vector<double>       m_previousPathYs;
    std::map<size_t, Vehicle> m_detectedVehicles;
};

} /* namespace PathPlanning */
} /* namespace CardND */

/* DECLARATIONS **************************************************************/

namespace CardND
{
namespace PathPlanning
{

/**
 * @brief Builds a vehicle from a JSON message.
 *
 * @param message The JSON message.
 *
 * @return The vehicle instance.
 */
Vehicle* buildVehicleFromMessage(const nlohmann::json& message);

} /* namespace PathPlanning */
} /* namespace CardND */

#endif /* CARDND_PATHPLANNING_VEHICLE_H_ */
