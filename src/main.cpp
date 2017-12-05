/* INCLUDES ***************************************************************/

#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Json/json.hpp"
#include "Math.h"
#include "Navigation.h"
#include "spline.h"

/* USINGS ********************************************************************/

using namespace CardND::PathPlanning;

using json = nlohmann::json;

/* DEFINITIONS ***************************************************************/

static const int         SIMULATOR_PORT            = 4567;
static const int         DATA_PACKAGE_OBJECT_INDEX = 1;
static const int         REFERENCE_LANE            = 1;
static const double      S_MAX                     = 6945.554; // The max s value before wrapping around the track back to 0
static const double      REFERENCE_VELOCITY        = 49.5;
static const std::string DATA_FILENAME             = "./data/highway_map.csv";
static const std::string EVENT_TELEMETRY           = "telemetry";
static const std::string RESPONSE_MANUAL           = "42[\"manual\",{}]";

/* STRUCT DECLARATION ********************************************************/

struct CarDataPackage
{
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;

  // Previous path data given to the Planner
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;

  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  //auto sensor_fusion = messageJson[1]["sensor_fusion"];
};

/* STATIC DECLARATIONS *******************************************************/

static uWS::Hub            g_webSocketHub;
static int                 g_currentLane;
static double              g_currentVelocity;
static std::vector<double> g_waypointsX;
static std::vector<double> g_waypointsY;
static std::vector<double> g_waypointsS;
static std::vector<double> g_waypointsDx;
static std::vector<double> g_waypointsDy;

/* PROTOTYPES ****************************************************************/

/**
 * @brief Checks if the SocketIO event message has JSON data.
 *
 * @param message The message to check.
 *
 * @return The JSON data. If no data is found, returns an empty string.
 */
static std::string getData(const std::string& message);

/**
 * @brief Parses the JSON message to get the car data.
 *
 * @param message The message.
 *
 * @return The car data package.
 */
static CarDataPackage parseMessage(const json& message);

/**
 * @brief Loads a data file.
 *
 * @param dataFilename The name of the file to load.
 */
static void loadDataFile(const std::string& dataFilename);

/**
 * @brief Executes when the websocket is connected.
 *
 * @param webSocket The web socket.
 * @param webSocket The HTTP request.
 */
static void onConnection(uWS::WebSocket<uWS::SERVER> webSocket, uWS::HttpRequest request);

/**'
 * @brief Executes when the websocket is disconnected.
 */
static void onDisconnection(uWS::WebSocket<uWS::SERVER> webSocket, int param1, char* data, size_t length);

/**
 * @brief Executes when a HTTP request is received.
 */
static void onHttpRequest(uWS::HttpResponse* res, uWS::HttpRequest req, char* data, size_t bytesToRead, size_t length);

/**
 * @brief Executes when a message is received by the web socket.
 *
 * @param ws     The web socket.
 * @param data   The message data.
 * @param length The lenght of the data.
 * @param opCode The operation code.
 */
static void onMessage(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode);

/* IMPLEMENTATION ************************************************************/

std::string
getData(const std::string& message)
{
  const auto  b1     = message.find_first_of("[");
  const auto  b2     = message.find_first_of("}");
  std::string result = "";

   if (b1 != std::string::npos && b2 != std::string::npos)
    result = message.substr(b1, b2 - b1 + 2);

  return result;
}

CarDataPackage
parseMessage(const json& message)
{
  const json     dataObject = message[DATA_PACKAGE_OBJECT_INDEX];
  CarDataPackage data;

  data.x     = dataObject["x"];
  data.y     = dataObject["y"];
  data.s     = dataObject["s"];
  data.d     = dataObject["d"];
  data.yaw   = dataObject["yaw"];
  data.speed = dataObject["speed"];

  // Previous path data given to the Planner
  data.previous_path_x = dataObject["previous_path_x"].get< std::vector<double> >();
  data.previous_path_y = dataObject["previous_path_y"].get< std::vector<double> >();

  // Previous path's end s and d values
  data.end_path_s = dataObject["end_path_s"];
  data.end_path_d = dataObject["end_path_d"];

  return data;
}

void
loadDataFile(const std::string& dataFilename)
{
  std::ifstream dataFile(dataFilename.c_str(), std::ifstream::in);
  std::string   line;

  while (getline(dataFile, line))
  {
    std::istringstream iss(line);

    double x;
    double y;
    float  s;
    float  d_x;
    float  d_y;

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;

    g_waypointsX.push_back(x);
    g_waypointsY.push_back(y);
    g_waypointsS.push_back(s);
    g_waypointsDx.push_back(d_x);
    g_waypointsDy.push_back(d_y);
  }
}

void
onConnection(uWS::WebSocket<uWS::SERVER>, uWS::HttpRequest)
{
  std::cout << "Connected!!!" << std::endl;
}

void
onDisconnection(uWS::WebSocket<uWS::SERVER> webSocket, int, char*, size_t)
{
  webSocket.close();
  std::cout << "Disconnected" << std::endl;
}

void
onHttpRequest(uWS::HttpResponse* httpResponse, uWS::HttpRequest request, char* data, size_t bytesToRead, size_t length)
{
  const std::string response = "<h1>Hello world!</h1>";

  if (request.getUrl().valueLength == 1)
    httpResponse->end(response.data(), response.length());
  else
    httpResponse->end(nullptr, 0);
}

void
onMessage(uWS::WebSocket<uWS::SERVER> webSocket, char* data, size_t length, uWS::OpCode opCode)
{
  if (length && length > 2 && data[0] == '4' && data[1] == '2')
  {
    const std::string rawData = getData(std::string(data));

    if (rawData.empty())
    {
     webSocket.send(RESPONSE_MANUAL.data(), RESPONSE_MANUAL.length(), uWS::OpCode::TEXT);
     return;
    }

    auto        messageJson = json::parse(rawData);
    std::string event       = messageJson[0].get<std::string>();

    if (event == EVENT_TELEMETRY)
    {
      CarDataPackage car = parseMessage(messageJson);

      // Sensor Fusion Data, a list of all other cars on the same side of the road.
      auto sensor_fusion = messageJson[1]["sensor_fusion"];

      int previousPathSize = car.previous_path_x.size();

//      double lane    = REFERENCE_LANE;
//      double ref_vel = car.speed;

      bool too_close = false;

      if (previousPathSize > 0)
      {
        car.s = car.end_path_s;
      }

      // find ref_v to use
      for (int i = 0; i < sensor_fusion.size(); ++i)
      {
        //car is in my lane?
        float d = sensor_fusion[i][6];

        if (d < (2 + 4 * g_currentLane + 2) && d > (2 + 4 * g_currentLane - 2))
        {
          double vx          = sensor_fusion[i][3];
          double vy          = sensor_fusion[i][4];
          double check_speed = sqrt(pow(vx, 2) + pow(vy, 2));
          double check_car_s = sensor_fusion[i][5];

          check_car_s += ((double) previousPathSize * 0.02 * check_speed);

          if ((check_car_s > car.s) && (check_car_s - car.s) < 30)
          {
            std::cout << "TOO CLOSE!!!!" << std::endl;
            too_close = true;

            if (g_currentLane > 0)
              g_currentLane = 0;
          }
        }
      }

      if (too_close)
      {
        g_currentVelocity -= .224;
      }
      else if (g_currentVelocity < REFERENCE_VELOCITY)
      {
        g_currentVelocity += .224;
      }


      // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
      // Later we will interpolate these waypoints with a spline
      std::vector<double> ptsx;
      std::vector<double> ptsy;

      // reference x, y, yaw states
      // either we will reference the starting point as where the car is or at the previous path's end point.
      double ref_x   = car.x;
      double ref_y   = car.y;
      double ref_yaw = degreesToRadians(car.yaw);

      // If previous size is almost empty, use the car as starting reference.
      if (previousPathSize < 2)
      {
        // Use two points that make the path tangent to the car.
        const double prev_car_x = car.x - cos(car.yaw);
        const double prev_car_y = car.y - sin(car.yaw);

        ptsx.push_back(prev_car_x);
        ptsy.push_back(prev_car_y);

        ptsx.push_back(car.x);
        ptsy.push_back(car.y);
      }
      else
      {
        // Use the previous path's end point as starting reference.
        const double ref_x_prev = car.previous_path_x[previousPathSize - 2];
        const double ref_y_prev = car.previous_path_y[previousPathSize - 2];

        ref_x   = car.previous_path_x[previousPathSize - 1];
        ref_y   = car.previous_path_y[previousPathSize - 1];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsy.push_back(ref_y_prev);
        ptsx.push_back(ref_x);
        ptsy.push_back(ref_y);
      }

      std::vector<double> next_wp0 = toCartesian(car.s + 30, 2 + 4 * g_currentLane, g_waypointsS, g_waypointsX, g_waypointsY);
      std::vector<double> next_wp1 = toCartesian(car.s + 60, 2 + 4 * g_currentLane, g_waypointsS, g_waypointsX, g_waypointsY);
      std::vector<double> next_wp2 = toCartesian(car.s + 90, 2 + 4 * g_currentLane, g_waypointsS, g_waypointsX, g_waypointsY);

      ptsx.push_back(next_wp0[0]);
      ptsx.push_back(next_wp1[0]);
      ptsx.push_back(next_wp2[0]);

      ptsy.push_back(next_wp0[1]);
      ptsy.push_back(next_wp1[1]);
      ptsy.push_back(next_wp2[1]);

      for (int i = 0; i < ptsx.size(); ++i)
      {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
      }

      tk::spline s;

      s.set_points(ptsx, ptsy);

      std::vector<double> next_x_vals;
      std::vector<double> next_y_vals;

      //Start with all of the previous path points from the last time.
      for (int i = 0; i < previousPathSize; ++i)
      {
        next_x_vals.push_back(car.previous_path_x[i]);
        next_y_vals.push_back(car.previous_path_y[i]);
      }

      // Calculate how to break up spline points so that we travel at our desired reference velocity.
      double target_x    = 30.0;
      double target_y    = s(target_x);
      double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));
      double x_add_on    = 0;

      for (int i = 1; i <= 50 - previousPathSize; ++i)
      {
        double n = target_dist / (0.02 * g_currentVelocity / 2.24); // 2.24 from the conversion to m/s
        double x_point = x_add_on + target_x / n;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        // rotate back to normal after rotating it earlier.
        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
      }

      // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds.

      json msgJson;

      msgJson["next_x"] = next_x_vals;
      msgJson["next_y"] = next_y_vals;

      auto msg = "42[\"control\","+ msgJson.dump()+"]";

      webSocket.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  }
}

int main()
{
  loadDataFile(DATA_FILENAME);

  g_webSocketHub.onMessage(&onMessage);
  g_webSocketHub.onHttpRequest(&onHttpRequest);
  g_webSocketHub.onConnection(&onConnection);
  g_webSocketHub.onDisconnection(&onDisconnection);

  if (g_webSocketHub.listen(SIMULATOR_PORT))
  {
    std::cout << "Listening to port " << SIMULATOR_PORT << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  g_currentLane     = REFERENCE_LANE;
  g_currentVelocity = 0;

  g_webSocketHub.run();
}
