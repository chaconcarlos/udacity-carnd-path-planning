/* INCLUDES ***************************************************************/

#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <thread>
#include <uWS/uWS.h>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Json/json.hpp"
#include "Math.h"
#include "Navigation.h"
#include "TrajectoryPlanner.h"

/* USINGS ********************************************************************/

using namespace CardND::PathPlanning;

using json = nlohmann::json;

/* DEFINITIONS ***************************************************************/

static const int         SIMULATOR_PORT  = 4567;
static const int         LANE_COUNT      = 3;
static const double      S_MAX           = 6945.554; // The max s value before wrapping around the track back to 0
static const double      MAX_SPEED       = 50;
static const std::string DATA_FILENAME   = "./data/highway_map.csv";
static const std::string EVENT_TELEMETRY = "telemetry";
static const std::string RESPONSE_MANUAL = "42[\"manual\",{}]";

/* STATIC DECLARATIONS *******************************************************/

static uWS::Hub           g_webSocketHub;
static Road*              g_road;
static TrajectoryPlanner* g_planner;

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

void
loadDataFile(const std::string& dataFilename)
{
  std::ifstream dataFile(dataFilename.c_str(), std::ifstream::in);
  std::string   line;

  static std::vector<double> g_waypointsX;
  static std::vector<double> g_waypointsY;
  static std::vector<double> g_waypointsS;

  while (getline(dataFile, line))
  {
    std::istringstream iss(line);

    double x;
    double y;
    float  s;

    iss >> x;
    iss >> y;
    iss >> s;

    g_waypointsX.push_back(x);
    g_waypointsY.push_back(y);
    g_waypointsS.push_back(s);
  }

  g_road    = new Road(g_waypointsX, g_waypointsY, g_waypointsS, S_MAX, MAX_SPEED, LANE_COUNT);
  g_planner = new TrajectoryPlanner(*g_road);
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
      // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds.
      std::ofstream output("/tmp/car.json");
      output << rawData;
      output.close();

      const Vehicle*   ego     = buildVehicleFromMessage(messageJson);
      const Trajectory newPath = g_planner->generateTrajectory(*ego);

      json msgJson;

      msgJson["next_x"] = newPath.trajectoryX;
      msgJson["next_y"] = newPath.trajectoryY;

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

  g_webSocketHub.run();
}
