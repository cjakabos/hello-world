/*
 * Copyright (C) 2020 Csaba Jakabos
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

// Structs to hold data
struct GridPoint {
  uint32_t i;
  uint32_t j;

  GridPoint(uint32_t a_i, uint32_t a_j): i(a_i), j(a_j) {}
};

struct Point {
  double x;
  double y;

  Point(double a_x, double a_y): x(a_x), y(a_y) {}
};

struct Line {
  double x0;
  double x1;
  double y0;
  double y1;

  Line(double a_x0, double a_y0, double a_x1, double a_y1):
    x0(a_x0), x1(a_x1), y0(a_y0), y1(a_y1) {}

  Point p0() {
    return Point(x0, y0);
  }

  Point p1() {
    return Point(x1, y1);
  }
};

std::vector<Line> walls; // wall container
std::vector<GridPoint> gridList; // grid container
std::vector<GridPoint> exportPath; // export container between scopes
Point exportEndPoint(0, 0); // export container between scopes
double exportGridSize{0.2}; // export container between scopes
double scaleFactor{100}; // for scaling drawings to reasonable size

// Functions
bool checkIntersection(Line a, Line b){
  double s0_x{a.x1 - a.x0};
  double s0_y{a.y1 - a.y0};
  double s1_x{b.x1 - b.x0};
  double s1_y{b.y1 - b.y0};

  double s{(-s0_y * (a.x0 - b.x0) + s0_x * (a.y0 - b.y0)) / 
    (-s1_x * s0_y + s0_x * s1_y)};
  double t{(s1_x * (a.y0 - b.y0) - s1_y * (a.x0 - b.x0)) / 
    (-s1_x * s0_y + s0_x * s1_y)};

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    return true;
  }
  return false;
}

void neighborCheck(GridPoint *currentPointer, 
                   int dj, 
                   int di, 
                   double gridSize, 
                   std::vector<std::vector<double>> &grid, 
                   std::vector<std::vector<int>> &status, 
                   std::vector<std::vector<GridPoint>> &parentMatrix, 
                   int type){
  
  //Define variables
  double neighborCell;
  double distance;
  int statusNeighborCell;


  //Position of neighbor
  int i = currentPointer->i + di;
  int j = currentPointer->j + dj; 

  // grid and status values of neighbor
  neighborCell = grid[j][i]; 
  statusNeighborCell = status[j][i];
  
  // give distance value, based on horizontal/vertical or diagonal
  distance = grid[currentPointer->j][currentPointer->i] + gridSize;
  
  if (type == 1) {
    distance = grid[currentPointer->j][currentPointer->i] + gridSize;
  }
  
  if (type == 2) {
    distance = grid[currentPointer->j][currentPointer->i] + 
      std::sqrt(gridSize * gridSize + gridSize * gridSize);
  }
  
  // if it is wall, don't do anything
  if ((int)neighborCell == -1) {
    return;
  }

  // if it is visited already, don't do anything
  if (statusNeighborCell == 1) {
    return;
  }

  // if grid distance value is less than calculated, update value and parent
  if (distance < grid[j][i]) {
    grid[j][i] = distance;
    GridPoint parentPoint(currentPointer->i, currentPointer->j);
    parentMatrix[j][i] = parentPoint;
  }

  // if neighbor is not visited yet, insert it to gridList and set 5 as listed
  if (statusNeighborCell == 0) {
    GridPoint listedNode(i, j);
    gridList.insert(gridList.end(), listedNode);
    status[j][i] = 1;
  }

  (void) neighborCell;
  (void) distance;
  (void) statusNeighborCell;
}


// Main function
int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") 
      || 0 == commandlineArguments.count("map-file")
      || 0 == commandlineArguments.count("start-x")
      || 0 == commandlineArguments.count("start-y")
      || 0 == commandlineArguments.count("end-x")
      || 0 == commandlineArguments.count("end-y")
      || 0 == commandlineArguments.count("frame-id")
      || 0 == commandlineArguments.count("freq")) {
    std::cerr << argv[0] << " finds a path between to points in a walled "
      "arena, and follows it." << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --freq=10 --frame-id=0 "
      "--map-file=/opt/simulation-map.txt --start-x=0.0 --start-y=0.0 "
      "--end-x=1.0 --end-y=1.0" << std::endl;
    retCode = 1;
  } else {
    bool const verbose = (commandlineArguments.count("verbose") != 0);

    // Part I: Find the path using the map and the start and end points
    std::vector<Point> path;
    {
      double gridSize = exportGridSize;

      Point startPoint(std::stod(commandlineArguments["start-x"]),
            std::stod(commandlineArguments["start-y"]));
      Point endPoint(std::stod(commandlineArguments["end-x"]),
            std::stod(commandlineArguments["end-y"]));
      exportEndPoint = endPoint;

      
      std::ifstream input(commandlineArguments["map-file"]);
      std::cout << "Reading in map file" << std::endl;

      // Parse walls
      uint32_t cellCountX;
      uint32_t cellCountY;
      {
        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::min();
        double maxY = std::numeric_limits<double>::min();

        for (std::string str; getline(input, str);) {
          std::vector<std::string> coordinates = stringtoolbox::split(
              stringtoolbox::split(stringtoolbox::trim(str), ';')[0], ',');
          if (coordinates.size() == 4) {
            double x0{std::stof(coordinates[0])};
            double y0{std::stof(coordinates[1])};
            double x1{std::stof(coordinates[2])};
            double y1{std::stof(coordinates[3])};
            minX = static_cast<double>(std::min(std::min(x0, x1), minX));
            minY = static_cast<double>(std::min(std::min(y0, y1), minY));
            maxX = static_cast<double>(std::max(std::max(x0, x1), maxX));
            maxY = static_cast<double>(std::max(std::max(y0, y1), maxY));
            Line line{x0, y0, x1, y1};
            walls.push_back(line);
            if (verbose) {
              std::cout << "Added wall from [" << x0 << "," << y0 << "] to [" 
                << x1 << "," << y1 << "]" << std::endl;
            }
          }
        }

        double distanceX = maxX - minX;
        double distanceY = maxY - minY;
        std::cout << "distanceX: " << distanceX << std::endl;
        std::cout << "distanceY: " << distanceY << std::endl;
        cellCountX = static_cast<uint32_t>(ceil(distanceX / gridSize));
        cellCountY = static_cast<uint32_t>(ceil(distanceY / gridSize));
        std::cout << "cellCountX: " << cellCountX << std::endl;
        std::cout << "cellCountY: " << cellCountY << std::endl;
      }

      // Allocate grid, set all distances to infinity

      std::cout << "Allocate grid, set all distances to infinity" << std::endl;

      // Define grid structure
      std::vector<std::vector<double>> grid(cellCountY, 
          std::vector<double>(cellCountX, 
              std::numeric_limits<double>::infinity()));

      // Define status structure, 0 for not visited, 
      // -1 for wall and 1 as listed to visit, 5 as visited
      std::vector<std::vector<int>> status(cellCountY, 
          std::vector<int>(cellCountX, 0));

      // Define currentNode
      GridPoint currentNode(0, 0);

      // Define parent structure
      std::vector<std::vector<GridPoint>> parentMatrix(cellCountY, 
          std::vector<GridPoint>(cellCountX, currentNode));

      // Initialize
      std::cout << "Initialize" << std::endl;

      for (uint32_t j = 0; j < cellCountY; j++) {

        for (uint32_t i = 0; i < cellCountX; i++) {
          Point gridP0(i * gridSize, j * gridSize);
          Point gridP1(i * gridSize + gridSize, j * gridSize);
          Point gridP2(i * gridSize, j * gridSize + gridSize);
          Point gridP3(i * gridSize + gridSize, j * gridSize + gridSize);

          for (auto &wall : walls) {
            Point wallP0 = wall.p0();
            Point wallP1 = wall.p1();

            // COMPLETE: If there is a wall in the grid, set grid value to -1:
            // Vertical walls
            if ((i * gridSize) <= wallP0.x &&
                wallP0.x <= (i * gridSize + gridSize) &&
                (i * gridSize) <= wallP1.x &&
                wallP1.x <= (i * gridSize + gridSize) &&
                j * gridSize >= (double)wallP0.y &&
                j * gridSize <= (double)wallP1.y) {

              grid[j][i] = -1.0;
              status[j][i] = -1;

            // Horizontal walls
            } else if ((j * gridSize) <= wallP0.y && 
                       wallP0.y <= (j * gridSize + gridSize) &&
                       (j * gridSize) <= wallP1.y &&
                       wallP1.y <= (j * gridSize + gridSize) &&
                       i * gridSize >= (double)wallP0.x &&
                       i * gridSize <= (double)wallP1.x) {

              grid[j][i] = -1.0;
              status[j][i] = -1;
            } 

            // COMPLETE: If the start position is in the grid cell, do:


            // Checking start node and its validity
            if (i * gridSize < startPoint.x &&
                startPoint.x < i * gridSize + gridSize &&
                j * gridSize < startPoint.y &&
                startPoint.y < j * gridSize + gridSize) {

              if ((int)grid[j][i] == -1) {
                std::cout << "--==WRONG INPUT, START AT WALL==--" << std::endl;
                exit (EXIT_FAILURE);
              }

              grid[j][i] = 0.0;
              currentNode = GridPoint(i, j);
            }

            // Checking end node validity
            if (i * gridSize <= endPoint.x &&
                endPoint.x <= i * gridSize + gridSize &&
                j * gridSize <= endPoint.y &&
                endPoint.y <= j * gridSize + gridSize) { 
          
              if ((int)grid[j][i] == -1) {
                std::cout << "--==WRONG INPUT, END AT WALL==--" << std::endl;
                exit (EXIT_FAILURE);
              }

            }

            (void) gridP0; // Remove when used
            (void) gridP1; // Remove when used
            (void) gridP2; // Remove when used
            (void) gridP3; // Remove when used
            (void) wallP0; // Remove when used
            (void) wallP1; // Remove when used
          }
        }
      }

      // Find the path
      std::cout << "Find the path" << std::endl;

      {
        bool pathFound = false;
        std::vector<GridPoint> gridPath;
        std::vector<GridPoint> sortPath;

	GridPoint *currentPointer;
	currentPointer = &currentNode;

        while (!pathFound) {
          // COMPLETE: Run your path search here!


          // Check current status to visited "1"
          status[currentNode.j][currentNode.i] = 1;

          // Check neighbors, 1 for vertical and horizontal, 2 diagonal
          neighborCheck(currentPointer, 0, 1, 
                        gridSize, grid, status, parentMatrix, 1);
          neighborCheck(currentPointer, 1, 0, 
                        gridSize, grid, status, parentMatrix, 1);
          neighborCheck(currentPointer, 1, 1, 
                        gridSize, grid, status, parentMatrix, 2);
          neighborCheck(currentPointer, 1, -1, 
                        gridSize, grid, status, parentMatrix, 2);

          // Check "negative" neighbors only if i or j is non-zero
          if (currentNode.i > 0 && currentNode.j > 0) { 
            neighborCheck(currentPointer, -1, 0, 
                          gridSize, grid, status, parentMatrix, 1);
            neighborCheck(currentPointer, 0, -1, 
                          gridSize, grid, status, parentMatrix, 1);
            neighborCheck(currentPointer, -1, 1, 
                          gridSize, grid, status, parentMatrix, 2);
            neighborCheck(currentPointer, -1, -1, 
                          gridSize, grid, status, parentMatrix, 2);
          }

          if (currentNode.i == 0 && currentNode.j > 0) { 
            neighborCheck(currentPointer, -1, 0, 
                          gridSize, grid, status, parentMatrix, 1);
          }

          if (currentNode.i > 0 && currentNode.j == 0) { 
            neighborCheck(currentPointer, 0, -1, 
                          gridSize, grid, status, parentMatrix, 1);
          }

          //std::cout << "check 3" << std::endl;
          // Define smallest as infinite and compare distances to it
          double smallest = std::numeric_limits<double>::infinity();

          // smallestIndex to hold index of smallest
          int smallestIndex{-1};


          // Check neighbors and hold smallest value and index
          for (int i = 0; i<(int)gridList.size(); i++) {
              if (smallest > grid[gridList[i].j][gridList[i].i]) {
                  smallest = grid[gridList[i].j][gridList[i].i]; 
                  smallestIndex = i; 
              }
          }

          // set the next currentNode to the smallest distance
          currentNode = GridPoint(gridList[smallestIndex].i, 
                                  gridList[smallestIndex].j);

          // remove smallest from gridList, as it will be visited
          if ((int)gridList.size()) {
            gridList.erase(gridList.begin() + smallestIndex);
          }

          // check if the new currentNode is at endPoint
          if (currentNode.i * gridSize < endPoint.x &&
            endPoint.x < currentNode.i * gridSize + gridSize &&
            currentNode.j * gridSize < endPoint.y &&
            endPoint.y < currentNode.j * gridSize + gridSize) {
            pathFound = true;
          }
        }
        // end of while loop



        std::cout << "Calculating path" << std::endl; 

        // Define motherNode
        GridPoint motherNode(0, 0);


        bool startFound = false;

        while (!startFound) {

          // Gather motherNode from the currentNode parentmatrix value
          motherNode = parentMatrix[currentNode.j][currentNode.i];

          // Add motherNode to gridPath
          gridPath.insert(gridPath.end(), motherNode);

          // if current motherNode matches start path is found
          if (motherNode.i * gridSize - startPoint.x <= 0.0 && 
              motherNode.j * gridSize - startPoint.y <= 0.0) {

            startFound = true;
          }

          // otherwise set motherNode to currentNode
          currentNode = motherNode;

        }

        //Export path to control part
        exportPath=gridPath;

      }

      // Visualise the grid
      if (verbose) {
        std::cout << "Visualise the grid" <<  walls.size() << std::endl;
 
        // Get size of map
        double xMax{0.0};
        double yMax{0.0};
        for (auto &wall : walls) {
          Point wallP0 = wall.p0();
          Point wallP1 = wall.p1();

          if (wallP0.x > xMax) {
            xMax = wallP0.x;
          } else if (wallP0.y > yMax) {
            yMax = wallP0.y;
          } else if (wallP1.x > xMax) {
            xMax = wallP1.x;
          } else if (wallP1.y > yMax) {
            yMax = wallP1.y;
          } else {
          }

        }
        
        std::cout << "xMax is = " << xMax << " yMax is = " << yMax << std::endl;
        
        // Define map area
        uint32_t w = (int)xMax * (int)scaleFactor + 1;
        uint32_t h = (int)yMax * (int)scaleFactor + 1;
        cv::Mat gridMap(h, w, CV_8UC3, cv::Scalar(0, 0, 0));

        // Print grids to map area
        for (uint32_t j = 0; j < cellCountY; j++) {
          for (uint32_t i = 0; i < cellCountX; i++) {
            Point gridP0(i * gridSize * scaleFactor, j * gridSize * scaleFactor);
            Point gridP3((i * gridSize + gridSize) * scaleFactor, 
                         (j * gridSize + gridSize) * scaleFactor);
 
              // Walls red
              if ((int)grid[j][i] == -1) {
                cv::rectangle(gridMap, 
                              cv::Point((int)gridP0.x, (int)gridP0.y), 
                              cv::Point((int)gridP3.x, (int)gridP3.y), 
                              cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
              // Otherwise white 
              } else {
                cv::rectangle(gridMap, 
                              cv::Point((int)gridP0.x, (int)gridP0.y), 
                              cv::Point((int)gridP3.x, (int)gridP3.y), 
                              cv::Scalar(255, 255, 255), CV_FILLED, 8, 0);
              }
          }
        }


        // Print start and end point
        cv::circle(gridMap, 
                   cv::Point((int)(startPoint.x * scaleFactor),
                   (int)(startPoint.y * scaleFactor)), 
                   (int)(gridSize * scaleFactor / 5), 
                   cv::Scalar(255, 0, 0),
                   CV_FILLED, 8,0);
        cv::circle(gridMap, 
                   cv::Point((int)(endPoint.x * scaleFactor),
                   (int)(endPoint.y * scaleFactor)), 
                   (int)(gridSize * scaleFactor / 5),
                   cv::Scalar(0, 0, 0),
                   CV_FILLED, 8,0);

        // Reverse it for control part
        std::reverse(exportPath.begin(),exportPath.end());   

        // Print path
        for (int k = 0; k<(int)exportPath.size(); k++) {
          cv::circle(gridMap, 
                     cv::Point((int)(exportPath[k].i * gridSize * scaleFactor),
                     (int)(exportPath[k].j * gridSize * scaleFactor)), 
                     (int)(gridSize * scaleFactor / 5), 
                     cv::Scalar(0, 128, 0),CV_FILLED, 8,0);
        }

        // Flip picture around X axis to get normal position
        cv::flip(gridMap, gridMap, 0);
        
        // Show map
        cv::imshow("Grid map", gridMap);
        cv::waitKey(1);
      }
    }
    // .. by leaving this scope, only the "path" and "verbose" are saved
    // A well-scoped design helps the reader to know how the microservice is
    // structured.
 

    // Part II: Path found, set up the OD4 session and start the path follower
    uint16_t const cid = std::stoi(commandlineArguments["cid"]);
    float const freq = std::stof(commandlineArguments["freq"]);
    uint32_t const frameId = static_cast<uint32_t>(
        std::stoi(commandlineArguments["frame-id"]));

    cluon::OD4Session od4(cid);

    opendlv::sim::Frame latestFrame;
    double distanceFront = 0.0;
    double distanceLeft = 0.0;
    double distanceRear = 0.0;
    double distanceRight = 0.0;

    std::mutex frameMutex;
    std::mutex distanceMutex;

    auto onFrame{[&frameId, &latestFrame, &frameMutex, &verbose](
        cluon::data::Envelope &&envelope)
      {
        uint32_t const senderStamp = envelope.senderStamp();
        if (frameId == senderStamp) {
          std::lock_guard<std::mutex> const lock(frameMutex);
          latestFrame = cluon::extractMessage<opendlv::sim::Frame>(
              std::move(envelope));
          //if (verbose) {
          //  std::cout << "Robot position [" << latestFrame.x() << ", " 
          //    << latestFrame.y() << ", " << latestFrame.yaw() << std::endl;
          //}
        }
    }};

    auto onDistanceReading{[&distanceFront, &distanceRear, &distanceMutex](
        cluon::data::Envelope &&envelope)
      {
        uint32_t const senderStamp = envelope.senderStamp();
        auto distanceReading = 
          cluon::extractMessage<opendlv::proxy::DistanceReading>(
              std::move(envelope));
          
        std::lock_guard<std::mutex> const lock(distanceMutex);
        if (senderStamp == 0) {
          distanceFront = distanceReading.distance();
        } else {
          distanceRear = distanceReading.distance();
        }
      }};

    auto onVoltageReading{[&distanceLeft, &distanceRight, &distanceMutex](
        cluon::data::Envelope &&envelope)
      {
        uint32_t const senderStamp = envelope.senderStamp();
        auto voltageReading = 
          cluon::extractMessage<opendlv::proxy::VoltageReading>(
              std::move(envelope));

        double voltageDividerR1 = 1000.0;
        double voltageDividerR2 = 1000.0;

        double sensorVoltage = (voltageDividerR1 + voltageDividerR2) 
          / voltageDividerR2 * voltageReading.voltage();
        double distance = (2.5 - sensorVoltage) / 0.07;

        std::lock_guard<std::mutex> const lock(distanceMutex);
        if (senderStamp == 0) {
          distanceLeft = distance;
        } else {
          distanceRight = distance;
        }
      }};

    auto atFrequency{[&latestFrame, &frameMutex, &distanceFront, &distanceLeft, 
      &distanceRear, &distanceRight, &distanceMutex, &path, &od4, &verbose]() 
        -> bool
      {
        double posX;
        double posY;
        double posYaw;
        double distFront;
        double distLeft;
        double distRear;
        double distRight;
 
        {
          std::lock_guard<std::mutex> const lock(frameMutex);
          posX = latestFrame.x();
          posY = latestFrame.y();
          posYaw = latestFrame.yaw();
        }
        {
          std::lock_guard<std::mutex> const lock(distanceMutex);
          distFront = distanceFront;
          distLeft = distanceLeft;
          distRear = distanceRear;
          distRight = distanceRight;
        }

        float groundSteering = 0.0f;
        float pedalPosition = 0.08f;

        // COMPLETE: Use the path, the current position, and possibly the
        // distance readings to calculate steering and throttle.

        // Find closest on exportPath
        double closest = std::numeric_limits<double>::infinity();
        int closeInd{-1};
        for (int i = 0; i<(int)exportPath.size(); i++) {
            if (closest > std::abs(exportPath[i].i * exportGridSize - posX) + 
                std::abs(exportPath[i].j * exportGridSize - posY)) { 
              closest = std::abs(exportPath[i].i * exportGridSize - posX) + 
              std::abs(exportPath[i].j * exportGridSize - posY); 
              closeInd = i; 
            }
        }

        // Define aim values and set it the one after the closest
        double aimX;
        double aimY;
        int aimInd{0};

        // If it is the second last always aim for the last
        if ((int)closeInd >= (int)(exportPath.size()-1)) {
          aimInd = exportPath.size()-1;
        } else {
          aimInd = closeInd + 1;
        }

        // Define aim point to the center of the picked path point
        aimX = exportPath[aimInd].i * exportGridSize + exportGridSize / 2;
        aimY = exportPath[aimInd].j * exportGridSize + exportGridSize / 2;
        

        // Calculate errors in X and Y
        double Ye = aimY-posY;
        double L = aimX-posX;
        double eT{0.0};

        // Calculate error in angles, due to atan being always positive
        // negative L values and Yaw angle has to be corrected
        if (L > 0) { 
          eT = atan(Ye/L) - posYaw;
        } else if (L < 0){

          if (posYaw > 0) { 
            eT = atan(Ye/L) - posYaw + 3.14159265; // +180 deg
          } else if (posYaw < 0){
            eT = atan(Ye/L) - posYaw - 3.14159265; // -180 deg
          } 

        } 

        // Tuning factor
        double magicK = 1.0;

        // Tuning factor
        groundSteering = (float)(magicK * eT);

        // Check if end reached
        if (std::abs(exportEndPoint.x - posX) < exportGridSize && 
            std::abs(exportEndPoint.y - posY) < exportGridSize){

          std::cout << "GOAL IS REACHED, PLEASE SWITCH ME OFF" << std::endl;
          groundSteering = 0.0f;
          pedalPosition = 0.0f;

        }

        opendlv::proxy::GroundSteeringRequest groundSteeringRequest;
        groundSteeringRequest.groundSteering(groundSteering);

        opendlv::proxy::PedalPositionRequest pedalPositionRequest;
        pedalPositionRequest.position(pedalPosition);
        
        cluon::data::TimeStamp sampleTime;
        od4.send(groundSteeringRequest, sampleTime, 0);
        od4.send(pedalPositionRequest, sampleTime, 0);

        if (verbose) {
          // Visualise the path and the robot

          // Get size of map
          double xMax{0.0};
          double yMax{0.0};
          for (auto &wall : walls) {
            Point wallP0 = wall.p0();
            Point wallP1 = wall.p1();

            if (wallP0.x > xMax) {
              xMax = wallP0.x;
            } else if (wallP0.y > yMax) {
              yMax = wallP0.y;
            } else if (wallP1.x > xMax) {
              xMax = wallP1.x;
            } else if (wallP1.y > yMax) {
              yMax = wallP1.y;
            } else {
            }
        }
        
        // Define map area
        uint32_t w = (int)xMax * (int)scaleFactor + 1;
        uint32_t h = (int)yMax * (int)scaleFactor + 1;
        cv::Mat globalMap(h, w, CV_8UC3, cv::Scalar(0, 0, 0));

        // Print walls to map area
        for (auto &wall : walls) {
          Point wallP0 = wall.p0();
          Point wallP1 = wall.p1();

          cv::line(globalMap, 
                   cv::Point((int)(wallP0.x * scaleFactor), 
                     (int)(wallP0.y * scaleFactor)), 
                   cv::Point((int)(wallP1.x * scaleFactor), 
                     (int)(wallP1.y * scaleFactor)), 
                   cv::Scalar(255, 0, 0), 2, cv::LINE_8);
        }

        // Define Kiwi rectangle
        cv::RotatedRect RotatedRect(cv::Point((int)(posX * scaleFactor),
                                      (int)(posY * scaleFactor)), 
                                    cv::Size2f(float(0.36 * scaleFactor),
                                      float(0.16 * scaleFactor)), 
                                    (float)(posYaw * 57.2957795));


        // We take the edges that OpenCV calculated for us
        cv::Point2f recPoints2f[4];
        RotatedRect.points(recPoints2f);

        // Convert them so we can use them in a fillConvexPoly
        cv::Point recPoints[4];
        for(int i = 0; i < 4; ++i){
          recPoints[i] = recPoints2f[i];
        }

        // Now we can fill the rotated rectangle with our specified color
        cv::fillConvexPoly(globalMap,
                           recPoints,
                           4,
                           cv::Scalar(0, 0, 255));


        // Flip picture around X axis to get normal position
        cv::flip(globalMap, globalMap, 0);

        // Show map
          cv::imshow("Global map", globalMap);
          cv::waitKey(1);
        }

        (void) posX; // Remove when used
        (void) posY; // Remove when used
        (void) posYaw; // Remove when used
        (void) distFront; // Remove when used
        (void) distLeft; // Remove when used
        (void) distRear; // Remove when used
        (void) distRight; // Remove when used
 
        return true;
      }};

    // Register the three data triggers, each spawning a thread
    od4.dataTrigger(opendlv::sim::Frame::ID(), onFrame);
    od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
    od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);
    
    // Register the time trigger, spawning a thread that blocks execution 
    // until CTRL-C is pressed
    od4.timeTrigger(freq, atFrequency);
  }
  return retCode;
}
