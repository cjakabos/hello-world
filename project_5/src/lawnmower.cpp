  
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

#include <iostream>

#include "cluon-complete.hpp"
#include "tme290-sim-grass-msg.hpp"


int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid")) {
    std::cerr << argv[0] 
      << " is a lawn mower control algorithm." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDLV session>" 
      << "[--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --verbose" << std::endl;
    retCode = 1;
  } else {
    bool const verbose{commandlineArguments.count("verbose") != 0};
    uint16_t const cid = std::stoi(commandlineArguments["cid"]);
    ;
    
    cluon::OD4Session od4{cid};

    auto onSensors{[&od4](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<tme290::grass::Sensors>(
            std::move(envelope));
        
        tme290::grass::Control control;

        // Do not change
        double pRainLimit{0.2};
        unsigned int pMaxRow{20};
        unsigned int pMaxColumn{30};

        // Battery and charging
        double pChargingLevel{1.0};
        double pBatteryLevel{0.22};
	
        // Grass limit under robot 
        double pGrassLimit{0.4};

        // Grass limit to specific direction
        double pGrassTopRight{0.4};
        double pGrassRight{0.1};
        double pGrassBottomRight{0.4};
        double pGrassDown{0.001};
        double pGrassBottomLeft{0.1};
        double pGrassLeft{0.5};
        double pGrassTopLeft{0.1};
        double pGrassTopCentre{0.1};


        //1. battery not full, do not leave charging station
        if (msg.i() == 0 && msg.j() == 0 && msg.battery() < pChargingLevel) {
          control.command(0);

        //2. battery level is low, go home
        } else if (msg.battery() <= pBatteryLevel) {

          //2(a) battery level is low, go home
          if (msg.i() > 0 && msg.j() > 0) {
          
            //2(a)/i below but not next to wall, go diagonal right up
            if (msg.i() < 30 && msg.j() > 21){
              control.command(3);

            //2(a)/ii below and next to wall, go right
            } else if (msg.i() < 30 && msg.j() == 21){
              control.command(4);

            //2(a)/iii below and end line of wall, go up
            } else if (msg.i() == pMaxColumn && msg.j() > pMaxRow) {
              control.command(2);

            //2(a)/iv go diagonal left up
            } else {
              control.command(1);
            }

          //2(b) if at edge in Y, go left
          } else if (msg.j() > 0) {
            control.command(2);

          //2(c) if edge in X, go up
          } else if (msg.i() > 0){
            control.command(8);
          }

        //3. grass under robot is over threshold and no rain, cut grass
        } else if (msg.grassCentre() > pGrassLimit && msg.rain() < pRainLimit){
          control.command(0);

        //4. sensor input from nearby cell, if over threshold, move. 
        } else if (msg.grassTopRight() > pGrassTopRight || 
                      msg.grassRight() > pGrassRight || 
                      msg.grassBottomRight() > pGrassBottomRight || 
                      msg.grassBottomCentre() > pGrassDown || 
                      msg.grassBottomLeft() > pGrassBottomLeft || 
                      msg.grassLeft() > pGrassLeft || 
                      msg.grassTopLeft() > pGrassTopLeft || 
                      msg.grassTopCentre() > pGrassTopCentre){

          //4(a) If topright cell has grass over threshold, move there
          if (msg.grassTopRight() > pGrassTopRight){
            control.command(3);

          //4(b) If right cell has grass over threshold, move there
          } else if (msg.grassRight() > pGrassRight){
            control.command(4);

          //4(c)If downright cell has grass over threshold, move there
          } else if (msg.grassBottomRight() > pGrassBottomRight){
            control.command(5);

          //4(d) If down cell has grass over threshold, move there
          } else if (msg.grassBottomCentre() > pGrassDown){
            control.command(6);
	 
          //4(e) If downleft cell has grass over threshold, move there
          } else if (msg.grassBottomLeft() > pGrassBottomLeft ){
            control.command(7);

          //4(f) If left cell has grass over threshold, move there
          } else if (msg.grassLeft() > pGrassLeft){
            control.command(8);
	 
          //4(g) If topleft cell has grass over threshold, move there
          } else if (msg.grassTopLeft() > pGrassTopLeft){
            control.command(1);
	 
          //4(h) If top cell has grass over threshold, move there
          } else if (msg.grassTopCentre() > pGrassTopCentre){
            control.command(2);
          }

        //5. if no sensor input and at corner of wall and map, follow the wall
        } else if (msg.j() == 19 && msg.i() <= 30){
          control.command(4);
		
        //6. no sensor input nearby cell, get sensor data
        } else {
          control.command(0);
        }

        od4.send(control);
      }};

    auto onStatus{[&verbose](cluon::data::Envelope &&envelope)
      {
        auto msg = cluon::extractMessage<tme290::grass::Status>(
            std::move(envelope));
        if (verbose) {
          std::cout << "Status at time " << msg.time() << ": " 
            << msg.grassMean() << "/" << msg.grassMax() << std::endl;
        }
      }};

    od4.dataTrigger(tme290::grass::Sensors::ID(), onSensors);
    od4.dataTrigger(tme290::grass::Status::ID(), onStatus);

    if (verbose) {
      std::cout << "All systems ready, let's cut some grass!" << std::endl;
    }

    tme290::grass::Control control;
    control.command(0);
    od4.send(control);

    while (od4.isRunning()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    retCode = 0;
  }
  return retCode;
}
