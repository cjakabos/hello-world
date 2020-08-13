## Introduction
In this project the program controls a lawnmower robot to cut grass, in a way that it keeps the grass in the area as short as possible (given by the Status message), and which never runs out of batteries. To test the behavior an already existing grass simulator is used as an OpenDLV microservice. Link: https://github.com/olbender/tme290-lawnmower

<p align="center">
  <a href="./images/Project5.gif"><img src="./images/Project5.gif" alt="Overview" width="60%" height="60%"></a>
</p>

## The simulation works according to:

 - The robot can move around in a 40x40 grid, where each grid cell keeps the grass length as a value [0,1]. The initial values are 1.0. The robot starts at (0,0),    where also the charging station is located. No grass will grow at the charging station.

 - From position (0, 20) to (29, 20) there is a wall that cannot be moved through. There is no grass growing in the cells occupied by the wall.

 - Time is counted in steps of minutes, where each action of the robot takes one minute to carry out. Every time step, the grass grows slightly in all grid cells.

 - The robot can carry out the following actions (values 0-8): Stay in the same grid cell (0), travel up left (1), up (2), up right (3), right (4), down right (5),    down (6), down left (7), and left (8). If the robots passes a grid cell some grass is cut, and if the robots stays in a grid cell for one turn all grass is cut.    If the robot stays at the charging station, the batteries are charged. Furthermore, if the robot stays in a grid cell to cut all grass, it will also sense how       much grass grows in all adjacent cells (given in the Sensors message), otherwise values of -1 will be given.

 - With a certain probability, it might start to rain. The amount of rain is given as a value [0,1] and if the value reaches above 0.2 no grass can be cut.

 - For each move, the robots batteries will drain slightly, and if staying put in a grid cell other than the charging station there will be a larger drainage. If    the batteries are depleted, the robot can not be recovered.
 
##  Behaviour-based robotics (BBR) solution
More info: https://en.wikipedia.org/wiki/Behavior-based_robotics

In general, when we behaviour-based solutions the behaviors of the robot should be well defined, well described, and well organized (clear conditions between behaviors). Note that the BBR pattern requires you to define isolated behavioural blocks that contains and isolates atomic actions
The logic of the robot follows: survival, then the main task cutting grass, then moving and getting new information, if needed. The behaviour blocks are the following:

1.	If: battery not full, do not leave charging station

2.	Else if: battery level is low, go home (when out in the field)

    - If not at edge of map, then go

      - If: below but not next to wall, go diagonal right up

      - Else if: below and next to wall, go right

      - Else if: below and end line of wall, go up

      - Else: go diagonal left up

    - Else if at edge in Y, go left

    - Else if edge in X, go up

3.	Else if: grass under robot is over threshold and no rain, cut grass. 

4.	Else if: sensor input from nearby cell, if over threshold, move. Follow clockwise order from topright cell, as we come from top/left direction (should be clear already).
    - If topright cell has grass over threshold, move there
    - If right cell has grass over threshold, move there
    - If downright cell has grass over threshold, move there
    - If down cell has grass over threshold, move there
    - If downleft cell has grass over threshold, move there
    - If left cell has grass over threshold, move there
    - If topleft cell has grass over threshold, move there
    - If top cell has grass over threshold, move there

5.	Else if: if no sensor input and at corner of wall and map, go right and follow the wall to be able to find areas with higher grass

6.	Else: no sensor input from nearby cell, get sensor data

##  Running the project
The solution is tme290-lawnmower-csaba.cpp
  - First terminal
    - go to project root folder
    - docker build -f Dockerfile.amd64 -t cjakabos/tme290-lawnmower-csaba .

    - docker run -ti --rm --net=host -e COLUMNS=$COLUMNS -e LINES=$LINES -e TERM=$TERM olbender/tme290-grass-simulator-amd64:v0.0.5 tme290-sim-grass --cid=111 --time-limit=0 --dt=0.0 –verbose

  - Second terminal same folder
    - docker-compose -f simulate-robot.yml up


