# Prerequisites

To be updated soon
cmake
libcluon
UDP multicast
curl
Docker




## Overview
The project assignment was to design a robot behavior for the robot car (both physical and virtual version) to drive around a track, without hitting any cones or other cars (by keeping the distance relatively constant behind a slower vehicle) and keeping right-hand rule in an intersection.

### Perception
Due to the performance limit of the raspberry pi, it was decided to use basic image processing methods, instead of machine learning algorithms. The basic idea behind the perception algorithm is to identify the cones, fit a curve on each cone line, remove the visual information outside of them and identify other car (masked with green rectangle to remove red/black information from the screen) or crossing scenario in the remaining image (red cones). If crossing scenario
is identintified, the full image without white mask is checked for another car, to reduce risk of collision due to undetected car.

#### Track detection
A near and far point are used in order to describe the track in front of the kiwi car. These two points are arrived at through a number of steps, finding the cones on the track, fitting curves to the detected cones and placing the points given the curves.
