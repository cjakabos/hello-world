# opendlv-logic-pedal-control

Microservice for controlling the speed of the Kiwi car using the current steering-angle.
The input to the microservice is the steering angle and a speed-limit.
The output is the pedal position.

Build with:

docker build -f Dockerfile.amd64 -t opendlv-logic-pedal-control .

