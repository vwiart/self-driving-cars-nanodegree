# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
# Building & running

The project can be built on the local machine using `cmake` and `make`. The recommended way to build and run is to use the provided docker image.

Building :
```
docker build -f Dockerfile.builder -t $YOUR_BUILDER_IMAGE_TAG
docker run --rm -it \
  --name sdcnd-t3p1-builder \
  -v $(pwd)/src:/app/src:ro \
  -v $(pwd)/CMakeLists.txt:/app/CMakeLists.txt:ro \
  -v $(pwd)/build:/app/build \
  $YOUR_BUILDER_IMAGE_TAG /bin/bash
```
Once in the container, the command `cmake .. && make` can be used to build the project

Running :
```
docker build -f Dockerfile.runner -t $YOUR_RUNNER_IMAGE_TAG .
docker run --rm -it \
    --name sdcnd-t3p1-runner \
    -e HIGHWAY_MAP=/app/data/highway_map.csv \
    -p 4567:4567 \
    -v %cd%/build:/app \
    -v %cd%/data:/app/data \
    $YOUR_RUNNER_IMAGE_TAG /bin/bash
```

Once in the container, the command `./path_planning` can be used to run the project.

# Architecture

The `main.cpp` file contains the initialisation for the project. The main function first load the data from the file located in the environment variable `HIGHWAY_MAP`.

The file initializes then a `SensorFusion` class which is responsible of handling the data coming from the sensors. It contains two public method. `update_state` stores the telemetry data into the state. `get_nearby_vehicles` returns the `NearbyVehicles`; it loops over the vehicles detected by sensors. If they are close to our vehicle, they will be distributed to the `left_lane`, `same_lane` or `right_lane` fields of the structure, allowing the `PathPlanner` to be aware of the vehicle's surroundings.

The `Vehicle` class is then initialized. This file is used to store the vehicle state by parsing the telemetry in the `update_state` method.

The `PathPlanner` class contains the main logic : 
- `update_state` uses the telemetry and the maps to update the internal state and the compound state.
- `drive` acts as a state machine and will be detailed later on.
- `switch_behaviour` is a helper function that allows state transition and logging
- `change_lane` handles the `ChangingLane` behaviour
- `accelerating` handles the `Accelerating` behaviour
- `steady` handles the `Steady` behaviour
- `generate_path` creates a trajectory based on the state.

The main logic is located in the `drive` state machine. The state machine relies on a set of 3 state:
- `Accelerating`: the car is accelerating
- `Steady`: the car is cruising
- `ChangingLane`: the car tries to change lane.

## Accelerating
The `PathPlanner` will check if there is at least 1 car in front of the vehicle, it will switch the behaviour to `ChangingLane`.

The car will then try to accelerate. If the car has reach the `max_speed`, it will switch the behaviour to `Steady`.

## ChanginLane
The `PathPlanner` will check if there room for a take over. If the car is located on the right lane or on the center lane and there is no vehicle on the left of the vehicle, the `PathPlanner` will switch lane to its left.

Alternatively, if the car is located on the left lane or center lane and there is no vehicle on its right, the `PathPlanner` will take over by the right.

If there is no vehicle in front of the vehicle, it means that the vehicle has slowed down too much and is ready to accelerate again. It's possible that the vehicle that we were trying to take over is finally out of reach. In that case, we might want to implement a "reset" switch to switch back to a normal behaviour. 

When the `PathPlanner` takes over, it switches to the `Accelerating` state. The reasonning here is that while taking over, the car may not be at its maximum speed. If it is though, the `Accelerating` state will handle the situation by caping the maximum speed of the vehicle.


## Steady
The car is defaulted to `Steady` at the beginning of the simulation. We need to switch to the `Accelerating` state if the speed is equal to 0.

If there is a car in the front of us, we need to change to the `ChangingLane` behaviour.

## Generating the trajectoy
The trajectory is generated using the `spline` method. It uses the existing previous path to create a smooth trajectory.

It uses a frenet transformation (cf. the `Frenet` class) to generates the points.

