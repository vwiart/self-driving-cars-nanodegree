# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

This project implements a PID controller for the SDCND simulator.

## Usage

1. Compile the project using the provided docker image: `docker build -t vwiart/sdcnd-t2-p1 .`
2. Run the container : `docker run --rm -p 4567:4567 -it vwiart/sdcnd-t2-p1`
3. Launch the simulator

## Reflection

The PID controller consists of 3 components:

- The P (proportional) allows the car to steer in the right direction.
- The D (derivative) allows the car to correct oversteering.
- The I (integral) prevents the car to drift.

The proportional elements have a tendency to "overshoot". If we only take it into consideration, the car oscilates around the trajectory. The derivative is introduced to prevent this effect.

The integral component takes into consideration the systemic bias (eg. the tires are not aligned).

![pid-controller](resources/pid.png)

I've implemented two controllers : the first controller controls the steering, the second controller controls the throttle.

## Parameter Tuning

The parameters have been set by a series of trial and errors. The final parameters are :

- [0.15, 0, 5] for the steering controller
- [0.2, 0.0002, 0.5] for the throttle controller.

In order to choose those parameters, I've considered those condition : 
- The car must stay on track
- The car should avoid oscillating

