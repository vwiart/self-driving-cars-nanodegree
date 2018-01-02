# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Reflections

The state is defined by :
- The current position of the car (x, y)
- The orientation of the car (psi)
- The velocity (v)

The state is updated with the following formulas: 
- `x_t1 = x_t + v_t * cos(psi_t) * dt`
- `y_t1 = y_t + v_t * sin(psi_t) * dt`
- `psi_t1 = psi_t + v_t / Lf * delta_t * dt`
- `v_t1 = vt + a_t * dt`
- `cte_t1 = f(x_t) - y_t + v_t * sin(epsi_t) * dt`
- `epsi_t1 = psi_t - psi des_t + v_t / Lf * delta_t * dt`

The following constraints are applied :
- Throttle is limited between [-1; 1] -> MPC.cpp L141-142
- Steering is limited between [-25; 25] -> MPC.cpp L137-138

Those N and dt variables have been initially set respectively to 10 and 0.1. Those seems adequate with regards to the vehicle speed and the circuit. I've then increase the N to 20 which seems to give better results. This should cover a bit more than 2s of horizon which seems more than enough.

The variables of the cost function have been chosen manually by trial and error in order to prevent the car from oscillating (the main issue I had during the tuning)

A 100ms latency has been added (main.cpp L171) to simulate a latency of the real world. In order to deal with this latency, I took the previous state and estimated the new state 100ms later and feed it to the model (main.cpp L112-114)