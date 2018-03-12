# Project Discuss

## Describes the model in detail. This includes the state, actuators and update equations.

There are 6 state in my MPC: px, py, psi, v, cte, epsi. The py, py is the global position(map cordinate) of the vehicle. the psi is the heading direction of the vehicle. The v stand for voilocity. The cte is the cross track error and epsi is the error of heading direction.

There are 2 actuators in my MPC: delta, a. The delta is the steering angle of the vehicle and a is the acceleration of the vehicle.

This is how the update equations look like:
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## Discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values.
I use 10 for N (timestep length) and 0.1 for dt (elapsed duration between timesteps). These values mean that the optimizer is considering a one-second duration in which to determine a corrective trajectory. 

##  Described preprocesses of the waypoints
I convert the waypoints to the viechle coordinate, so that it is easier to compute the reslult and plot the waypoits,
This is how i convert the waypoints:
```
for (int i = 0; i < ptsx.size(); i++) {
      double dx = ptsx[i] - px;
      double dy = ptsy[i] - py;
      x_way.push_back(dx * cos(-psi) - dy * sin(-psi));
      y_way.push_back(dx * sin(-psi) + dy * cos(-psi));
}

```
## how to deal with latency.
I use the car's current state and predict the new state with the latency time period. Then I pass the new state into the MPC. This is the code that I impliment to counteract the latency:
```
vector<double> ptsx = j[1]["ptsx"];
vector<double> ptsy = j[1]["ptsy"];
double px = j[1]["x"];
double py = j[1]["y"];
double psi = j[1]["psi"];
double v = j[1]["speed"];
double delta = j[1]["steering_angle"];
double acceleration = j[1]["throttle"];

//taken the latency into account
double latency = 0.1; 
x = x + v*cos(psi)*latency;
y = y + v*sin(psi)*latency;
psi = psi + v*delta/Lf*latency;
v = v + acceleration*latency;
```
