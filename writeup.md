# Project Discuss

## Describes the model in detail. This includes the state, actuators and update equations.

There are 6 state in my MPC: px, py, psi, v, cte, epsi. The py, py is the global position(map cordinate) of the vechile. the psi is the heading direction of the vechile. The v stand for voilocity. The cte is the cross track error and epsi is the error of heading direction.

There are 2 actuators in my MPC: delta, a. The delta is the steering angle of the vechile and a is the acceleration of the vechile.

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
I using the folowing approch to chosen N (timestep length) and dt (elapsed duration between timesteps) values:
First I set the 

## how to deal with latency.
I use the car's current state and predict the new state in the latency time perious. Then I pass the new state into the MPC. This is the code that I impliment to counteract the latency:
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
