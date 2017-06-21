# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
This project is for the model predction control lesson in Udacity. In this project I have to finish the code for controlling a car in a [simulated environment](https://github.com/udacity/self-driving-car-sim/releases). See the report section for some details regarding some answers to given questions in the rubic.

## Dependencies

* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
* Fortran Compiler
  * Linux: `sudo apt-get install gfortran`.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Linux
    * You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
* [CppAD](https://www.coin-or.org/CppAD/)
  * Linux `sudo apt-get install cppad` or equivalent.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Report

### Model Details

I use a kinematic model for the car, the states are the position (*x*,*y*), the speed of the car *v*, and the direction of the car $\psi$. The actions applied to the car are the steering angle of the car $\delta$ and the accelaration of the car *a*. The kinematic functions regarding these variable are given as:

$$x_{t+1}​​=x_t ​​+ v_t​​\cos(\psi_t​​){dt}$$
$$y​_{t+1}​​​=y_t​​+v​_t​​\sin(\psi_t​​​​){dt}$$
$$\psi_{t+1}=\psi_t​​​​​​+​{L​_f}{​​​​v_​t}\delta{dt}$$
$$v​_{t+1}​​=v​_t​​+{a_​t}​​{dt}$$
```
where $L​_f$ is the distance between the front of the vehicle and its center of gravity.

### Timestep Length and Elapsed Duration

1. The chosen N * dt should not be too small. It must at least cover the main dynamic process of the agent. On the other hand, N * dt should not be too large, or the prediction will be non-sense. Therefore, I choose 1s for N*dt.
2. In addition, the dt should not be too small, or the agent cannot respond to the commands so frequently. On the other hand, dt should not be too large, or there may be precision loss. I choose 0.1s for dt.
3. I have tried (N,dt) pairs such as (20,0.05) (10,0.05) (20,0.1) (10,0.2)...

### Waypoints Preprocessing

The way points cannot be used directly. This really confused me a lot at the beginning of the project. One has to convert the way point from the global coordinate to the car coordinate. This is because the *etc* is calculated by $etc = y_interpolated - y_predicted$. This is not exactly the perpendicular distance from the predicted point to the interpolated way line. We can use $etc = y_interpolated - y_predicted$ only when *etc* can be seen as an approximation of the perpendicular distance. If we do not convert the waypoints to the car coordinate, the approximation may not hold at all. The convertion is given as follows:

```
          for (int i = 0; i < ptsx.size(); i++ ) {
            double dx = ptsX[i] - px;
            double dy = ptsY[i] - py;
            ptsx[i] = dx * cos(-psi) - dy * sin(-psi);
            ptsy[i] = dx * sin(-psi) + dy * cos(-psi);
          }
```

### Latency Problem

I deal with latency by assuming the car's actual position is 'latency' times ahead. The car's actual state when receiving the command can be obtained by using the state updating equations as follows:

```
          double latency = 0.1;
          px = v*latency;
          py = 0;
          psi = -v*delta/Lf*latency;
          v = v + acceleration*latency;
```

### Parameter Tuning

In this project, only three weights are different from the starter code given by the Udacity quiz. The first is the weight on the value gap between sequential steering actions. If this value is too small, the car would occilate greately, perticularly when a turn is reached. It should alos not be too large, or the car cannot perform steering. The second weight is for the direction (It is a state member). If this value is too small, the car would occilate greately. The third weight is for *etc*. This value shouls not be too small, or the car would deviate from the line. Conversly, he car would occilate greately. My settings are given as follows. With this set of setting, the car can run at a speed of 93mph without crashing.

```
    // Reference State Cost
    for (uint t = 0; t < N; t++) {
      fg[0] += 4*CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 1000*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (uint t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (uint t = 0; t < N - 2; t++) {
      fg[0] += 10000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```

### Recorded video

In the previous submittion, it seems that the performance of the car is different between running on the reviewer's device and on my device. Anyway, the suggestions given by the reviewer is really pertinent. So, I have carefully revised my code according to his suggestions. In addition, I have recorded the simulation results in this submittion and posted it on Youtube. Click [this address](https://youtu.be/svCiJ1t0-Ls) to see it.

<iframe width="560" height="315" src="https://www.youtube.com/embed/svCiJ1t0-Ls" frameborder="0" allowfullscreen></iframe>



