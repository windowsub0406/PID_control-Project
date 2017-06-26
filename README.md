# PID Controller Project

<p align="center">
    <img src="./image/pid_control_2.gif" width="480" alt="main_image" /><br>
    <b>result</b><br>
</p>

## Reflection

### PID Componets

The `P` or `Proportional` component indicates a steering angle that is proportional to the cross-track error (CTE). If we use only P component, it cause overshooting problem. continuous oscillations.
  
The `I` or `Integral` component can compensate the bias that the P component can not handle.
  
The `D` or `Differential` component helps reduce overshoot and helps keep the car in the center.

### P, I, D value Tuning

Hyperparameters (P, I, D values) were decided by manual testing. Setting order was P->D->I and the result is as below.    

```
double PID_p = 0.11;  // proportional coefficient
double PID_i = 0.0002;  // integral coefficient
double PID_d = 2;  // differential coefficient
```


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

