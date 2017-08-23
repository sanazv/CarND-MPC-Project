# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Report
The goal of this project was to implement model predictive control to drive the autonomous car successfully around the track. 
I this report I address the points raised in the rubric for the project. I have also included a video of the track here as well. Please feel free to download *MPC_drive.mov* and have a look. The car successfully completes the track without leaving the drivable parts or perform any unsafe behaviour. The velocity can be modified to up to 80 mph and the performance will not be compromised.

#### Model Description:
In MPC we use information about state of the car (position, heading and velocity) to predict what the control inputs (Steering angle and throttle/brake) should be such that the car archives a path which is the closest to the waypoints of the road, i.e. not leaving the track and following the road curvature along the way. The equations of motion below describe the dynamics of the system. In this case we ignore the effects of friction (slip, damping etc), torque etc.
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
where:
```
(x,y): position of the car 
v: velocity
psi: heading
epsi: orientation error
cte: cross track error (distance offset to the side of the road)
Lf: turn rate (physical characteristic of the vehicle)
t, t+1: two consecutive timesteps
and control inputs (with some constraints):
delta: steering angle 
a: acceleration (+ for throttle and - for break)
```
#### Cost-function and parameter tuning: 
A cost function is used with the aid of optimizer to minimize the deviation of the predicted track from the desired waypoints.
The cost function I use, is sensitive to CTE (Cross Track Error), espi (orientation error), offset of velocity from the reference value, steering angle, acceleration, and the change in steering angle and the change in acceleration between consecutive timesteps. The two latter terms are so that the car avoid very strong corrections one way or the other and appear to drive in a "calmer" way.
The weight of these terms are chosen to represent the relative importance of each component. The final cost function is set up as below, with cost function to be most penalizing to sharp changes to streeting in angle and acceleration:

 ```
 CF  = sum_i CTE(i)^2 + epsi(i)^2 + (v(i)-v_ref)^2 + delta(i)^2 + tune_a a(i)^2 + tune_diff_d * [delta(i+1)-delta(i)] + [a(i+1)-a(i)]
 ```

where `tune_a = 10` and `tune_diff_d = 600`.



#### Choice of N and dt:
N(number of timesteps) and dt(length of each timestep) are related to each other via: `T = N x dt`, where `T` is the time horizon for which the forward predictions are being made. I have tried a series of `T` values, and discovered that if `T` is too large, then the 3rd order polynomial fit has to fit a stretch of the road which will not only bend but even appear almost horizontal to the car point of view. This will cause the waypoints to form shapes that are no longer 3rd order polynomials, so the fit would be less accurate. In summary, in order to grasp a piece of the road ahead that can be approximated by a 3rd order polynomial (reach some curvature beyond straight road, but not too curvy), time horizon of 1s is chosen, which has to perform at a range of velocities. I found that time horizon in 1s-1.25s range is a good choice. With that I fixed `T` to 1 sec.
I also chose dt to be 0.1s for convenience, since the latency is also 0.1s, so I can lock the first element of the fitted vector which corresponds to the first 0.1s of the path.
I chose `N = 10`, which is the result of T/dt. That being said I have tried other values such as `N = 20 & dt = 0.1`, `N = 10, dt = 0.05` and many more. 

#### Waypoints preprocessing: 

The waypoints and the car state data are passed from the simulator to the code in global reference. In order to simplify the polynomial fit and also easily update and repeat the prediction after each timestamp, I convert all of these values to a coordinate system which is centered on the car and the x-axis is rotated to be along the direction of the car's heading. This way, `(x,y,psi)` of the car are all 0.

#### Latency implementation:
The drive is supposed to handle a 100ms (0.1s) latency from simulated to replicate the delays in actuator controls in real settings. For this purpose, when the solver finds the parameters which minimize the cost function, the full `N` values of them are kept. The latency index is calculated by `latency_target/dt`, which in this case is `0.1/0.1 = `1. Then the values of acceleration and steering angle at this index are cached and in the next iteration of optimization, the values at this index are kept constant and the optimizer optimizes for the rest. This replicates the fact that the actuator commands to a and delta, will be processed by the car with 0.1s delay.


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
