# Project: PID Control

### Udacity Self-Driving Car Engineer Nanodegree Program

---

The goal of this project are the following:

* Write (in C++) a PID controller that can be used to guide the simulated vehicle in driving a virtual track provided by Udacity [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases).
* Integrate the PID controller into a uWebSocket-based server and verify the effectiveness of the PID controller.
* Tune PID hyperparameters so that the simulated vehicle can drive safely on the virtual track.

[//]: # (Image References)
[40-01]: ./images/40mph-01.png
[40-02]: ./images/40mph-02.png
[40-03]: ./images/40mph-03.png
[90-01]: ./images/90mph-01.png
[90-02]: ./images/90mph-02.png
[90-03]: ./images/90mph-03.png
[cte-40]: ./analysis/40.png
[cte-60]: ./analysis/60.png

## Rubric Points

* The code must faithfully implement the principles of the PID control technique.
* The simulated vehicle must successfully drive the track.

The complete rubric points for this project can be found [here](https://review.udacity.com/#!/rubrics/1972/view).

## Program Build & Execution

In the project root directory, execute (in a shell) the following sequence of commands.

```
# mkdir build
# cd build
# cmake .. && make
# ./pid
```

Besides, run the above mentioned simulator on the same machine, which connects and sends requests at port 4567 of the `localhost`. The program `pid` should listen to this port, connect, and responds to the simulator's requests by sending two json-encoded values: `"steering_angle"` and `"throttle"` that are going to be used by the simulator to control the vehicle.

## Input Data

The request from the simulator contains the following parameters encoded in json format.

* CTE (cross-track error): deviation of the vehicles lateral position from the center of the track
* Speed: current velocity of the vehicle
* Steering angle: current steering angle of the vehicle

The PID controller receives a single input of CTE, and the other two are used for debugging and monitoring purposes.

## Implementation

### PID Controller

The controller is implemented in a single class (`class PID`). It contains PID terms and errors calculated in each step, state variables to keep the error terms, and most importantly, the PID coefficients. They determine the impact of each of the P (proportional), I (integral), and D (Differential) controllers, being set by initialization (using hyperparameters derived in a way that will be described shortly) and never changed while in operation.

The member function `Init()` receives as arguments values for the above mentioned hyperparameters and set them in the respective member variables. These are used to calculate the control value in `GetControlValue()` which returns the control output calculated by a linear combination of P, I, and D errors with coefficients being the respective hyperparameters.

When `UpdateError()` is called, it receives the current CTE (as an argument) and updates each of the P, I, and D error values. The P controller tries to drive the system to the desired state by applying the control value in proportion to the current error. Therefore, the P error is simply defined by the current CTE that captures the system's present state.

On the other hand, the D controller tries to bring the system to a stable state by minimizing the change in the CTE. For this purpose, it applies the control value in proportional to the CTE's first derivative. This way, the D controller aims at driving the system closer to the desired state in the future.

Finally, the I controller tries to compensate for the systematic bias or the error accumulated over a longer period of time. For this purpose, it applies the control value in proportional to the CTE integrated over time. This way, the I controller's role in the whole control system is to make up for the loss in the past.

Unlike the simple one (implemented in Python) in the lesson, the PID controller for this project must take time into account, because the simulator (in contrast to the `class Robot` in the lesson) operates in real time. This means that the interval at which PID errors are updated differs (1) from one request to the next in a single simulation, and (2) for different settings and environments that the simulator is run in. Therefore, in calculating the I and D errors, a variable `dt` is used that captures the elapsed time between the previous update and the current one (relying on the system clock).

By taking the time into account, we are expected to (1) eliminate the impact of small deviations of CTE sampling rate, and moreover (2) apply the same PID controller to different simulating environment. For instance, the simulator executed on a local MacOS system with 1024 x 768 resolution and "Simple" graphics quality sends requests (that triggers the error update) approximately at a 30 ms interval, whereas the simulator on the Udacity VM (with the same settings) sends them approximately at a 70 ms interval. The interval also differs for different resolution and quality settings.

### Vehicle Control

Two instances of the above described PID controller are instantiated and employed in the vehicle control algorithm implemented in `main.cpp`: one for throttle control and the other for steering.

The throttle PID controller is used in a very simple manner. Without any hyperparameter tuning, the algorithm senses the current steering angle (from requests from the simulator) and controls the throttle accordingly. Specifically, only P control is used with the gain set at 0.05, while the other two controls are disabled by setting the respective gains at zero. With the P error being the difference between the vehicle's current velocity and the target speed (both in terms of mph), the control value for the throttle is at the maximum acceleration (1.0) for the vehicle 20 mph slower than target; and conversely at the maximum deceleration (-1.0) for the vehicle 20 mph faster than target, while excess values are trimmed.

With these settings, the target speed is determined according to the current steering angle, since it was observed that the vehicle could not safely drive past a steep corner with high speed. Again, this control is done in a very simple calculation where a deceleration factor of 2.0% for one degree of steering to the maximum speed that is pre-set in the code. In other words, the controller aims at the maximum speed when the vehicle drives straightforward, and decelerate it in proportion to the steering angle.

For the steering PID control, all the three components of the PID controller is enabled by hyperparameters set in a manner that will be described below. Since the PID error is defined in terms of the cross-track error, the controller aims at keeping the vehicle as close as possible to the center of the track. Since the track center is a moving target as the car drives along it, the controller's capability of keeping the center depends on the vehicle's driving speed (also dynamically adjusted). In addition, the controller's performance is also impacted by the granularity of control affecting the system's (vehicle's) behaviour. This issue will be discussed in the following section ("Results").

### Hyperparameter Tuning

The beauty of a PID controller lies in the fact that the same simple framework can be used for virtually all kinds of feedback control systems without the need for knowing the internal structure or mechanism of the target system under control. At the same time, the brutal characteristic of a PID controller is that the control parameters are not systematically derived from system analysis. Rather, the control value depends solely on the observed error (CTE) without taking any other aspect of the system into account. Therefore, the hyperparameters are determined in an empirical manner. A two-step approach was taken to derive the P, I, and D gains (coefficients). First, a reasonably working (testable) combination is sought in a trial-and-error method. Second, fine-tuning was done in a way similar to the twiddle algorithm explained in the lessons. The procedure described here is only applied to one of the two PID controllers (steering control).

In trying to find a reasonable configuration, first the P gain is set largely according to a rule of thumb. With the other two (I & D) controls disabled and the target speed set at a moderately low value, a value that can smoothly steer the vehicle without too much overshoot (on a straight portion of the track) is selected. By gradually increasing the target speed to a certain extent, the simulator gives a "feel" for what could be a candidate around appropriate value of the P gain. Of course, however, since the track is not straight all the time, the P control alone cannot stabilize the vehicle close to the track center.

Next, deriving a reasonable value for the D gain is attempted. Since the D control aims at stabilizing the CTE regardless of its current value, a large D gain results in the control value close to zero even with a large amount of error (overdamping), whereas a small value does not succeed in stabilizing the vehicle before it overshoots (underdamping). Between these two states (overdamped and underdamped), coarse-grain (manual) tweaking was done to derive a tentative value for the D gain.

After the two (P & D) gains are set in a rough manner, the I control is experimented by gradually increasing the I gain in a small magnitude. Here, we expect the I control behaves in a way that it corrects residual (although probably small) error present for a longer period of time even on a (relatively) straight portion of the track. In this experimentation, probably both the P and the D gains need to be adjusted (hopefully slightly) so that the resulting control system becomes more stable as the process repeats.

This manual derivation of a reasonable set of initial values for the three gains may be a tedious (or boring, time-consuming, sometimes even frustrating) process. Nonetheless, tenacious trials (and errors) will pay off at the end, when the vehicle is able to drive at least one lap of the track without coming off of it or heavily treading on an unsafe area, even at a low maximum speed is applied to the throttle control. If more controlled experiment had been possible (e.g. automated repetition of the simulation using the same scenario but with different gain value settings), it would have been much less of a pain to come up with a viable setting, for example applying a heuristic method such as the twiddle algorithm.

Now that we have a set of initial values to tinker with, we perform the second step of the procedure, i.e., fine-tuning the hyperparameters in a more systematic manner. In this project, (again) a manual tuning is done which approximately follows the behaviour of the twiddle algorithm. In other words, beginning from (relatively) large variations in all the three gains, the total error (defined as mean squared error in this project) is measured by increasing and decreasing one gain value while others held constant. The simulation are run for the same (beginnig) portion of the track to compare the resulting MSEs in the same condition.

Note, however, that this measurement does not give the same MSE every time with the same setting, since the simulator presumably involves a certain degree of randomness (at least in terms of its interaction with the server process). Therefore, we do not rely on the accurate and precise measurement of the error. Instead, the twiddling process is rather coarsely, for example discarding a run of simulation when the vehicle goes out of the track or increasing and reducing each hyperparameter's variation amount in a manner not very much consistent. Nevertheless, this fine-tuning is largely expected to enhance the controller's performance although slightly, since we rely on a hard fact (measured MSE) instead of observing the system's behaviour by human eyes which was done in the first phase of the hyperparameter derivation. Of course, the resulting hyperparameters neither guarantee a global minimum of MSE nor give the optimal performance of the system. This is an intrinsic characteristic of the PID control, which nonetheless exhibits good overall performance in practice for a wide range of feedback control systems.

## Results

With the hyperparameters derived according to the method explained in the previous section, a set of simulations were executed on both (1) a local machine running MacOS, and (2) the virtual machine environment provided by Udacity.

![Behaviour of the Simulated Vehicle @ 90 mph #1][90-01]
![Behaviour of the Simulated Vehicle @ 90 mph #2][90-02]
![Behaviour of the Simulated Vehicle @ 90 mph #3][90-03]

The three figures above give screenshots taken while the simulator was run on a local machine, with the maximum speed set at 90 mph. Algthough the throttle controller accelerates the vehicle on a straight portion of the track aiming at the target of 90 mph, on average the actual velocity reaches approximately 70 mph at maximum (as illustrated in the first figure). When the vehicle drives in a curved section, it drops the speed down to about 50 mph (see the second and the third figures). Although the vehicle swerves close to the curb, the controller was able to manage to drive it without any serious incident.

If we lower the maximum speed setting, the controller shows a more stable steering of the car, as shown in the following three screenshots taken at approximately the same locations of the track simulated with the maximum speed at 40 mph.

![Behaviour of the Simulated Vehicle @ 40 mph #1][40-01]
![Behaviour of the Simulated Vehicle @ 40 mph #2][40-02]
![Behaviour of the Simulated Vehicle @ 40 mph #3][40-03]

We can observe that the speedometer indicates lower (roughly half) speeds for all three cases, the vehicle maintains its trajectory closer to the track center. This behaviour is explained by the natural fact that more speed causes larger deviation when the vehicle is steered to either side. The deviation is greater for steeper corner of the track, where the vehicle gets closer to the edge even though it reduces its speed target and declerates accordingly.

Finally, even though no screenshots are provided here, the controller was also applied to simulation on the Udacity-provided virtual machine. As mentioned earlier, this environment is much different from the local environment in terms of the interval between consecutive requests from (and responses to) the simulator and thus the granularity of control. In other words, the interval at which control is applied to the simulated vehicle is greater in this environment than in the previously described set of simulations.

For this reason, the maximum speed that can be employed by the controller (without causing any trouble for the vehicle) is significantly lower. Specifically, we verified that at least one lap of safe driving was achievable for the maximum speed setting of 60 mph, but that the vehicle drives out of the track for a 90-mph setting.


## Discussion

We monitored the CTE for each invocation of the controller by adding a debug output code that emits the value to `stderr` and redirecting the output to a file. The following figure shows the result of plotting the CTE for approximately one lap with the maximum speed setting of 40 mph.

![CTE Plotted over Time @ 40 mph][cte-40]

We observe that the error is between -2 and 2 during the lap. The reason for the error being positive most of the time is that the track bends towards the left hand side most of the time (positive error means the car is right to the center track). The deep valley between the X coordinate 1,500 and 2,000 corresponds to a steep right turn, which is captured in the final screenshots of both sets of three figures in the previous section.

![CTE Plotted over Time @ 60 mph][cte-60]

The above figure shows the same plot for the setting of the maximum speed at 60 mph. Note that the magnitude of the CTE is higher than in the previous case (40-mph setting). However, the trend (captured by the graph's shape) is very similar for both simulation runs. Also note that the scale of X axis is different, since it took a smaller time to complete a lap and therefore a smaller number of requests were made during the simulation.

A final remark is worth making about the controller's performance impacted by logging the CTE. We could not obtain a plot similar to the above two figures with a 90-mph maximum speed setting. This is presumably due to the fact that the server program (implemented in this project) had to spend unnegligible time performing I/O needed to emit a number of strings to `stderr`. This causes degraded responsiveness of the PID controller, and therefore limiting the performance.