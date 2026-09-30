# KELO Tulip

> **Maintenance fork.** The `seal` branch is a maintained fork of
> [kelo-robotics/kelo_tulip](https://github.com/kelo-robotics/kelo_tulip) for
> ROS 2 Jazzy. It tracks the upstream ROS 2 feature branches and adds, on top:
>
> - a `/cmd_vel` watchdog (`cmd_vel_timeout`): stale commands ramp the platform to zero;
> - the joypad no longer drives the wheels; `/cmd_vel` is the only velocity input;
> - configurable `odom_frame` / `base_frame`, and `publish_tf` (default off);
> - the EtherCAT thread really runs with `SCHED_FIFO`;
> - KELOdrive V2 automatic wheel recovery that never re-enables a wheel disabled
>   through `setWheelsEnable`, disables a wheel it gives up on, and uses a
>   monotonic clock;
> - a slave that stops answering for a few seconds is waited for, not retried
>   away: recovery spends no attempt while it is gone, holds every wheel at
>   zero (ramped) and re-enables it once it looks sane; a drive that stays in
>   error, or drops out again and again, is still given up;
> - EtherCAT diagnostics: the ESC error counters per slave and a CSV dump of
>   the last seconds of the 1 kHz loop on a communication error;
> - INIT re-runs the drive start sequence up to 3 times before giving up;
> - `platform_driver` exits (non-zero) once its EtherCAT loop has stopped, so a
>   process supervisor can restart it instead of it idling with dead drives.
>
> Licensing is unchanged from upstream: see [LICENSE](LICENSE).

## EtherCAT diagnostics

`platform_driver` reads every slave's ESC error counters (registers
0x0300-0x0313, read-only) every `ethercat_diagnostics_period_s` (5, 0 = off)
and right after a communication error ends, from a thread of its own. Changes
are logged as `[ecat-diag] slave N: rx_error[p0] +3`, and each read is
published on `~/ethercat_diagnostics` (`diagnostic_msgs/DiagnosticArray`, one
status per slave; `rx_error_pN` etc. are the absolute counters, `delta_*` what
grew since the previous read). A counter that grows on one port points at the
cable segment behind that port. Counters saturate at 255 and restart when a
slave is power-cycled.

A black box keeps the last `blackbox.history_s` (2) seconds of the loop, per
wheel: `sensor_ts`, `voltage_bus`, `current_in`, `status1`, `status2`, the
commanded enable, setpoints and current limits, measured iq, plus the WKC. On a
WKC error, a lost slave or a wheel recovery it writes
`ecat_<date>-<time>_<n>_<reason>.csv` into `blackbox.dir` (default
`$ROS_HOME/kelo_tulip/ethercat_dumps`), `blackbox.post_trigger_s` (0.5) after
the trigger; `t_ms` is relative to the row with `trigger=1`. Dumps are limited
to one per `blackbox.min_interval_s` (10, the first one kept); a wheel giving up
is never suppressed. Disk use is bounded by `blackbox.max_files` (100) and
`blackbox.max_total_mb` (256), oldest files deleted first. `blackbox.enabled:
false` turns it off. The 1 kHz thread only copies into a preallocated ring and
sets atomics; a writer thread does the file I/O.

This package contains the *KELO Tulip* software. This software takes a velocity vector for the overall platform and converts it to commands for the individual KELO Drives of the platform. It implements an EtherCAT master to communicate with the KELO Drives and provides a simple velocity controller that can be used on real robots as well as for simulation.

## KELO Drives and how to build a platform
The [KELO Drive](https://www.kelo-robotics.com/technologies/#kelo-drives) is the patent pending novel drive concept for mobile robots developed by KELO robotics. These wheels can be attached to any rigid platform in order to transform it into a mobile robot. It can even be used to [robotize a container](https://www.kelo-robotics.com/customized-designs/#robotized-material-container). A few screws are enough.

After mechanically connecting the KELO Drives to your platform, you only need to connect them to power via an XT-60 power connector and to your computer via an EtherCAT cable. For multiple wheels, you should either use a Beckhoff Ethernet Switch or one of the KELO power distribution boards. After the wheels are powered and connected to your computer via EtherCAT, the software in this repository will help you make your robot move. 

You can move your mobile platform via a joypad for test purposes or use any software that publishes ROS `geometry_msgs/Twist` messages to the `cmd_vel` topic. See the Interface section below. The software can also be used without ROS. Please contact the developers for that.

## System requirements

This software was tested on Ubuntu 20.04 with ROS Foxy and Ubuntu 22.04 with ROS Humble.
The `seal` branch is built and tested on Ubuntu 24.04 with ROS Jazzy.
For ROS1 version please read the documentation on the master branch.

For the ROS version it is enough to install the base system (for Humble ros-humble-ros-base).


## Installation

The package can be compiled like any ROS package. Clone or copy it into a ROS workspace source folder and run `colcon build` at the ROS workspace root directory.

~~~ sh
cd <WORKSPACE_DIR>/src
git clone -b ros2-develop https://github.com/kelo-robotics/kelo_tulip.git
cd ..
colcon build
~~~

The unit tests (gtest) run with:

~~~ sh
colcon build --packages-select kelo_tulip --cmake-args -DUSE_SETCAP=OFF
colcon test --packages-select kelo_tulip
colcon test-result --verbose
~~~

`-DUSE_SETCAP=OFF` skips the install step that grants the driver its network
capabilities with `sudo setcap`; grant them separately on the robot.

## Usage
### Starting the program

The program can be started by running

```
ros2 launch kelo_tulip example.launch.py
```

or

```
ros2 launch kelo_tulip example_joypad.launch.py
```

### Parameters

The default launch file in `kelo_tulip/launch/example.launch.py` loads the YAML configuration from `config/example.yaml`. Feel free to change parameters directly in this config file, or to make a copy and adjust the launch file to load the new file.

#### Network interface

The following setting defines the network interface used by the driver:

```
device: enp2s0
```

This setting must be adjusted to the network interface by which the KELO drives are connected via EtherCAT.

#### Modules

The following setting defines and configures the modules launched by kelo_tulip. The main module of the package is the `platform_driver`, but the package can also be extended for other devices such as grippers, power management unit, etc.
Another module included in the package is `robile_master_battery`. This module manages the communication between the main cpu with a robile master battery.

```
platform_driver:
  ros__parameters:
    modules:
      list: ["platform_driver"]
      platform_driver:
        type: platform_driver
        controller: velocity_platform_controller
```

A new module can be added to kelo_tulip by adding the module name to the list of modules in TulipMain.cpp.

#### Wheels

The controller needs to know the number of wheels and their location in the body fixed frame of the platform as well as the offset of their pivot encoder (the encoder value when the wheel is oriented forward). This information should be included in the YAML configuration file in the following manner:

```
num_wheels: 4

wheel0:
  ethercat_number: 6
  x: 0.175
  y: 0.1605
  a: 3.14

wheel1:
  ...
```

Please adjust the list of wheels with the correct number and location of the wheels on your platform. `wheel0` refers to the first wheel, starting counting with zero. The `ethercat_number` is the EtherCAT slave number of that wheel. The possible numbers can be seen from when starting kelo_tulip, it will print out information about all slaves found on the EtherCAT bus.

The values `x` and `y` are the coordinates of the wheels center according to the fixed frame of the platform in meters. `a` is the offset of pivot encoder in rad. 

#### Controller Limits

The velocity and acceleration limits of the controller can be set by using these parameters:

```
vlin_max: 1.2
va_max: 1.1
vlin_acc_max: 0.4 
vlin_dec_max: 1.0
va_acc_max: 0.5
va_dec_max: 1.0
```

The explanation of the parameters are as follows: 

- `vlin_max`: maximum linear velocity in m/s
- `va_max`: maximum angular velocity in rad/s
- `vlin_acc_max`: maximum linear acceleration in m/s^2
- `vlin_dec_max`: maximum linear deceleration in m/s^2
- `va_acc_max`: maximum angular acceleration in rad/s^2
- `va_dec_max`: minimum angular acceleration in rad/s^2

### ROS Interfaces

Currently the kelo_tulip software uses ROS as a middleware, subscribing resp. publishing to the following topics.

#### /cmd_vel

This topic accepts [`geometry_msgs/Twist`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html) messages. Any motion software that creates a velocity vector for the platform and publishes `geometry_msgs/Twist` messages to the `cmd_vel` topic can be used. The ROS package [`Nav2`](https://github.com/ros-navigation/navigation2) is an example that conforms to that.

Commands must keep arriving: if no message is received for `cmd_vel_timeout` seconds (parameter, default 0.2, valid range (0, 2.0]; other values are rejected and the default is used), the target velocity is set to zero and the platform ramps down with its configured deceleration limits. Publish at a rate well above `1 / cmd_vel_timeout`.

#### /joy

On the `seal` branch the joypad does not drive the wheels: `/cmd_vel` is the only velocity input, so manual driving goes through whatever node publishes `/cmd_vel` (for example a velocity arbiter that also handles the joypad). `/joy` is only used when `active_by_joypad` is true: holding the button with index 5 (`RB`) then lets the driver switch from READY to ACTIVE once.

#### /odom and /tf

On the topic `/odom` odometry data in form of [`nav_msgs/Odometry`](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html) are published. Each time the program is started, the position is reset to the origin.

With `publish_tf` set to true (default false), the same odometry is also published on `/tf` as the transform `odom_frame` → `base_frame` (defaults `odom` → `base_footprint`). Leave it off when another node, such as a state estimator, owns that transform.

#### /status

On this topic an integer representing status information about the controller is published periodically in form of [`std_msgs::Int32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Int32.html) messages. The single bits of the number have the following meaning:

| Bit       | Description                                                                                                                             |
|-----------------------|-------------------------------------------------------------------------------------------------------------------------------------|
| 0x0001    | Status OK, EtherCAT communication and all drives are working properly.
| 0x0100    | An unspecified error occured.
| 0x0200    | Error detected in EtherCAT communcation because of wrong WKC value. Might be temporary communication loss or power off of one wheel.
| 0x0400    | Timestamp of one KELO Drive did not increase as expected, probably it stopped communicating.
| 0x0800    | One KELO Drive did not have the expected status bits set, probably it got deactivated for some reasons.


### Velocity controller

kelo_tulip includes a simple velocity controller to demonstrate the general principle how to command wheel velocities. It considers velocity and acceleration limits for the platform. Being velocity based it does not allow for the compliant motion of the platform. It is rather meant as a guideline how own controllers can be developed. The controller itself is implemented as a C++ class and does not have any dependencies on ROS. 

The concept of this simple controller is as follows. For each drive a target velocity is computed that depends on the desired velocity for the whole platform and the mounting position of the drive on the platform. The velocity of the individual wheel determines which pivot angle it should have. This pivot angle must be achieved by rotating the wheel around its center. This can be done by moving the left and right hubwheel with the same speed in opposite directions; similar to like a robot with differential drive would rotate around its center. In addition the linear motion of the drive can be achieved by giving the left and right hubwheel the same speed in the same direction. Just overlaying both parts, speeds in opposite direction for pivot-rotation and speeds in same direction for linear motion, will lead to the desired overall motion of the platform.

The crucial part here is that each drive is handled on its own, separately from the others. This makes the whole approach very modular and does not cause any issues when more wheels are added or their mounting locations are changed. A flaw of this simple approach is the tendency to give already a forward motion if the wheels do not point yet into the correct orientation. If different wheels then start to move into different directions, the result on the platform can be very suboptimal. One possible solution is to delay giving forward velocities until all wheels are oriented more or less correctly. This can increase the stability of the motion, but also make maneuvers slower, and for the sake of simplicitiy was not done in the provided example controller.

The controller can be found in the file `src/VelocityPlatformController.cpp`. The main functions are the following:

#### VelocityPlatformController()

The constructor initializes several variables, there are in particular:

- `platform_target_vel_`: A struct with `x`, `y`, and `a` members to set the target values as requested by the external application
- `platform_ramped_vel_`: A struct with `x`, `y`, and `a` members to set velocities that acceleration limits into account
- `platform_limits_`: A struct with minimum and maximal settings for velocity, acceleration and deceleration

#### setPlatformTargetVelocity()

Called to set the desired platform velocity in x, y, and rotational direction.

#### initialise()

This function is called with a configuration setting for each wheel, in particular containing the location of each wheel in the platform's coordinate center.

#### calculatePlatformRampedVelocities()

This function ramps up or down the current velocity setpoints according to the acceleration limits and the platform's target velocities. Each dimension is considered separately from the others.


#### calculateWheelTargetVelocity()

This is the main function that computes the setpoint for the left and right hub wheels of each drive. It must be called once for each drive at each step. The computation consists of the following main steps:

1. Determine the x and y position of each (left and right) hub wheel relative to the platform's center.
2. Calculate the velocity this wheel unit should have at its pivot position, i.e. between the left and right hub wheel.
3. Calculate the target pivot angle based on the wheel velocity.
4. Apply a simple P-controller to minimize the error between measured pivot angle and target pivot angle.
5. Calculate the single hubwheel velocity based on the pivot-controller result and the target velcoity of the drive.

The result is the setpoints for the left and right hubwheel, which can then be sent to the real KELO drive.

















