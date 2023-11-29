# Volksbot Driver

Revised version of volksbot_driver. Works with ROS-noetic.

## Dependencies

[epos2_motor_controller](https://github.com/uos/epos2_motor_controller)


## Old README:

Volksbot Driver
===============

ROS driver for the [volksbot robots](https://www.volksbot.de/). 

## Installation
  `sudo apt install ros-melodic-volksbot-driver`
  
## Volksbot URDF
See the unified robot description file in `urdf/volksbot.urdf.xacro` and the `urdf/example.urdf.xacro` file to use the 
volksbot base for your needs.

## Parameters
| parameter            | data type  | default value |
|----------------------|------------|---------------|  
| `wheel_radius`       | `double`   | `0.0985`      |
| `axis_length`        | `double`   | `0.41`        |
| `gear_ratio`         | `int`      | `74`          | 
| `turning_adaptation` | `double`   | `0.95`        |
| `x_stddev`           | `double`   | `0.002`       |
| `rotation_stddev`    | `double`   | `0.017`       |
| `cov_xy`             | `double`   | `value`       |
| `cov_xrotation`      | `double`   | `0.0`         |
| `cov_yrotation`      | `double`   | `0.0`         |
| `publish_tf`         | `bool`     | `False`       |
| `num_wheels`         | `int`      | `6`           |
| `joint_names`        | `str list` |               |

