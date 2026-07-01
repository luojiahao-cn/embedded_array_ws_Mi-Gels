# mi_gels_motion

Configurable MoveIt-based motion trajectories for MI-GELS experiments.

The package is intentionally independent from `triple_arm_visual_servo`: it uses
MoveIt directly, reads a YAML motion config, generates Cartesian waypoints, and
uses MoveIt to plan and execute Cartesian motion.

## Run

Run from zlab:

```bash
ssh zlab
cd ~/embedded_array_ws_Mi-Gels
source tools/zlab_ros_env.sh
```

Start Diana7 MoveIt simulation through `zlab_robots_bringup`, wait for
`/move_group`, then execute the default MI-GELS line motion:

```bash
roslaunch mi_gels_motion motion_executor.launch
```

Plan but do not execute:

```bash
roslaunch mi_gels_motion motion_executor.launch plan_only:=true
```

Connect to the real diana7 bringup instead of simulation:

```bash
roslaunch mi_gels_motion motion_executor.launch sim:=false plan_only:=true
```

Add a `DisplayTrajectory` display in RViz and subscribe it to:

```text
/mi_gels_motion/display_trajectory
```

Use another config:

```bash
roslaunch mi_gels_motion motion_executor.launch \
  config_file:=$(rospack find mi_gels_motion)/config/diana7_patterns.yaml
```

## Config

`start.mode` can be:

- `current`: read the current `frame_id -> ee_link` pose from TF, with MoveIt current pose as fallback.
- `absolute`: use `start.pose` from the YAML file.

Supported `trajectory.type` values:

- `line`
- `s_curve`
- `figure8`
- `spiral`

Motion speed has separate controls:

- `positioning_velocity_scaling` and `positioning_acceleration_scaling`: joint-space speed used to move to the configured start pose.
- `trajectory_velocity_scaling` and `trajectory_acceleration_scaling`: MoveIt scaling used for the experiment trajectory.
- `trajectory_tcp_speed_mps`: stretches the experiment trajectory to avoid exceeding the requested approximate TCP speed.
