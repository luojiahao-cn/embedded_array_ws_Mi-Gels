# mi_gels_bringup

`mi_gels_bringup` owns the project-level launch and configuration entry points
for MI-GELS experiments.

The first migration stage keeps runtime node implementations in their existing
packages:

- `zed_wrapper` still owns the ZED2i camera node.
- `tagslam` still owns AprilTag/tagSLAM processing.
- `sensor_data_collection` still owns MagGrad collection and helper scripts.
- `zlab_robots_calibration` still provides lab calibration files.

This package collects the launch/config layer so the experiment flow can move
into MI-GELS gradually without editing upstream dependency defaults.

## Camera profiles

- `config/zed2i_hd1080_30hz.yaml`: default higher-rate profile for tracking.
- `config/zed2i_hd720_60hz.yaml`: faster experimental profile when lower image
  resolution is acceptable.

## Launch files

Start only the camera:

```bash
roslaunch mi_gels_bringup zed2i_camera.launch
```

Start the camera, AprilTag detector, tagSLAM, semantic TCP transforms, and the
frame reprojector:

```bash
roslaunch mi_gels_bringup apriltag_tracking.launch
```

Start MagGrad continuous collection through the new project entry point:

```bash
roslaunch mi_gels_bringup maggrad_collection.launch
```

Use the 60 Hz profile:

```bash
roslaunch mi_gels_bringup apriltag_tracking.launch \
  zed_profile:=$(rospack find mi_gels_bringup)/config/zed2i_hd720_60hz.yaml
```
