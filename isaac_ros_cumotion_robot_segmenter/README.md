# isaac_ros_cumotion_robot_segmenter — fork with local extensions

Fork of the NVIDIA package (v4.5.0) with three local extensions to the
`robot_mask` output: polarity inversion, silhouette dilation and morphological
closing. Implementation: `src/robot_segmenter.cpp`,
`include/isaac_ros_cumotion_robot_segmenter/robot_segmenter.hpp`.

## Running

The node is a component — a container named `manipulator_container` must be
running first (the launch file only loads the component into it):

```bash
# terminal 1 — container
ros2 run rclcpp_components component_container \
    --ros-args -r __node:=manipulator_container

# terminal 2 — segmenter (example with the package test data)
ros2 launch isaac_ros_cumotion_robot_segmenter robot_segmenter.launch.py \
    robot_segmenter.urdf_path:=$(ros2 pkg prefix --share isaac_ros_cumotion_robot_segmenter)/test/test_data/robot.urdf \
    robot_segmenter.xrdf_path:=$(ros2 pkg prefix --share isaac_ros_cumotion_robot_segmenter)/test/test_data/robot.xrdf
```

Environment: after `colcon build`, source the **new** install
(`/ws/install/setup.bash`) before any older dev-package and **restart the
container** — a running container keeps the previous component library in
memory and new parameters will not show up.

The mask is published on `robot_mask` (by default
`/cumotion/camera_1/robot_mask`, `16UC1`/`32FC1` matching the depth encoding).

## Added parameters

All three have defaults in `params/robot_segmentation_params.yaml` and can be
overridden as launch arguments with the `robot_segmenter.` prefix, e.g.
`robot_segmenter.invert_robot_mask:=true`.

| Parameter | Type | Default | Description |
| `invert_robot_mask` | bool | `False` | Inverts mask polarity. Raw: robot=0, background=max (65535 for 16UC1 / 1.0 for 32FC1). Inverted: robot=max, background=0. |
| `robot_mask_dilation_radius` | int (px) | `0` | Dilates the robot silhouette with an elliptical kernel of `2r+1`; implemented as `cv::erode` because the robot is the dark (minimum-valued) foreground in the raw mask. `0` = disabled. |
| `robot_mask_closing_radius` | int (px) | `10` | Morphological closing of the silhouette (fills holes up to the radius), implemented as `cv::morphologyEx(..., MORPH_OPEN, ...)` because the robot is the dark foreground. **Enabled by default**; `0` = disabled; negative value → warning and fallback to 10. |

Processing order: `segmentation → dilation → closing → inversion → publish`.
Inversion runs last, so dilation and closing behave identically for both
output polarities.

## Performance

The OpenCV operations run on CPU (loop: depth copy from GPU, segmentation,
post-processing, result copy back to GPU). Measured on the target workstation:
closing with `r=20` lowers the mask topic rate from approx. **15 FPS to
approx. 11 FPS**. Dilation costs about the same as closing at the same radius;
with `robot_mask_closing_radius:=0` and `robot_mask_dilation_radius:=0` the
overhead disappears (code path identical to the original package).

## Building

```bash
colcon build --packages-up-to isaac_ros_cumotion_robot_segmenter
source /ws/install/setup.bash
```

Requires OpenCV (`core` and `imgproc` components, added in `CMakeLists.txt`
and `package.xml`) plus the original package dependencies (cuMotion SDK via
`isaac_ros_nitros`, NitROS, tf2, etc.).
