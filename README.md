# pr2_eih_ros


## register hand kinect
#### to run:
```
roslaunch kinect_transform ar_hand.launch
roslaunch kinect_transform carmine_registration.launch
rosrun kinect_transform kinect_transform
```

## kinfu
#### to run:
```
roslaunch kinect_transform wrist_to_hand.launch
roslaunch pcl_utils kinfu.launch
roslaunch pcl_utils occlusion_parameters.launch
rosrun pcl_utils kinfu
rostopic pub /get_occlusions std_msgs/Empty -1 [this will make kinfu download its data and perform occlusion finding once]
```
#### publishes:
```
/kinfu/camera_points (current [hand] camera points)
/kinfu/kinfu_points (zero-crossing points downloaded from kinfu)
/kinfu/objects (visualization markers - faces and gaussians)
/kinfu/occluded_points (clusters and points behind them)
/kinfu/occluded_region_array (faces and gaussians)
```
to be added:
```
/kinfu/graspable_points [all points above table]
/kinfu/plane_bounding_box
```

## handle detector
#### input:
    /kinfu/graspable_points
#### to run:
```
roslaunch handle_detector kinfu_handle_detector
rosrun handle_poses_publisher
```
#### publishes:
    /handle_detector/avg_handle_poses

## bsp:
#### outline:
> 1. get ___ closest regions from kinfu
> 2. initialize BSP optimization trajectory
> 3. run bsp
> 4. execute controls
> 5. if no good handles to grasp:
>     6. go to 1
> 7. else:
>     8. trajectory <- /check_handle_grasps/trajectory [on callback]
>     9. execute grasp
>         a. move in
>         b. close gripper
> 	c. move up
> 	d. move back
> 	e. drop off

#### to run:
(TBD)

## grasping:
#### input:
```
/handle_detector/avg_handle_poses
/kinfu/graspable_points
/kinfu/plane_bounding_box
```
#### outline:
> 1. convex decomposition of pointcloud - add table from table pose
> 2. for each handle pose:
>     1. call trajopt (with collisions)
>     2. if no collision && reaches handle:
>     3. publish pose trajectory [/check_handle_grasps/trajectory]

#### publishes:
    /check_handle_grasps/trajectory

