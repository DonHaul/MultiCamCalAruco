## Multi Camera Calibration via Arucos

**Keywords:** Camera networks, 3D reconstruction, RGB-D data, Multi Cloud Registration

### Description:
This repository was built during my Msc Thesis, and it contains part of the code develop that it make possible to register a set of 3D camera, by using arucos.

In this repository there are two main functionalities:

### 1. Obtain the Aruco Calibration Object:
 1. Run `python PosePipelineMaker.py Pipelines/cangalho_realsense.json` in the passed file, there is information relative to the topic to capture the capturing mode and the ids of the present arucos.
 2. Type an uppercase `R` to first calculate the rotations
 3. Type an uppercase `T` to calculate the translations
 4. The poses generated will be save in a file.
 
 ### 2. Obtain the Poses between camerast:
 1. Run `python PosePipelineMaker.py Pipelines/realsense_regular.json` in the passed file, there is information relative to the topics to capture the capturing mode, the calibration object model and its ids.
 2. Type an uppercase `R` to first calculate the rotations
 3. Type an uppercase `T` to calculate the translations
 4. The poses generated will be save in a file.

<br>

On top of these, the present other functionalities:

 ### Broadcasting the camera poses to ROS via `\tf`:
 Run `tfbroadcasterv2.py POSES_FILE` where poses file is the file containing the camera poses retrieved from before. Opening `rviz` it is possible to see the camera poses in 3D space.

Run `boxcaster.py` and in another terminal `rospccropper.py CAMERA_NAME`, to crop a given pointcloud to only contain points within a box.

Run `IntrinsicFetcher.py` to obtain the intrinsic parameters of all the existing cameras
