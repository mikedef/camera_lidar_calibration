# camera_lidar_calibration
ROS package for an Online Camera-Lidar calibrarion tool using RVIZ, ImageView2, and OpenCV Perspective-n-Point (PnP) optimization. To find image (2D) to pointcloud (3D) correspondences, select points in the image viewer and points in RVIZ that correspond to one another. These points will be injested by the PnP algorithm to give a translation and rotation matrix between the two sensors. 

## How-To
- Pause bagfile playback when a known object to passes in front of camera, i.e. sailboat or calibration board.
- In RVIZ zoom into object's pointcloud return.
  - Helps to select Views->FPS
- Select 2D/3D points clock-wise in for each.
  - Start with 2D image and select lower left corner of object in image_view2 window. 
  - Next select same point on 3D pointcloud in RVIZ window. Select `Publish Point` tool and click on similar point as the 2D selection.
  - Repeat process in a clock-wise rotation until at least 5 points are selected on each. 

### TODO
- [x] Sub to camera (camera/image_raw) topic
- [x] Sub to camera_info (camera/image_raw/info) topic for camera intrinsic data
- [] Sub to camera-lidar TF for extrinsic data
- [x] Sub to lidar (pointcloud2) topic
- [x] Launch Image Viewer (image_view2) for Image clicked points (correspondences)
- [x] Sub to clicked image points
- [x] Launch RVIZ for Pointcloud clicked points (correspondences)
- [x] Sub to clicked pointcloud points
- [] Write rvec and tvec to file
- [x] Test with filtered camera view pointcloud
  - [x] lidar pointcloud
  - [x] radar pointcloud
  - Works but is very slow. Need to sub to clusters or polygon of objects
  - [] remove overlay function, pc sub and image sub and move to fusion app
- [] Create fusion application
  - [] Sub to camera
  - [] Read in rvec and tvec from file
  - [] Sub to PC cluster msg and reproject centroid to image?
    - [] Check if centroid is within object detection bounding box


#### Dependencies
- ros-noetic-image-view2
