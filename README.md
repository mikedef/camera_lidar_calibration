# camera_lidar_calibration

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
- [] Sub to lidar (pointcloud2) topic
- [x] Launch Image Viewer (image_view2) for Image clicked points (correspondences)
- [x] Sub to clicked image points
- [x] Launch RVIZ for Pointcloud clicked points (correspondences)
- [x] Sub to clicked pointcloud points
- [] Write rvec and tvec to file
- [] Test with filtered camera view pointcloud


#### Dependencies
- ros-noetic-image-view2
