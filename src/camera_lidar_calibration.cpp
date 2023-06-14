/*****************************************************                                                  
 * Author: Michael DeFilippo (mikedef@mit.edu), AUV Lab                                              
 *   NAME: Michael DeFilippo                                                                           
 *    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA                                          
 *    FILE: camera_lidar_calibration.cpp                                                               
 *    DATE: 2022-06-12                                                                               
 *    NOTE: Node to calibrate camera and lidar sensors    
 *                                                                                                     
 * This is unreleased BETA code. no permission is granted or                                            
 * implied to use, copy, modify, and distribute this software                                           
 * except by the author(s), or those designated by the author.                                          
************************************************************************/

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>

#include <cv_bridge/cv_bridge.h>  // Change from ROS img msg to CV Mat
#include <image_transport/image_transport.h> // sub/pub of ROS image msgs
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>

#include <sensor_msgs/PointCloud2.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#define __APP_NAME__ "camera_lidar_calibration"

class CameraLidarCal
{
  ros::NodeHandle nh_;

  ros::Subscriber image_intrinsics_sub_;
  ros::Subscriber image_extrinsics_sub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_raw_sub_;
  ros::Subscriber image_point_sub_;
  ros::Subscriber clicked_point_sub_;
  image_transport::Publisher projected_pub_;

  std::vector<cv::Point2f> clicked_image_points_;
  std::vector<cv::Point3f> clicked_pointcloud_points_;
  cv::Size image_size_;
  //image_geometry::PinholeCameraModel cam_model_;
  cv::Vec3d rotational_vector_, translational_vector_;
  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;
  bool calibrated = false;
  cv_bridge::CvImagePtr cv_ptr_;
  
  void ImageClickedPointCb(const geometry_msgs::PointStamped& in_clicked_point)
    {
      clicked_image_points_.push_back(cv::Point2f(in_clicked_point.point.x,
						  in_clicked_point.point.y));
      std::cout << cv::Point2f(in_clicked_point.point.x,
			       in_clicked_point.point.y) << std::endl << std::endl;

      Calibrate();
    }

  void RvizClickedPointCb(const geometry_msgs::PointStamped& in_clicked_point)
    {
      clicked_pointcloud_points_.push_back(cv::Point3f(in_clicked_point.point.x,
						       in_clicked_point.point.y,
						       in_clicked_point.point.z));
      std::cout << cv::Point3f(in_clicked_point.point.x,
			       in_clicked_point.point.y,
			       in_clicked_point.point.z) << std::endl << std::endl;
      Calibrate();
    }

  void Calibrate()
    {
      if(clicked_image_points_.size() > 4 && clicked_pointcloud_points_.size() > 4)
      {
	ROS_INFO("[%s] Time to calibrate", __APP_NAME__);
	cv::solvePnPRansac(clicked_pointcloud_points_,
			   clicked_image_points_,
			   cameraMatrix_,
			   distCoeffs_,
			   rotational_vector_,
			   translational_vector_);
	std::cout << "rvec: " << rotational_vector_ <<
	  "\n tvec: " << translational_vector_ << std::endl;

	calibrated = true;
	
      }
    }

  void ImageCb(const sensor_msgs::ImageConstPtr& in_image)
    {
      // ROS_INFO("[%s] Image Msg", __APP_NAME__);
      //rvec: [0.962355, -0.734223, 1.03141]
      //tvec: [11.1595, -13.9989, 9.39018]

      if (calibrated)
	{
	  // convert image msg to cv image
	  try
	  {
	    cv_ptr_ = cv_bridge::toCvCopy(in_image, sensor_msgs::image_encodings::BGR8);
	  }
	  catch (cv_bridge::Exception& e)
	  {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	  }

	  // Test reprojection of clicked 3d points before full pcl msg
	  
	  std::vector<cv::Point2f> image_points;
	  cv::projectPoints(clicked_pointcloud_points_, rotational_vector_, translational_vector_,
			    cameraMatrix_, distCoeffs_, image_points);
	  for (const auto& p : image_points)
	  {
	    cv::circle(cv_ptr_->image, cv::Point(p.x, p.y),
		       2, cv::Scalar(0,255,0), 5);
	  projected_pub_.publish(cv_ptr_->toImageMsg());
	  }
	}
    }

  void IntrinsicsCb(const sensor_msgs::CameraInfoConstPtr& in_info)
    {
      image_size_.height = in_info->height;
      image_size_.width = in_info->width;

      //cam_model_.fromCameraInfo(info_msg);

      // Work around for const double to double copy issue
      cv::Mat k(3,3,CV_64FC1, (void *) in_info->K.data());
      cameraMatrix_= k.clone();
      cv::Mat d(1,5,CV_64FC1, (void *) in_info->D.data());
      distCoeffs_ = d.clone();
   
      std::cout << "Camera Matrix: " << cameraMatrix_ << std::endl << std::endl;
      std::cout << "Distortion Coefficients: " << distCoeffs_ << std::endl << std::endl;

      image_intrinsics_sub_.shutdown();
      ROS_INFO("[%s] Image Intrinsics set: %i, %i", __APP_NAME__,
	       image_size_.height, image_size_.width);
    }

public:
  CameraLidarCal()
    : it_(nh_)
    {
    }
  ~CameraLidarCal()
    {
    }
  
  void Run()
    {
      ros::NodeHandle private_nh("~");
      std::string image_src, image_info_src, camera_to_lidar_extrinsics;

      // Get Params
      private_nh.getParam("image_src", image_src);
      ROS_INFO("[%s] image_src: %s", __APP_NAME__, image_src.c_str());
      private_nh.getParam("image_info_src", image_info_src);
      ROS_INFO("[%s] image_info_src: %s", __APP_NAME__, image_info_src.c_str());
      private_nh.getParam("camera_to_lidar_extrinsics", camera_to_lidar_extrinsics);
      ROS_INFO("[%s] camera_to_lidar_extrinsics: %s", __APP_NAME__, camera_to_lidar_extrinsics.c_str());

      // Subs
      ROS_INFO("[%s] image_raw_sub_ subscribing to %s", __APP_NAME__, image_src.c_str());
      image_raw_sub_ = it_.subscribe(image_src, 1, &CameraLidarCal::ImageCb, this);
      ROS_INFO("[%s] image_intrinsics_sub_ subscribing to %s", __APP_NAME__, image_info_src.c_str());
      image_intrinsics_sub_ = nh_.subscribe(image_info_src, 1, &CameraLidarCal::IntrinsicsCb, this);
      /*
      ROS_INFO("[%s] image_extrinsics_sub_ subscribing to %s", __APP_NAME__,
	       camera_to_lidar_extrinisics.c_str());
      image_extrinsics_sub_ = nh_.subscribe(camera_to_lidar_extrinsics, 1,
					    &CameraLidarCal::ExtrinsicsCb, this);
      */
      ROS_INFO("[%s] image_point_sub_ subscribing to Image ClickedPoint from ImageView2 %s/screenpoint",
	       __APP_NAME__, image_src.c_str());
      image_point_sub_ = nh_.subscribe(image_src+"/screenpoint", 1,
				       &CameraLidarCal::ImageClickedPointCb, this);
      ROS_INFO("[%s] clicked_point_sub_ subscribing to Pointcloud ClickedPoint from RVIZ /clicked_point"
	       , __APP_NAME__);
      clicked_point_sub_ = nh_.subscribe("/clicked_point", 1,
					 &CameraLidarCal::RvizClickedPointCb, this);
      
      // Pubs
      projected_pub_ = it_.advertise(image_src + "/projected", 100);

      ROS_INFO("[%s] Starting Camera/Lidar cal", __APP_NAME__);
      ros::spin();
      ROS_INFO("[%s] Done", __APP_NAME__);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, __APP_NAME__);

  CameraLidarCal node;

  node.Run();

  return 0;
}
