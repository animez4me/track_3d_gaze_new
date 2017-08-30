
#include <fstream>  // NOLINT
#include <iomanip>
#include <iostream>  //NOLINT
#include <list>
#include <vector>
#include <map>
#include <stdexcept>
#include <chrono>

#include <brisk/brisk.h>
#include <brisk/brute-force-matcher.h>
#include <brisk/command-line-parser.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Kinect
//#include "libfreenect2/k2g.h"
//#include <pcl/visualization/cloud_viewer.h>

#include "RGB.h"
#include "RGBD.h"
#include "matcher.h"

//#include "System.h"
//#include "ETG.h"
//#include "OptiTrack.h"
#include "Utils.h"
#include "EyeTracking_2D3D.h"

//#include "track.h"
#include <pcl_conversions/pcl_conversions.h>
//save compiler switches
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//restore compiler switches
#pragma GCC diagnostic pop

// 0 = use fixation time, 1 = click on stream
#define TEST_FIXATION 0
// 0 = compute pose after click, 1 = continously estimate pose
#define POSE_CONTINIOUS 1

float fixation_speed_thres = 10;
float fixation_duration_thres = 2;

bool show_kinect = false;
bool show_mesh = false;
bool show_matches = false;
bool triangulate = false; //should be true in the runtime
bool kalman_filter = false;

//cv::Point3f fixed_pt;
std::string RGBD_CAMERA_intrinsics = "/home/mike/catkin_ws/src/track_3d_gaze/calib_color.yaml";
std::string RGBD_IR_intrinsics = "/home/mike/catkin_ws/src/track_3d_gaze/calib_ir.yaml";
std::string RGB_CAMERA_intrinsics = "/home/mike/catkin_ws/src/track_3d_gaze/calib_RGB_ETG.xml";
std::string KINECT_CLOUD_TYPE = "/kinect2/qhd/points";
std::string KINECT_COLOUR_TYPE = "/kinect2/hd/image_color_rect";

int counter = 0;
cv::Point2f testpoint = cv::Point2f(0,0);
bool new_click = false;

cv::Mat frame, depthframe;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

unsigned int gCurrentFrameNumber = 0;
unsigned int gLastFrameNumber = 1;
unsigned int fix_start = 0;
unsigned int fix_duration;


void PointsSelectionCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        testpoint = cv::Point2f(x, y);
        new_click=true;
    }
}


//Converts ROS Images from RGB and Depth to OpenCV Images
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_color_sub_;
  image_transport::Subscriber image_depth_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_color_sub_ = it_.subscribe(KINECT_COLOUR_TYPE, 1,
                                     &ImageConverter::imageColorCb, this);
    image_depth_sub_ = it_.subscribe("/kinect2/bigdepth", 1,
                                     &ImageConverter::imageDepthCb, this);
    //image_pub_ = it_.advertise("/image_converter/output_video", 1);

  }

  ~ImageConverter()
  {

  }

  cv_bridge::CvImagePtr cv_color_ptr;
  void imageColorCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
//      ROS_INFO_STREAM("hee");
      cv_color_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      frame = cv_color_ptr->image;
      //return cv_color_ptr->image;

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

    cv_bridge::CvImagePtr cv_depth_ptr;
    void imageDepthCb(const sensor_msgs::ImageConstPtr& msg)
    {
      try
      {
        cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
        depthframe = cv_depth_ptr->image;
//        std::cout << depthframe << std::endl << std::endl << std::endl;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
};


//Converts from PointCloud2 to PCL cloud
void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr msg)
{
//  ROS_INFO("GOT PT CLOUD");
  sensor_msgs::PointCloud2 sensor_cloud_msg_ = sensor_msgs::PointCloud2(*msg);

  if(sensor_cloud_msg_.data.size() == 0)
  {
    ROS_ERROR_STREAM("Cloud message is invalid, returning detection failure");
    //exit(1);
  }

  pcl::fromROSMsg (sensor_cloud_msg_, *cloud);
}


int main(int argc, char **argv)
{

  //Initialising ROS node, publisher and subscriber to Kinect point cloud
  ros::init(argc, argv, "track_3D_gaze_node");
  ros::NodeHandle n;
  ros::Publisher fixation_pub = n.advertise<geometry_msgs::Point>("fixation_3d", 1000);
  ros::Subscriber kinect_cloud_subscriber = n.subscribe(KINECT_CLOUD_TYPE, 1, &point_cloud_callback);
  ros::Duration(1).sleep();
  ros::spinOnce();
  static tf::TransformBroadcaster pose_broadcaster, fixation_broadcaster;
//  image_color_sub_ = it_.subscribe("/kinect2/qhd/image_color_rect",1,imageColorCb);


    RGBDcamera kinect(RGBD_CAMERA_intrinsics, RGBD_IR_intrinsics, cloud);
    RGBcamera camera(RGB_CAMERA_intrinsics);
    matcher briskf;
    EyeTracking3D et3d(camera.camMatrix, camera.distCoeffs);

// Initialize eye-tracking data read
        //std::vector<EyeTracker_Fixations> ET_Fixations;
        std::vector<EyeTracker_PoR> ET_PoRs;
        int last_activation = 0;
        // Instantiate Eye-tracking ******************************************************
        int fixation_idx = 0;
        bool fixation_active = false;
        float current_time = 0;
        int counter = 0; // frame counter
        float fps_SMIET = 24;

    std::cout << "On \"Viewer\": \t Green is for Kinect camera \t Purple is for EyeTracker's camera \t Red is for 3D fixation" << std::endl;
    std::cout << "Click on \"EyeTracker\" window to activate 3D fixation localisation" << std::endl << std::endl;
//    sleep(2);
 ImageConverter ic;
 //ros::Subscriber kinect_cloud_subscriber = n.subscribe(KINECT_CLOUD_TYPE, 1, &point_cloud_callback);
 //ros::Duration(0.3).sleep();
    for(;;) {

  //Calling callbacks to retrieve RGB frame and kinect cloud


  ros::spinOnce();

//*

  cv::Mat kinect_RGBFrame = frame;
  cv::flip(depthframe, kinect.DepthFrame, 1);

    //kinect.k2g.get(kinect.RGBFrame, kinect.DepthFrame, kinect.cloud);
    camera.captureFrame();
//    cv::cvtColor(kinect.RGBFrame, briskf.imgGray1, CV_BGR2GRAY);
//std::cout << kinect_RGBFrame.empty() << camera.RGBFrame.empty() << std::endl;
  if (!(kinect_RGBFrame.empty() || camera.RGBFrame.empty()))
  {
//    cv::namedWindow("kinect_RGBFrame");
//    cv::setMouseCallback("kinect_RGBFrame", PointsSelectionCallBackFunc, NULL);
//    cv::imshow("kinect_RGBFrame", kinect_RGBFrame);

//    cv::Mat bigdepth;
//    double min2;
//    double max2;
//    kinect.DepthFrame.convertTo(bigdepth, CV_16UC1);
//    cv::minMaxIdx(bigdepth, &min2, &max2);
//    cv::Mat adjMap2;
//    bigdepth.convertTo(adjMap2, CV_8UC1, 255 / (max2-min2), -min2);
////    std::cout << bigdepth << std::endl; break;
//    cv::Mat falseColorsMap2;
//    applyColorMap(adjMap2, falseColorsMap2, cv::COLORMAP_RAINBOW);
//    cv::namedWindow("Out2");
//    cv::circle(falseColorsMap2, cv::Point2f(int(testpoint.x + 0.5) + 1, int(testpoint.y + 0.5)), 3, cv::Scalar(255,255,255));
//    cv::imshow("Out2", falseColorsMap2);
//    char key = cvWaitKey(1);
//    if (key == 27) // ESC
//        break;

//    if (new_click)
//    {
//      cv::Point3f point3d_rgbd;
//        bool a = kinect.getDepthByColor(testpoint, point3d_rgbd);
//        printf("[%d,%d:%f,%f,%f]\n", int(testpoint.x), int(testpoint.y), point3d_rgbd.x, point3d_rgbd.y, point3d_rgbd.z);
//        new_click=false;
//    }
    EyeTracker_PoR currentPOR;
#ifdef EYETRACKERS
    
    gLastFrameNumber = camera.gazedata[0];
    currentPOR.frame = camera.gazedata[0];
    currentPOR.PoR = cv::Point2f(camera.gazedata[1], camera.gazedata[2]);
    currentPOR.speed = 0;
    currentPOR.is_saccade = true;

    if (ET_PoRs.size() > 0)
    {
      currentPOR.speed = sqrt(pow(currentPOR.PoR.x - ET_PoRs.back().PoR.x, 2) + pow(currentPOR.PoR.y - ET_PoRs.back().PoR.y, 2)) / sqrt(pow(currentPOR.frame - ET_PoRs.back().frame, 2));
//      std::cout << "speed " << currentPOR.speed << std::endl;
      if (!(currentPOR.speed > fixation_speed_thres))
      {
        currentPOR.is_saccade = false;

        if (ET_PoRs.back().is_saccade)
          fix_start = currentPOR.frame;

        fix_duration = currentPOR.frame - fix_start;
        //std::cout << "fix_duration " << fix_duration << std::endl;
        //std::cout << "x y1  " << currentPOR.PoR.x << "," << currentPOR.PoR.y << std::endl;
        //std::cout << "x y2  " << gGazeX << "," << gGazeY << std::endl;
        if (fix_duration / fps_SMIET > fixation_duration_thres && currentPOR.PoR.x > 0 && currentPOR.PoR.x <= 1280)
        {
          currentPOR.is_fixation = true;
          std::cout << "Fixation for " << fix_duration / fps_SMIET << " sec, from " << fix_start << " at " << currentPOR.PoR << std::endl;
        }
        else
        {
          currentPOR.is_fixation = false;
        }
      }
      else
      {
        currentPOR.is_saccade = true;
      }
    }
    // std::cout << "currentPOR.PoR " << currentPOR.PoR << std::endl;
    ET_PoRs.push_back(currentPOR);
#endif


    cv::namedWindow("EyeTracker");
    cv::setMouseCallback("EyeTracker", PointsSelectionCallBackFunc, NULL);

    #ifdef EYETRACKERS
    cv::Mat camerawithfixation;
    camera.RGBFrame.copyTo(camerawithfixation);
    cv::circle(camerawithfixation, currentPOR.PoR, 22, cv::Scalar(0,0,255), 3, CV_AA, 0);
    cv::imshow("EyeTracker", camerawithfixation);
    #else
    cv::imshow("EyeTracker", camera.RGBFrame);
    #endif

    char key = cvWaitKey(1);
    if (key == 27) // ESC
        break;

  //    std::cout << "Kinect frame size: " << kinect.RGBFrame.rows << "," << kinect.RGBFrame.cols << std::endl;
  //    std::cout << "Camera frame size: " << camera.RGBFrame.rows << "," << camera.RGBFrame.cols << std::endl;


    // Pose estimation ***********************************************************************************************************************************
#if POSE_CONTINIOUS
          cv::cvtColor(kinect_RGBFrame, briskf.imgGray1, CV_BGR2GRAY);

          #if GRAY_SCALE
            #ifdef EYETRACKERS
              briskf.imgGray2 = camera.RGBFrame;
            #endif
          #else
            cv::cvtColor(camera.RGBFrame, briskf.imgGray2, CV_BGR2GRAY);
          #endif


          briskf.matchFrames();

          /// Drawing
          if (show_matches)
          {
              cv::Mat outimg;

              drawMatches(camera.RGBFrame, briskf.keypoints2, kinect_RGBFrame, briskf.keypoints1, briskf.matches, outimg, cv::Scalar(0, 255, 0),
                          cv::Scalar(0, 0, 255), std::vector<std::vector<char> >(),
                          cv::DrawMatchesFlags::DEFAULT);
              cv::namedWindow("Matches");
              cv::imshow("Matches", outimg);
          }


          std::vector<cv::Point3f> list_points3d_rgbd; // container for the model 3D coordinates found in the scene
          std::vector<cv::Point2f> list_points2d_rgb; // container for the model 2D coordinates found in the scene

          // extract 2D matches
          for( size_t i = 0; i < briskf.matches.size(); i++ )
          {
              for( size_t j = 0; j < briskf.matches[i].size(); j++ )
              {

                  int i1 = briskf.matches[i][j].queryIdx;
                  int i2 = briskf.matches[i][j].trainIdx;
                  cv::Point2f point2d_rgb = briskf.keypoints2[i1].pt;
                  cv::Point2f point2d_rgbd = briskf.keypoints1[i2].pt;
                  cv::Point3f point3d_rgbd;
                  // @todo: Fix depth by color acquisition
                  if (kinect.getDepthByColor(point2d_rgbd, point3d_rgbd))
                  {

                      list_points2d_rgb.push_back(point2d_rgb);
                      list_points3d_rgbd.push_back(point3d_rgbd);

    //                  printf("[X,Y] %d,%d --> [X,Y,Z] %f,%f,%f\n", int(point2d_rgbd.x), int(point2d_rgbd.y), point3d_rgbd.x, point3d_rgbd.y, point3d_rgbd.z);
                  }
              }
          }

          // RANSAC parameters
          int iterationsCount = 500;      // number of Ransac iterations.
          float reprojectionError = 2.0;  // maximum allowed distance to consider it an inlier.
          double confidence = 0.95;        // ransac successful confidence.
          int pnpMethod = cv::ITERATIVE;
          cv::Mat inliers_idx;
          std::vector<cv::Point2f> list_points2d_inliers;

          if(list_points3d_rgbd.size() > 0) // None matches, then RANSAC crashes
          {
              // Estimate the pose using RANSAC approach
              et3d.estimatePoseRANSAC(list_points3d_rgbd, list_points2d_rgb,
                                      pnpMethod, inliers_idx, iterationsCount, reprojectionError, confidence);

              // Inliers keypoints to draw
              for (int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index) {
                  int n = inliers_idx.at<int>(inliers_index);         // i-inlier
                  cv::Point2f point2d = list_points2d_rgb[n]; // i-inlier point 2D
                  list_points2d_inliers.push_back(point2d);           // add i-inlier to list
              }
    //          std::cout << "#inliers: " << inliers_idx.rows << std::endl;
    //          std::cout << "Pose: " << et3d.pose << std::endl;


          }
#endif


#if TEST_FIXATION
      if (new_click)
#else
         // std::cout << "currentpor is fixation: " << currentPOR.is_fixation << std::endl;
         // std::cout << "etpor: " << ET_PoRs[ET_PoRs.size() - 2].is_fixation << std::endl;
         // std::cout << "currentPOR.PoR.x: " << currentPOR.PoR.x << std::endl;
         // std::cout << "camera.RGBFrame.cols: " << camera.RGBFrame.cols << std::endl;

       if (currentPOR.is_fixation && currentPOR.PoR.x >0 && currentPOR.PoR.x < camera.RGBFrame.cols)
#endif
          //if (bTrackingValid && try_fixation)
      {

         // Pose estimation ***********************************************************************************************************************************
#if !POSE_CONTINIOUS
               cv::cvtColor(kinect_RGBFrame, briskf.imgGray1, CV_BGR2GRAY);

               #if GRAY_SCALE
                   briskf.imgGray2 = camera.RGBFrame;
               #else
                   cv::cvtColor(camera.RGBFrame, briskf.imgGray2, CV_BGR2GRAY);
               #endif

               briskf.matchFrames();

               /// Drawing
               if (show_matches)
               {
                   cv::Mat outimg;

                   drawMatches(camera.RGBFrame, briskf.keypoints2, kinect_RGBFrame, briskf.keypoints1, briskf.matches, outimg, cv::Scalar(0, 255, 0),
                               cv::Scalar(0, 0, 255), std::vector<std::vector<char> >(),
                               cv::DrawMatchesFlags::DEFAULT);
                   cv::namedWindow("Matches");
                   cv::imshow("Matches", outimg);
               }



               std::vector<cv::Point3f> list_points3d_rgbd; // container for the model 3D coordinates found in the scene
               std::vector<cv::Point2f> list_points2d_rgb; // container for the model 2D coordinates found in the scene

               // extract 2D matches
               for( size_t i = 0; i < briskf.matches.size(); i++ )
               {
                   for( size_t j = 0; j < briskf.matches[i].size(); j++ )
                   {
                       int i1 = briskf.matches[i][j].queryIdx;
                       int i2 = briskf.matches[i][j].trainIdx;
                       cv::Point2f point2d_rgb = briskf.keypoints2[i1].pt;
                       cv::Point2f point2d_rgbd = briskf.keypoints1[i2].pt;
                       cv::Point3f point3d_rgbd;
                       // @todo: Fix depth by color acquisition
                       if (kinect.getDepthByColor(point2d_rgbd, point3d_rgbd))
                       {
                           list_points2d_rgb.push_back(point2d_rgb);
                           list_points3d_rgbd.push_back(point3d_rgbd);
         //                  printf("[X,Y] %d,%d --> [X,Y,Z] %f,%f,%f\n", int(point2d_rgbd.x), int(point2d_rgbd.y), point3d_rgbd.x, point3d_rgbd.y, point3d_rgbd.z);
                       }
                   }
               }

               // RANSAC parameters
               int iterationsCount = 500;      // number of Ransac iterations.
               float reprojectionError = 2.0;  // maximum allowed distance to consider it an inlier.
               double confidence = 0.95;        // ransac successful confidence.
               int pnpMethod = cv::ITERATIVE;
               cv::Mat inliers_idx;
               std::vector<cv::Point2f> list_points2d_inliers;

               if(list_points3d_rgbd.size() > 0) // None matches, then RANSAC crashes
               {
                   // Estimate the pose using RANSAC approach
                   et3d.estimatePoseRANSAC(list_points3d_rgbd, list_points2d_rgb,
                                           pnpMethod, inliers_idx, iterationsCount, reprojectionError, confidence);

                   // Inliers keypoints to draw
                   for (int inliers_index = 0; inliers_index < inliers_idx.rows; ++inliers_index) {
                       int n = inliers_idx.at<int>(inliers_index);         // i-inlier
                       cv::Point2f point2d = list_points2d_rgb[n]; // i-inlier point 2D
                       list_points2d_inliers.push_back(point2d);           // add i-inlier to list
                   }
         //          std::cout << "#inliers: " << inliers_idx.rows << std::endl;
         //          std::cout << "Pose: " << et3d.pose << std::endl;


               }
#endif

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




          new_click = false;
          std::cout << std::endl << "Fixation more than x seconds" << std::endl;
            last_activation = fixation_idx;
  //        std::cout << std::endl << "Fixation more than x seconds" << std::endl;
  //        std::cout << std::endl << "Current pose: " << et3d.pose << std::endl;


          std::cout << "Calculating 3D fixation ..." << std::endl;
  //        high_resolution_clock::time_point t1 = high_resolution_clock::now();
#if TEST_FIXATION
          currentPOR.PoR = testpoint;
#endif
          bool intersection_found = et3d.backproject2DPoint(cloud, currentPOR.PoR, et3d.current_3Dfixation);

          geometry_msgs::Point fixation;
          fixation.x = et3d.current_3Dfixation.x;
          fixation.y = et3d.current_3Dfixation.y;
          fixation.z = et3d.current_3Dfixation.z;


          ROS_INFO_STREAM("3D fixation: \n" << fixation);

          fixation_pub.publish(fixation);
//*/


      }

  }

  tf::Transform pose;
  pose.setOrigin(tf::Vector3( -et3d.pose.at<float>(0,3), -et3d.pose.at<float>(1,3), -et3d.pose.at<float>(2,3)));
  tf::Matrix3x3 tf3d;
  cv::Mat R = et3d._R_matrix.inv();
  tf3d.setValue(static_cast<double>(R.at<float>(0,0)), static_cast<double>(R.at<float>(0,1)), static_cast<double>(R.at<float>(0,2)),
                static_cast<double>(R.at<float>(1,0)), static_cast<double>(R.at<float>(1,1)), static_cast<double>(R.at<float>(1,2)),
                static_cast<double>(R.at<float>(2,0)), static_cast<double>(R.at<float>(2,1)), static_cast<double>(R.at<float>(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  pose.setRotation(tfqt);
  pose_broadcaster.sendTransform(tf::StampedTransform(pose, ros::Time::now(), "kinect2_rgb_optical_frame", "ETG_pose"));

  tf::Transform fixation3d;
  fixation3d.setOrigin(tf::Vector3( et3d.current_3Dfixation.x, et3d.current_3Dfixation.y, et3d.current_3Dfixation.z));
  fixation3d.setRotation(tf::Quaternion(0, 0, 0, 1));
  pose_broadcaster.sendTransform(tf::StampedTransform(fixation3d, ros::Time::now(), "kinect2_rgb_optical_frame", "fixation3D"));

} //end of loop

  return 0;
}
