#pragma GCC diagnostic ignored "-Wpedantic"
#include <iostream>
#include <sstream>

#include "EyeTracking_2D3D.h"
#include "Mesh.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>



  EyeTracking3D::EyeTracking3D(const cv::Mat intrinsics, const cv::Mat distortion):
    octree_resolution(0.02f), octree(octree_resolution)
  {
    intrinsics.copyTo(_A_matrix);
    _A_matrix.convertTo(_A_matrix, CV_32F);
    distortion.copyTo(distCoeffs);
    distCoeffs.convertTo(distCoeffs, CV_32F);

    //_A_matrix.at<float>(0, 0) = intrinsics[0];       //      [ fx   0  cx ]
    //_A_matrix.at<float>(1, 1) = intrinsics[1];       //      [  0  fy  cy ]
    //_A_matrix.at<float>(0, 2) = intrinsics[2];       //      [  0   0   1 ]
    //_A_matrix.at<float>(1, 2) = intrinsics[3];
    //_A_matrix.at<float>(2, 2) = 1;
    _R_matrix = cv::Mat::zeros(3, 3, CV_32F);   // rotation matrix
    _t_matrix = cv::Mat::zeros(3, 1, CV_32F);   // translation matrix
    _P_matrix = cv::Mat::zeros(3, 4, CV_32F);   // rotation-translation matrix
    pose = cv::Mat::zeros(4, 4, CV_32F);   // rotation-translation matrix
  }

  EyeTracking3D::~EyeTracking3D()
  {
    // TODO Auto-generated destructor stub
  }

  void EyeTracking3D::set_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix)
  {
    // Rotation-Translation Matrix Definition
    R_matrix.copyTo(_P_matrix.rowRange(0, 3).colRange(0, 3));
    t_matrix.copyTo(_P_matrix.rowRange(0, 3).col(3));
        pose = cv::Mat::eye(4, 4, CV_32F);
        _P_matrix.copyTo(pose.rowRange(0, 3).colRange(0, 4));

    //_P_matrix.at<float>(0, 0) = R_matrix.at<float>(0, 0);
    //_P_matrix.at<float>(0, 1) = R_matrix.at<float>(0, 1);
    //_P_matrix.at<float>(0, 2) = R_matrix.at<float>(0, 2);
    //_P_matrix.at<float>(1, 0) = R_matrix.at<float>(1, 0);
    //_P_matrix.at<float>(1, 1) = R_matrix.at<float>(1, 1);
    //_P_matrix.at<float>(1, 2) = R_matrix.at<float>(1, 2);
    //_P_matrix.at<float>(2, 0) = R_matrix.at<float>(2, 0);
    //_P_matrix.at<float>(2, 1) = R_matrix.at<float>(2, 1);
    //_P_matrix.at<float>(2, 2) = R_matrix.at<float>(2, 2);
    //_P_matrix.at<float>(0, 3) = t_matrix.at<float>(0);
    //_P_matrix.at<float>(1, 3) = t_matrix.at<float>(1);
    //_P_matrix.at<float>(2, 3) = t_matrix.at<float>(2);

    //std::cout << "Current P: " << std::endl;
    //std::cout << _P_matrix << std::endl;
    set_RT_matrices_fromP();

  }

  void EyeTracking3D::set_RT_matrices_fromP()
  {
    //std::cout << "test" << std::endl;
    _R_matrix = _P_matrix.rowRange(0, 3).colRange(0, 3);
    _t_matrix =  _P_matrix.rowRange(0, 3).col(3);

    //_R_matrix.at<float>(0, 0) = _P_matrix.at<float>(0, 0);
    //_R_matrix.at<float>(0, 1) = _P_matrix.at<float>(0, 1);
    //_R_matrix.at<float>(0, 2) = _P_matrix.at<float>(0, 2);
    //_R_matrix.at<float>(1, 0) = _P_matrix.at<float>(1, 0);
    //_R_matrix.at<float>(1, 1) = _P_matrix.at<float>(1, 1);
    //_R_matrix.at<float>(1, 2) = _P_matrix.at<float>(1, 2);
    //_R_matrix.at<float>(2, 0) = _P_matrix.at<float>(2, 0);
    //_R_matrix.at<float>(2, 1) = _P_matrix.at<float>(2, 1);
    //_R_matrix.at<float>(2, 2) = _P_matrix.at<float>(2, 2);
    //_t_matrix.at<float>(0) = _P_matrix.at<float>(0, 3);
    //_t_matrix.at<float>(1) = _P_matrix.at<float>(1, 3);
    //_t_matrix.at<float>(2) = _P_matrix.at<float>(2, 3);
  }
  // *******************************************************************************************************************************
  // Backproject a 3D point to 2D using the estimated pose parameters

  cv::Point2f EyeTracking3D::backproject3DPoint(const cv::Point3f &point3d)
  {

    // 3D point vector [x y z 1]'
    cv::Mat point3d_vec = cv::Mat(4, 1, CV_32F);
    point3d_vec.at<float>(0) = point3d.x;
    point3d_vec.at<float>(1) = point3d.y;
    point3d_vec.at<float>(2) = point3d.z;
    point3d_vec.at<float>(3) = 1;

    // 2D point vector [u v 1]'
    cv::Mat point2d_vec = cv::Mat(4, 1, CV_32F);

    point2d_vec = _A_matrix * _P_matrix * point3d_vec;

    // Normalization of [u v]'
    cv::Point2f point2d;
    point2d.x = (float)(point2d_vec.at<float>(0) / point2d_vec.at<float>(2));
    point2d.y = (float)(point2d_vec.at<float>(1) / point2d_vec.at<float>(2));

    //std::cout << point3d << std::endl;

    return point2d;
  }


  // Back project a 2D point to 3D and returns if it's on the mesh surface
  bool EyeTracking3D::backproject2DPoint(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Point2f &point2d, cv::Point3f &point3d)
  {
    // Triangles list of the object mesh
    //std::vector<std::vector<int> > triangles_list = mesh->getTrianglesList();
    //std::cout << _P_matrix << std::endl;
    //std::cout << _R_matrix << std::endl;
    //std::cout << _t_matrix << std::endl;
    cv::cv2eigen(_A_matrix,_A_mat);
    cv::cv2eigen(_R_matrix,_R_mat);
    cv::cv2eigen(_t_matrix,_t_mat);

    double lambda = 8;
    double u = point2d.x;
    double v = point2d.y;

    // Point in vector form
    Eigen::Vector3f point2d_vec(u*lambda, v*lambda, lambda);

    // Point in camera coordinates
    Eigen::Vector3f X_c;
    X_c = _A_mat.inverse() * point2d_vec; // 3x1

    // Point in world coordinates
    Eigen::Vector3f X_w = _R_mat.inverse() * (X_c - _t_mat); // 3x1

    /*std::cout << X_w << std::endl;*/

    // Center of projection
    Eigen::Vector3f C_op = (_R_mat.inverse())*(-1) * _t_mat; // 3x1

    // Ray direction vector
    Eigen::Vector3f ray = X_w - C_op; // 3x1
    ray = ray / ray.norm(); // 3x1

    Eigen::Vector3f origin(C_op(0,0), C_op(1,0), C_op(2,0));
    Eigen::Vector3f direction(ray(0,0), ray(1,0), ray(2,0));
    std::vector<int> k_indices;

    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    int intersections = octree.getIntersectedVoxelIndices(origin, direction, k_indices, 0);

    std::cout << "Intersections: " << intersections << std::endl;
//    std::cout << "k_indices.size(): " << k_indices.size() << std::endl;
//    std::cout << "k_indices2.size(): " << k_indices2.size() << std::endl;
//    std::cout << "k_indices2[0]: " << k_indices2[0] << std::endl;
//    for (int i=0; i<k_indices.size(); i++)
//        std::cout << "k_indices: " << cloud_xyz->points[k_indices[i]] << std::endl;

    if (!k_indices.empty())
    {
      point3d.x = cloud->points[k_indices[0]].x;
      point3d.y = cloud->points[k_indices[0]].y;
      point3d.z = cloud->points[k_indices[0]].z;
      return true;
    }
    else
    {
      point3d = cv::Point3f(0,0,0);
      return false;
    }
  }


// Estimate the pose given a list of 2D/3D correspondences with RANSAC and the method to use
void EyeTracking3D::estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d,        // list with model 3D coordinates
                   const std::vector<cv::Point2f> &list_points2d,        // list with scene 2D coordinates
                   int flags, cv::Mat &inliers, int iterationsCount,     // PnP method; inliers container
                   float reprojectionError, float confidence )           // Ransac parameters
{

  cv::Mat rvec; rvec = cv::Mat::zeros(3, 1, CV_32F);          // output rotation vector
  cv::Mat tvec; tvec = cv::Mat::zeros(3, 1, CV_32F);          // output translation vector
  bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
  // initial approximations of the rotation and translation vectors

  cv::solvePnPRansac( list_points3d, list_points2d, _A_matrix, distCoeffs, rvec, tvec,
            useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
            inliers, flags );
  Rodrigues(rvec,_R_matrix);                   // converts Rotation Vector to Matrix
  _t_matrix = tvec;                            // set translation matrix

//    std::cout << "list_points3d: " << list_points3d << std::endl;
//    std::cout << "list_points2d: " << list_points2d << std::endl;
//    std::cout << "_A_matrix: " << _A_matrix << std::endl;
//    std::cout << "rvec: " << rvec << std::endl;
//    std::cout << "tvec: " << tvec << std::endl;
//    std::cout << "_R_matrix: " << _R_matrix << std::endl;
this->set_P_matrix(_R_matrix, _t_matrix);    // set rotation-translation matrix
//  this->set_P_matrix(_R_matrix.inv(), (-1)*_t_matrix);    // set rotation-translation matrix
}

