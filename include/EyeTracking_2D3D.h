
#ifndef BRISK_EYETRACKING_2D3D_H
#define BRISK_EYETRACKING_2D3D_H

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <pcl/octree/octree_search.h>
// PCL
#include "Mesh.h"
//#include "ModelRegistration.h"

//#include "opencv2/core/mat.hpp"
#include "Utils.h"

class EyeTracking3D
{

public:
    EyeTracking3D(const cv::Mat intrinsics, const cv::Mat distortion);  // custom constructor
    virtual ~EyeTracking3D();

    //bool backproject2DPoint(const std::vector<std::vector<cv::Point3f> > trianglesVertices, const cv::Point2f &point2d, cv::Point3f &point3d);
    bool backproject2DPoint (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, const cv::Point2f &point2d, cv::Point3f &point3d);
    cv::Point2f backproject3DPoint(const cv::Point3f &point3d);
//		bool estimatePose(const std::vector<cv::Point3f> &list_points3d, const std::vector<cv::Point2f> &list_points2d, int flags);
    void estimatePoseRANSAC(const std::vector<cv::Point3f> &list_points3d, const std::vector<cv::Point2f> &list_points2d, int flags, cv::Mat &inliers, int iterationsCount, float reprojectionError, float confidence);
// Estimate the pose given a list of 2D/3D correspondences with RANSAC and the method to use
//    void estimatePoseRANSAC(const std::vector<cv::Point3f> &list_points3d, const std::vector<cv::Point2f> &list_points2d, int flags, cv::Mat &inliers, int iterationsCount, float reprojectionError, float confidence );
    cv::Mat get_A_matrix() const { return _A_matrix; }
    cv::Mat get_R_matrix() const { return _R_matrix; }
    cv::Mat get_t_matrix() const { return _t_matrix; }
    cv::Mat get_P_matrix() const { return _P_matrix; }

    cv::Mat get_distCoeffs() const { return distCoeffs; }

    void set_P_matrix(const cv::Mat &R_matrix, const cv::Mat &t_matrix);
    void set_RT_matrices_fromP();

    //void EyeTracking3D::computeError(cv::InputArray _m1, cv::InputArray _m2, cv::InputArray rotation, cv::InputArray translation, cv::OutputArray _err);

    cv::Point3f current_3Dfixation;

    //private:
    /** The calibration matrix */
    cv::Mat _A_matrix;
    Eigen::Matrix3f _A_mat;
    /** The computed rotation matrix */
    cv::Mat _R_matrix;
    Eigen::Matrix3f _R_mat;
    /** The computed translation matrix */
    cv::Mat _t_matrix;
    Eigen::Vector3f _t_mat;
    /** The computed projection matrix */
    cv::Mat _P_matrix;
//    Eigen::MatrixXf _P_mat;
    cv::Mat pose;
    cv::Mat distCoeffs;
    cv::Point2f current_2DPoR;

private:
    float octree_resolution;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree;
};

#endif /* BRISK_EYETRACKING_2D3D_H */
