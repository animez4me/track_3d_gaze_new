//
// Created by alexandros on 21/07/17.
//

#include "RGBD.h"

void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data);
void point_picking_callback(const pcl::visualization::PointPickingEvent& event, void*);

RGBDcamera::RGBDcamera(const std::string RGBD_CAMERA_intrinsics, const std::string RGBD_IR_intrinsics, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)//:
        //viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
{
//  RGBDcamera::RGBDcamera(std::string filename,):
//    k2g(OPENGL, true, true),
//    viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"))
//  {


//    cloud = k2g.getCloud();

//    k2g.printParameters();


//    cloud->sensor_orientation_.w() = 0.0;
//    cloud->sensor_orientation_.x() = 1.0;
//    cloud->sensor_orientation_.y() = 0.0;
//    cloud->sensor_orientation_.z() = 0.0;

//    viewer->setBackgroundColor (0, 0, 0);
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
//    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");


//    viewer->registerKeyboardCallback(KeyboardEventOccurred);
//    viewer->registerPointPickingCallback(&point_picking_callback);

    readCameraParametersColour(RGBD_CAMERA_intrinsics);
    readCameraParametersIR(RGBD_IR_intrinsics);

//    k2g.disableLog();

////    RGBDcameraThread_ = std::thread(&RGBDcamera::captureRGBDdata, this);

}

RGBDcamera::~RGBDcamera()
{
//    RGBDcameraThread_.join();
}

//void RGBDcamera::captureRGBDdata() {
//    for (;;)
//        k2g.get(RGBFrame, DepthFrame, cloud);

//}

void RGBDcamera::readCameraParametersColour(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        std::cout << "RGBD calibration file cannot be opened" << std::endl;
    fs["cameraMatrix"] >> camMatrix;
    fs["distortionCoefficients"] >> distCoeffs;
    std::cout << "RGBD cameraMatrix: " << camMatrix << std::endl;
    std::cout << "RGBD distortion: " << distCoeffs << std::endl;
}

void RGBDcamera::readCameraParametersIR(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        std::cout << "RGBD calibration file cannot be opened" << std::endl;
    fs["cameraMatrix"] >> camMatrixIR;
    fs["distortionCoefficients"] >> distCoeffsIR;
    std::cout << "RGBD cameraMatrixIR: " << camMatrixIR << std::endl;
    std::cout << "RGBD distortionIR: " << distCoeffsIR << std::endl;

}

bool RGBDcamera::getDepthByColor(const cv::Point2f &point2d, cv::Point3f &point3d) {

    const float cx(camMatrix.at<double>(0,2)), cy(camMatrix.at<double>(1,2));
    const float fx(1 / camMatrix.at<double>(0,0)), fy(1 / camMatrix.at<double>(1,1));

//    float scaleY = 1080.0/424.0;
//    float scaleX = 1920.0/512.0;
//    const float cx(camMatrixIR.at<double>(0,2)*scaleX), cy(camMatrixIR.at<double>(1,2)*scaleY);
//    const float fx(1 / (camMatrixIR.at<double>(0,0)*scaleX)), fy(1 / (camMatrixIR.at<double>(1,1)*scaleY));

    int c = int(point2d.x + 0.5), r = int(point2d.y + 0.5);
    float x, y, z;

    const float depth_val = DepthFrame.at<unsigned short>(r+1, c) / 1000.0f;

    const float bad_point = std::numeric_limits<float>::quiet_NaN();

    if (isnan(depth_val) || depth_val <= 0.001) {
        //depth value is not valid
        x = y = z = bad_point;
    } else {
        x = (c + 0.5 - cx) * fx * depth_val;
        y = (r + 0.5 - cy) * fy * depth_val;
        z = depth_val;
    }

    point3d = cv::Point3f(x,y,z);

    return (std::isfinite(point3d.x) && point3d.x < 100);
}

void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
    std::string pressed = event.getKeySym();
    if(event.keyDown ())
    {
        if(pressed == "s")
        {
            std::cout << "pressed s" << std::endl;
        }
    }
}

void point_picking_callback(const pcl::visualization::PointPickingEvent& event, void*)
{
    if (event.getPointIndex() == -1)
        return;
    cv::Point3f current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);

    std::cout << "3D point: " << current_point.x << ", " << current_point.y << ", " << current_point.z << "" << std::endl;

}
