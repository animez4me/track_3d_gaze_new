//
// Created by alexandros on 24/08/17.
//

#ifndef WIN_LINUX_COM_CLASS_RGB_H
#define WIN_LINUX_COM_CLASS_RGB_H


#include <opencv2/opencv.hpp>
#include <iostream>  //NOLINT
#include <netdb.h>
#include <arpa/inet.h>

//#define WEBCAM
#define EYETRACKERS

#define GRAY_SCALE 1
#define SCALE 1
#define DATA_SIZE 3 //frame, gazeX, gazeY

#define IMG_WIDTH 1280*SCALE
#define IMG_HEIGHT 960*SCALE

#define DATA_BSIZE 34 //10 (4294967295) + 1(;) 1(+-) + 10(2147483647) + 1(;) + 1(+-) + 10(2147483647)

#if GRAY_SCALE
#define COLOUR_TYPE CV_8UC1
#else
#define COLOUR_TYPE CV_8UC3
#endif

class RGBcamera
{
public:
    RGBcamera(std::string filename);
    virtual ~RGBcamera();

    void captureFrame();

private:
    void readCameraParameters(std::string filename);

public:
    cv::Mat RGBFrame;
    cv::Mat camMatrix;
    cv::Mat distCoeffs;

    std::vector<int> gazedata;


private:
    #ifdef WEBCAM
        cv::VideoCapture cap;
    #endif

    #ifdef EYETRACKERS
      int status;
      struct addrinfo hints, * res;
      int listner;
      int new_conn_fd;
      size_t n;
      int bytes;
      int  imgSize;
      uchar *iptr;

    #endif

};

#endif //WIN_LINUX_COM_CLASS_RGB_H
