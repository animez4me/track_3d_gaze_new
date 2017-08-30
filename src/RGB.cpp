//
// Created by alexandros on 24/08/17.
//
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>



#include "RGB.h"

void * get_in_addr(struct sockaddr * sa)
{
    if(sa->sa_family == AF_INET)
        return &(((struct sockaddr_in *)sa)->sin_addr);

    return &(((struct sockaddr_in6 *)sa)->sin6_addr);
}

RGBcamera::RGBcamera(std::string filename)
{
    #ifdef WEBCAM
        std::cout << "WEBCAM mode on" << std::endl;
        cap = cv::VideoCapture(0);
    #endif

    readCameraParameters(filename);

    #ifdef EYETRACKERS
      std::cout << "EYETRACKERS mode on" << std::endl;
      // Before using hint you have to make sure that the data structure is empty
      memset(& hints, 0, sizeof hints);
      // Set the attribute for hint
      hints.ai_family = AF_UNSPEC; // We don't care V4 AF_INET or 6 AF_INET6
      hints.ai_socktype = SOCK_STREAM; // TCP Socket SOCK_DGRAM
      hints.ai_flags = AI_PASSIVE;

      // Fill the res data structure and make sure that the results make sense.
      status = getaddrinfo(NULL, "8888" , &hints, &res);
      if(status != 0)
          fprintf(stderr,"getaddrinfo error: %s\n",gai_strerror(status));

      // Create Socket and check if error occured afterwards
      listner = socket(res->ai_family,res->ai_socktype, res->ai_protocol);
      if(listner < 0 )
          fprintf(stderr,"socket error: %s\n",gai_strerror(status));

      // Bind the socket to the address of my local machine and port number
      status = bind(listner, res->ai_addr, res->ai_addrlen);
      if(status < 0)
          fprintf(stderr,"bind: %s\n",gai_strerror(status));

      status = listen(listner, 10);
      if(status < 0)
          fprintf(stderr,"listen: %s\n",gai_strerror(status));

      // Free the res linked list after we are done with it
      freeaddrinfo(res);


      // We should wait now for a connection to accept
      struct sockaddr_storage client_addr;
      socklen_t addr_size;
      char s[INET6_ADDRSTRLEN]; // an empty string

      // Calculate the size of the data structure
      addr_size = sizeof client_addr;

      printf("Accepting connections ...\n");
      new_conn_fd = accept(listner, (struct sockaddr *) & client_addr, &addr_size);
      if(new_conn_fd < 0)
          fprintf(stderr,"accept: %s\n",gai_strerror(new_conn_fd));

      inet_ntop(client_addr.ss_family, get_in_addr((struct sockaddr *) &client_addr),s ,sizeof s);
      printf("Connected to %s \n",s);
  //    status = send(new_conn_fd,"Start", 5, 0);
      bytes = 0;

      RGBFrame = cv::Mat::zeros( IMG_HEIGHT, IMG_WIDTH, COLOUR_TYPE);
      imgSize = RGBFrame.total()*RGBFrame.elemSize();
      iptr = RGBFrame.data;
    #endif

}

RGBcamera::~RGBcamera() {}

void RGBcamera::captureFrame()
{
    #ifdef WEBCAM
        cap.read(RGBFrame);
    #endif

    #ifdef EYETRACKERS
//        std::cout << imgSize << std::endl;
        if ((bytes = recv(new_conn_fd, iptr, imgSize , MSG_WAITALL)) == -1) {
            std::cerr << "recv failed, received bytes = " << bytes << std::endl;
        }
        // Receive data in buffer
        char gazeDataBuf[DATA_BSIZE];
        n = recv(new_conn_fd, gazeDataBuf, DATA_BSIZE, 0);

        // Send back confirmation
        status = send(new_conn_fd, "1", 1, 0);

        if(status == -1)
        {
            close(new_conn_fd);
            _exit(4);
        }
    //        std::cout << gazeDataBuf << std::endl;
        char *pch = strtok(gazeDataBuf,";");
        gazedata.clear();
        for (int i=0; i<DATA_SIZE; i++) {
            gazedata.push_back(std::atoi(pch));
    //            printf ("%s\n",pch);
            pch = strtok(NULL, ";");
        }
   #endif
}

void RGBcamera::readCameraParameters(std::string filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        std::cout << "RGB calibration file cannot be opened" << std::endl;
    fs["Camera_Matrix"] >> camMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    std::cout << "RGB cameraMatrix: " << camMatrix << std::endl;
    std::cout << "RGB distortion: " << distCoeffs << std::endl;
}


