/*

Author: Florian Lier [flier AT techfak.uni-bielefeld DOT de]

By downloading, copying, installing or using the software you agree to this license.
If you do not agree to this license, do not download, install, copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

Copyright (C) 2000-2016, Intel Corporation, all rights reserved.
Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
Copyright (C) 2009-2016, NVIDIA Corporation, all rights reserved.
Copyright (C) 2010-2013, Advanced Micro Devices, Inc., all rights reserved.
Copyright (C) 2015-2016, OpenCV Foundation, all rights reserved.
Copyright (C) 2015-2016, Itseez Inc., all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are disclaimed.
In no event shall copyright holders or contributors be liable for any direct,
indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.

*/


#pragma once

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

//TF
#include <tf/transform_listener.h>

// STD
#include <mutex>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>

// CV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// BOOST
#include "boost/date_time/posix_time/posix_time.hpp"

class ROSGrabber {

public:
    ROSGrabber(std::string i_scope);
    ~ROSGrabber();
    cv::Vec3f getDepth(const cv::Mat & depthImage, int x, int y, float cx, float cy);
    geometry_msgs::PoseStamped getDetectionPose(const cv::Mat & depthImage, int x, int y, float cx, float cy);
    void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void getImage(cv::Mat *mat);
    void setPyr(bool pyr);
    ros::Time getTimestamp();
    ros::NodeHandle node_handle_;
    std::string frame_id;
    int getLastFrameNr();
    int pyr;
private:
    int frame_nr;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber info_depth_sub;
    cv::Mat output_frame;
    cv::Mat source_frame;
    ros::Time frame_time;
    std::recursive_mutex mtx;

    //DepthImage stuff
    float depthConstant_;
    float depthConstant_factor;
    float camera_image_rgb_width;
    float scale_factor_ = 2.0;
    float camera_image_depth_width;
    double shift_center_y;
    bool depthConstant_factor_is_set = false;
    tf::TransformListener* listener;
};

