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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>

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

class ROSGrabberDepth {

public:
    ROSGrabberDepth(std::string i_scope);
    ~ROSGrabberDepth();
    cv::Vec3f getDepth(const cv::Mat & depthImage, cv::Rect* bb);
    geometry_msgs::PoseStamped getDetectionPose(const cv::Mat & depthImage, cv::Rect* bb);
    void depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void getImage(cv::Mat *mat);
    void setPyr(bool pyr);
    ros::Time getTimestamp();
    ros::NodeHandle node_handle_;
    std::string frame_id;
    int getLastFrameNr();
    int pyr;
    void createVisualisation(geometry_msgs::Pose& pose, ros::Publisher& pub);
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
    bool depthConstant_factor_is_set = false;
    tf::TransformListener* listener;

    geometry_msgs::Point generate_position(geometry_msgs::Point centre, double angle, double dx, double dy) {
    float s = sin(angle);
    float c = cos(angle);

    // rotate point
    geometry_msgs::Point res;
    res.x = dx * c - dy * s;
    res.y = dx * s + dy * c;

    // translate point back:
    res.x += centre.x;
    res.y += centre.y;
    res.z  = centre.z;
    return res;
  }

    geometry_msgs::Pose generate_extremity_position(geometry_msgs::Pose centre, double dx, double dy, double z) {
      double angle = tf::getYaw(centre.orientation) + M_PI/2;
      geometry_msgs::Point p = centre.position;
      p.z = z;
      centre.position = generate_position(p, angle, dx, dy);
      return centre;
    }

    visualization_msgs::Marker createMarker(int id, int type, int action, geometry_msgs::Pose pose,
            geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "base_link";
      marker.header.stamp = ros::Time::now();
      //marker.header.seq = ++marker_seq;
      marker.ns = "people_tracker";
      marker.id = id;
      marker.type = type;
      marker.action = action;
      marker.pose = pose;
      marker.scale = scale;
      marker.color = color;
      marker.lifetime = ros::Duration(1);
      return marker;
    }

    visualization_msgs::Marker createBody(int id, int action, geometry_msgs::Pose pose) {
      geometry_msgs::Vector3 scale;
      scale.x = 0.35;
      scale.y = 0.35;
      scale.z = 0.7;
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 139.0F/255.0F;
      color.g = 0.0F/255.0F;
      color.b = 0.0F/255.0F;
      pose.position.z = 1.1;
      return createMarker(id, visualization_msgs::Marker::CYLINDER, action, pose, scale, color);
    }

    visualization_msgs::Marker createHead(int id, int action, geometry_msgs::Pose pose) {
      geometry_msgs::Vector3 scale;
      scale.x = 0.3;
      scale.y = 0.3;
      scale.z = 0.3;
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 233.0F/255.0F;
      color.g = 150.0F/255.0F;
      color.b = 122.0F/255.0F;
      pose.position.z = 1.6;
      return createMarker(id, visualization_msgs::Marker::SPHERE, action, pose, scale, color);
    }

    std::vector<visualization_msgs::Marker> createLegs(int idl, int idr, int action, geometry_msgs::Pose pose) {
      std::vector<visualization_msgs::Marker> legs;
      geometry_msgs::Vector3 scale;
      scale.x = 0.15;
      scale.y = 0.2;
      scale.z = 0.8;
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 0.0F/255.0F;
      color.g = 0.0F/255.0F;
      color.b = 139.0F/255.0F;
      legs.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, 0.1, 0.0, 0.4), scale, color));
      legs.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, -0.1, 0.0, 0.4), scale, color));
      return legs;
    }

    std::vector<visualization_msgs::Marker> createArms(int idl, int idr, int action, geometry_msgs::Pose pose) {
      std::vector<visualization_msgs::Marker> arms;
      geometry_msgs::Vector3 scale;
      scale.x = 0.1;
      scale.y = 0.1;
      scale.z = 0.7;
      std_msgs::ColorRGBA color;
      color.a = 1.0;
      color.r = 139.0F/255.0F;
      color.g = 0.0F/255.0F;
      color.b = 0.0F/255.0F;
      arms.push_back(createMarker(idl, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, 0.2, 0.0, 1.1), scale, color));
      arms.push_back(createMarker(idr, visualization_msgs::Marker::CYLINDER, action, generate_extremity_position(pose, -0.2, 0.0, 1.1), scale, color));
      return arms;
    }

    std::vector<visualization_msgs::Marker> createHuman(int id, geometry_msgs::Pose pose) {
      std::vector<visualization_msgs::Marker> human;
      human.push_back(createHead(id++, visualization_msgs::Marker::ADD, pose));
      human.push_back(createBody(id++, visualization_msgs::Marker::ADD, pose));
      std::vector<visualization_msgs::Marker> legs = createLegs(id++, id++, visualization_msgs::Marker::ADD, pose);
      human.insert(human.end(), legs.begin(), legs.end());
      std::vector<visualization_msgs::Marker> arms = createArms(id++, id++, visualization_msgs::Marker::ADD, pose);
      human.insert(human.end(), arms.begin(), arms.end());
      return human;
    }



};

