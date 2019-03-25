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


// SELF
#include "ros_grabber_depth.hpp"

using namespace cv;
using namespace std;

ROSGrabberDepth::ROSGrabberDepth(std::string i_scope) : it_(node_handle_) {
    image_sub_ = it_.subscribe(i_scope+"/image_raw", 1, &ROSGrabberDepth::imageCallback, this);
    info_depth_sub = node_handle_.subscribe(i_scope+"/camera_info", 1, &ROSGrabberDepth::depthInfoCallback, this);
    listener = new tf::TransformListener();
    frame_nr = -1;
    pyr = 0;
    ROS_DEBUG(">>> ROS grabber depth init done");
    ROS_INFO(">>> ROS grabber D %s", i_scope.c_str());
}

ROSGrabberDepth::~ROSGrabberDepth() { }

void ROSGrabberDepth::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        if (msg->encoding == "16UC1") {
           cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } else if (msg->encoding == "32FC1") {
           cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        } else {
          ROS_ERROR(">>> Unknown image encoding %s", msg->encoding.c_str());
          return;
        }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR(">>> CV_BRIDGE exception: %s", e.what());
      return;
    }

    mtx.lock();
    frame_time = msg->header.stamp;
    frame_nr = (int)msg->header.seq;
    frame_id = msg->header.frame_id;
    source_frame = cv_ptr->image;
    if (pyr > 0) {
        cv::pyrUp(source_frame, output_frame, cv::Size(source_frame.cols, source_frame.rows));
    } else {
        output_frame = source_frame;
    }
    mtx.unlock();
}

void ROSGrabberDepth::getImage(cv::Mat *mat) {
    mtx.lock();
    *mat = output_frame;
    mtx.unlock();
}

void ROSGrabberDepth::setPyr(bool _pyr) {
    pyr = _pyr;
}

ros::Time ROSGrabberDepth::getTimestamp() {
    return frame_time;
}

int ROSGrabberDepth::getLastFrameNr() {
    return frame_nr;
}

void ROSGrabberDepth::depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& cameraInfoMsg) {
    ROS_DEBUG(">>> Entered depth info callback");
    if(!depthConstant_factor_is_set) {
        ROS_INFO(">>> Setting camera intrinsics - fx: %f; fy: %f; cx: %f; cy: %f", cameraInfoMsg->P[0], cameraInfoMsg->P[5], cameraInfoMsg->P[2], cameraInfoMsg->P[6]);
        ROS_INFO(">>> Depth camera intrinsics - depth_constant: %f; w: %f; h: %f", cameraInfoMsg->K[4], cameraInfoMsg->width, cameraInfoMsg->height);
        depthConstant_factor = cameraInfoMsg->K[4];
        camera_image_rgb_width = cameraInfoMsg->width;
        camera_image_depth_width = cameraInfoMsg->width;
        depth_fx = cameraInfoMsg->P[0];
        depth_fy = cameraInfoMsg->P[5];
        depth_cx = cameraInfoMsg->P[2];
        depth_cy = cameraInfoMsg->P[6];

        depthConstant_factor_is_set = true;
    } else {
      // Unsubscribe, we only need that once.
      info_depth_sub.shutdown();
    }
}

geometry_msgs::PoseStamped ROSGrabberDepth::getDetectionPose(const cv::Mat & depthImage, cv::Rect* bb) {
    geometry_msgs::PoseStamped base_link_pose;
    base_link_pose.header.frame_id = "invalid";

    cv::Vec3f center3D = getDepth(depthImage, bb);

    if (isfinite(center3D.val[0]) && isfinite(center3D.val[1]) && isfinite(center3D.val[2])) {
        geometry_msgs::PoseStamped camera_pose;
        camera_pose.header.frame_id = frame_id;
        camera_pose.header.stamp = ros::Time::now();
        camera_pose.pose.position.x = center3D.val[0];
        camera_pose.pose.position.y = center3D.val[1];
        camera_pose.pose.position.z = center3D.val[2];
        camera_pose.pose.orientation.x = 0.0;
        camera_pose.pose.orientation.y = 0.0;
        camera_pose.pose.orientation.z = 0.0;
        camera_pose.pose.orientation.w = 1.0;

        base_link_pose.header.frame_id = "map";

        try{
            //ROS_DEBUG(">>> Transforming received position into MAP coordinate system.");
            listener->waitForTransform(camera_pose.header.frame_id, base_link_pose.header.frame_id, camera_pose.header.stamp, ros::Duration(5.0));
            listener->transformPose(base_link_pose.header.frame_id, ros::Time(0), camera_pose, camera_pose.header.frame_id, base_link_pose);
        } catch(tf::TransformException ex) {
            ROS_ERROR(">>> Failed transform: %s", ex.what());
            base_link_pose = camera_pose;
        }
    }

    return base_link_pose;

}

void ROSGrabberDepth::createVisualisation(geometry_msgs::Pose& pose, ros::Publisher &pub) {
    //ROS_DEBUG(">>> Creating markers");
    visualization_msgs::MarkerArray marker_array;
    std::vector <visualization_msgs::Marker> human = createHuman(0, pose);
    marker_array.markers.insert(marker_array.markers.begin(), human.begin(), human.end());
    pub.publish(marker_array);
}

cv::Vec3f ROSGrabberDepth::getDepth(const cv::Mat & depthImage, cv::Rect* bb) {

    double x = (bb->br().x - bb->size().width/2) + 0.5f;
    double y = (bb->br().y - bb->size().height/2) + 0.5f;
 
    if(!(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows))
	{
		ROS_ERROR(">>> Point must be inside the image!");
		return Vec3f(
				numeric_limits<float>::quiet_NaN(),
				numeric_limits<float>::quiet_NaN(),
				numeric_limits<float>::quiet_NaN());
	}

	cv::Vec3f pt;

	// Use correct principal point from calibration
    float depthConstant_ = 1.0f/depthConstant_factor;

    // TODO: Make ros param!
	bool isInMM = false;//depthImage.type() == CV_16UC1; // is in mm?

    bool is16BitType = depthImage.type() == CV_16UC1; // is in mm?

	float unit_scaling = isInMM?0.001f:1.0f;
    float constant_x = unit_scaling / depth_fx; //cameraInfo.K.at(0)
    float constant_y = unit_scaling / depth_fy; //cameraInfo.K.at(4)
    float bad_point = std::numeric_limits<float>::quiet_NaN();

    float depth;
    bool isValid;

    if(is16BitType) {
        // Sample fore depth points to the right, left, top and down
        std::vector<float> valueList;
        for (int i=0; i<5; i++) {
            if((float)depthImage.at<uint16_t>(y,x+i) != 0){
                valueList.push_back((float)depthImage.at<uint16_t>(y,x+i));
            }
            if((float)depthImage.at<uint16_t>(y,x-i) != 0){
                valueList.push_back((float)depthImage.at<uint16_t>(y,x-i));
            }
            if((float)depthImage.at<uint16_t>(y+i,x) != 0){
                valueList.push_back((float)depthImage.at<uint16_t>(y+i,x));
            }
            if((float)depthImage.at<uint16_t>(y-i,x) != 0){
                valueList.push_back((float)depthImage.at<uint16_t>(y-i,x));
            }
        }

        ROS_DEBUG("Sampled %d values", valueList.size());

        if(!valueList.empty()) {
            std::sort (valueList.begin(), valueList.end());
            float median = valueList[(int)(valueList.size()/2)];
            depth = median;
            ROS_DEBUG("Median of depth: %f", depth);
            isValid = true;
        } else {
            depth = 0;
            isValid = false;
        }

    } else {
        float depth_samples[21];

        // Sample fore depth points to the right, left, top and down
        for (int i=0; i<5; i++) {
            depth_samples[i] = depthImage.at<float>(y,x+i);
            depth_samples[i+5] = depthImage.at<float>(y,x-i);
            depth_samples[i+10] = depthImage.at<float>(y+i,x);
            depth_samples[i+15] = depthImage.at<float>(y-i,x);
        }

        depth_samples[20] = depthImage.at<float>(y,x);

        int arr_size = sizeof(depth_samples)/sizeof(float);
        std::sort(&depth_samples[0], &depth_samples[arr_size]);
        float median = arr_size % 2 ? depth_samples[arr_size/2] : (depth_samples[arr_size/2-1] + depth_samples[arr_size/2]) / 2;

        depth = median;
        ROS_DEBUG("Median of depth: %f", depth);
        isValid = std::isfinite(depth);
    }

	// Check for invalid measurements
	if (!isValid)
	{
	    ROS_DEBUG(">>> WARN Image is invalid, whoopsie.");
		pt.val[0] = pt.val[1] = pt.val[2] = bad_point;
	} else{
		// Fill in XYZ
        pt.val[0] = (float(x) - depth_cx) * depth * constant_x;
		pt.val[1] = (float(y) - depth_cy) * depth * constant_y;
		pt.val[2] = depth*unit_scaling;
	}
    
	return pt;
}