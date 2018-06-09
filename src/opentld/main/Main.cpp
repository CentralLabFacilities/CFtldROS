/*  Copyright 2011 AIT Austrian Institute of Technology
*
*   This file is part of OpenTLD.
*
*   OpenTLD is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   OpenTLD is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with OpenTLD.  If not, see <http://www.gnu.org/licenses/>.
*
*/
/*
 * MainX.cpp
 *
 *  Created on: Nov 17, 2011
 *      Author: Georg Nebehay
 */

#include <chrono>
#include <thread>
#include <signal.h>

#include "Main.h"
#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"
#include "TLDUtil.h"
#include "Trajectory.h"
#include "opencv2/imgproc/imgproc.hpp"

// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include "bayes_people_tracker_msgs/PeopleWithHead.h"
#include <people_msgs/People.h>
#include <people_msgs/Person.h>

using namespace tld;
using namespace cv;

bool stop = false;

bool Main::toggleCB(clf_perception_vision_msgs::ToggleCFtldTrackingWithBB::Request& request, clf_perception_vision_msgs::ToggleCFtldTrackingWithBB::Response& response) {
    ROS_INFO("Received toggle service call");
    toggleMutex.lock();
    if (!isToggeled) {
        if (request.roi.width != 0 && request.roi.height != 0) {
            ROS_INFO("Tracking is now active");
            initialBB = new int[4];
            initialBB[0] = request.roi.x_offset;
            initialBB[1] = request.roi.y_offset;
            initialBB[2] = request.roi.width;
            initialBB[3] = request.roi.height;
            newBB = true;
            isToggeled = !isToggeled;
        } else {
            ROS_WARN("YOUR BOUNDING BOX WAS NOT VALID!");
        }
    } else {
        ROS_INFO("Deactivated tracking");
        isToggeled = !isToggeled;
    }
    toggleMutex.unlock();
    return true;
}

void inthand(int signum) {
    printf(">> CTRL+C...\n");
    stop = true;
}

void Main::doWork() {

    signal(SIGINT, inthand);

    Trajectory trajectory;

    Mat colorImage, depthImage;
    IplImage *img;

    ROS_INFO(">>> Setting up ros subcribers");
    image_transport::ImageTransport it(ros_grabber->node_handle_);
    image_transport::Publisher pub = it.advertise("cftld/detection", 1);
    ros::Publisher pub_detect_heads = ros_grabber_depth->node_handle_.advertise<bayes_people_tracker_msgs::PeopleWithHead>("/cftld/people_with_head", 1);
    ros::Publisher pub_marker_array = ros_grabber_depth->node_handle_.advertise<visualization_msgs::MarkerArray>("/cftld/marker_array", 1);
    ROS_INFO(">>> Subscribers initialized");

    std::string toggleServiceTopic = "/cftld/toggle";
    ros::ServiceServer serviceCrowd = ros_grabber_depth->node_handle_.advertiseService(toggleServiceTopic, &Main::toggleCB, this);

    if (!isRosUsed) {
        printf(">> ROS IS OFF\n");
        img = imAcqGetImg(imAcq);
        colorImage = cvarrToMat(img, true);
    } else {
        ROS_DEBUG(">>> ROS IS ON");
        // first spin, to get callback in the queue
        ros::spinOnce();
        ros_grabber->getImage(&colorImage);
        ros_grabber_depth->getImage(&depthImage);
        while (colorImage.rows*colorImage.cols < 1 || depthImage.rows * depthImage.cols < 1) {
            ros::spinOnce();
            ros_grabber->getImage(&colorImage);
            ros_grabber_depth->getImage(&depthImage);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            last_frame_nr = ros_grabber->getLastFrameNr();
            if (stop) {
                break;
            }
        }
        // Don't resize depth image, it is already 320x240 on Pepper!
        cv::resize(colorImage, colorImage, cv::Size(), 0.50, 0.50);
        img = new IplImage(colorImage);
    }

    if (colorImage.channels() == 1)
        cv::cvtColor(colorImage, colorImage, cv::COLOR_GRAY2BGR);

    if (showTrajectory)     {
        trajectory.init(trajectoryLength);
    }

    if (selectManually) {
        CvRect box;

        if (getBBFromUser(img, box, gui) == PROGRAM_EXIT) {
            return;
        }

        toggleMutex.lock();

        if (initialBB == NULL) {
            initialBB = new int[4];
        }

        initialBB[0] = box.x;
        initialBB[1] = box.y;
        initialBB[2] = box.width;
        initialBB[3] = box.height;
        toggleMutex.unlock();
    }

    FILE *resultsFile = NULL;

    if (printResults != NULL) {
        resultsFile = fopen(printResults, "w");
        if (!resultsFile) {
            fprintf(stderr, "Error: Unable to create results-file \"%s\"\n", printResults);
            exit(-1);
        }
    }

    bool reuseFrameOnce = false;
    bool skipProcessingOnce = false;
    bool paused = false;
    bool step = false;
    double tic = 0;
    double toc = 0;

    if (initialBB != NULL) {
        Rect bb = tldArrayToRect(initialBB);
        printf("Starting at %d %d %d %d\n", bb.x, bb.y, bb.width, bb.height);
        tic = static_cast<double>(getTickCount());
        tld->selectObject(colorImage, &bb);
        toc = static_cast<double>(getTickCount()) - tic;
        skipProcessingOnce = true;
        reuseFrameOnce = true;
    }

    // imAcqHasMoreFrames(imAcq)
    while (stop == false) {
        // Loop spinner
        ros::spinOnce();

        // Make sure we only run with image framerate to save CPU cycles
        if(isRosUsed) {
            if(!(ros_grabber->getLastFrameNr() != last_frame_nr)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                last_frame_nr = ros_grabber->getLastFrameNr();
                ROS_DEBUG(">>> Skipping computation step ...");
                continue;
            }
        }

        if(isToggeled) {

            if (newBB) {
                ROS_INFO("Passing new bounding box to tld");
                Rect bb = tldArrayToRect(initialBB);
                tic = static_cast<double>(getTickCount());
                tld->selectObject(colorImage, &bb);
                toc = static_cast<double>(getTickCount()) - tic;
                skipProcessingOnce = true;
                reuseFrameOnce = true;
                
                newBB = false;
            }

            if (!reuseFrameOnce && (!paused || step)) {

                if (!isRosUsed) {
                    cvReleaseImage(&img);
                    img = imAcqGetImg(imAcq);
                    colorImage = cvarrToMat(img, true);
                } else {
                    ros_grabber->getImage(&colorImage);
                    ros_grabber_depth->getImage(&depthImage);
                    cv::resize(colorImage, colorImage, cv::Size(), 0.50, 0.50);
                    img = new IplImage(colorImage);
                    last_frame_nr = ros_grabber->getLastFrameNr();
                }

                if (colorImage.channels() == 1)
                    cv::cvtColor(colorImage, colorImage, cv::COLOR_GRAY2BGR);

                if (img == NULL) {
                    printf("current image is NULL, assuming end of input.\n");
                    break;
                }
            }

            if (!skipProcessingOnce && (!paused || step)) {
                tic = static_cast<double>(getTickCount());
                tld->processImage(colorImage);
                toc = static_cast<double>(getTickCount()) - tic;
            }
            else {
                skipProcessingOnce = false;
            }

            float fps = static_cast<float>(getTickFrequency()) / toc;

            if (printResults != NULL) {
                if (tld->currBB != NULL) {
                    fprintf(resultsFile, "%d, %.2d, %.2d, %.2d, %.2d, %f, %f\n", imAcq->currentFrame - 1,
                        tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf,
                        fps);
                }
                else {
                    fprintf(resultsFile, "%d, NaN, NaN, NaN, NaN, NaN, %f\n", imAcq->currentFrame - 1, fps);
                }
            }

            if (showOutput || saveDir != NULL || isRosUsed) {
                char string[128];
                char learningString[10] = "";

                if (paused && step)
                    step = false;

                if (tld->learning) {
                    strcpy(learningString, "Learning");
                }

                sprintf(string, "#%d, process fps: %.2f, #numwindows:%d, %s", imAcq->currentFrame - 1,
                        fps, tld->detectorCascade->numWindows, learningString);
                CvScalar yellow = CV_RGB(255, 255, 0);
                CvScalar blue = CV_RGB(0, 0, 255);
                CvScalar black = CV_RGB(0, 0, 0);
                CvScalar white = CV_RGB(255, 255, 255);
                CvScalar red = CV_RGB(255, 0, 0);

                if (tld->currBB != NULL) {
                    CvScalar rectangleColor = red;
                    cvRectangle(img, tld->currBB->tl(), tld->currBB->br(), rectangleColor, 2, 8, 0);

                    geometry_msgs::PoseStamped pose = ros_grabber_depth->getDetectionPose(depthImage, tld->currBB);

                    if (pose.header.frame_id != "invalid") {
                        bayes_people_tracker_msgs::PeopleWithHead supremePeople;

                        supremePeople.header = pose.header;

                        people_msgs::Person person;
                        person.position = pose.pose.position;
                        person.reliability = 1.0;

                        supremePeople.people.push_back(person);
                        supremePeople.head_positions.push_back(pose.pose.position);

                        pub_detect_heads.publish(supremePeople);
                        ros_grabber_depth->createVisualisation(pose.pose, pub_marker_array);
                    }
                }

                CvFont font;
                cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 2, 8);
                // cvRectangle(img, cvPoint(0, 0), cvPoint(img->width, 50), black, CV_FILLED, 8, 0);
                cvPutText(img, string, cvPoint(25, 25), &font, red);

                // Publish every 10th cycle
                if (last_frame_nr % 10 == 0) {
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cvarrToMat(img, false)).toImageMsg();
                    pub.publish(msg);
                }

                if (showOutput) {
                    gui->showImage(img);
                    char key = gui->getKey();

                    if (key == 'q')
                        break;

                    if (key == 'p')
                        paused = !paused;

                    if (paused && key == 's')
                        step = true;

                    if (key == 'c') {
                        //clear everything
                        tld->release();
                    }

                    if (key == 'l') {
                        tld->learningEnabled = !tld->learningEnabled;
                        printf("LearningEnabled: %d\n", tld->learningEnabled);
                    }

                    if (key == 'a') {
                        tld->alternating = !tld->alternating;
                        printf("alternating: %d\n", tld->alternating);
                    }

                    if (key == 'r') {
                        CvRect box;

                        if (getBBFromUser(img, box, gui) == PROGRAM_EXIT) {
                            break;
                        }

                        Rect r = Rect(box);
                        tld->selectObject(colorImage, &r);
                    }
                }

                if (saveDir != NULL) {
                    char fileName[256];
                    sprintf(fileName, "%s/%.5d.png", saveDir, imAcq->currentFrame - 1);
                    cvSaveImage(fileName, img);
                }
            }
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        if (reuseFrameOnce) {
            reuseFrameOnce = false;
        }
    
        if(stop) { break; }
    }

    ROS_INFO(">>> Bye Bye!");

    if (!isRosUsed) {
        cvReleaseImage(&img);
    }

    img = NULL;
    delete ros_grabber;
    delete ros_grabber_depth;

    if (resultsFile) {
        fclose(resultsFile);
    }
}
