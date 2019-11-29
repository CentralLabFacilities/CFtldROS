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
 * imAcq.h
 *
 *  Created on: 18 May 2011
 *      Author: Georg Nebehaiy
 *  Modified on: 26th May 2013
 *      By: Jonathan Senecal
 */

#ifndef IMACQ_IMPL_H_
#define IMACQ_IMPL_H_

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/video/video.hpp>

/**
 * Capturing method
 */
enum ImacqMethod
{
    IMACQ_IMGS, //!< Images
    IMACQ_CAM, //!< Camera
    IMACQ_VID, //!< Video
    IMACQ_LIVESIM, //!< Livesim
    IMACQ_STREAM, //!< Stream
    IMACQ_ROS //!< ROS
};

typedef struct
{
    int method;
    const char *imgPath;
    cv::VideoCapture *capture;
    int lastFrame;
    int currentFrame;
    int startFrame;
    int camNo;
    int64 startTime;
    float fps;
} ImAcq;

ImAcq *imAcqAlloc();

void imAcqInit(ImAcq *imAcq);

void imAcqRelease(ImAcq *imAcq);

void imAcqVidSetNextFrameNumber(ImAcq *imAcq, int nFrame);
int imAcqVidGetNextFrameNumber(ImAcq *imAcq);
int imAcqVidGetNumberOfFrames(ImAcq *imAcq);
int imAcqHasMoreFrames(ImAcq *imAcq);
cv::Mat imAcqGetImgAndAdvance(ImAcq *imAcq);
cv::Mat imAcqGetImg(ImAcq *imAcq);
cv::Mat imAcqGetImgByFrame(ImAcq *imAcq, int fNo);
cv::Mat imAcqGetImgByCurrentTime(ImAcq *imAcq);
cv::Mat imAcqLoadImg(ImAcq *imAcq, char *path);
cv::Mat imAcqLoadCurrentFrame(ImAcq *imAcq);
cv::Mat imAcqLoadVidFrame(cv::VideoCapture *capture);
cv::Mat imAcqGrab(cv::VideoCapture *capture);
void imAcqAdvance(ImAcq *imAcq);
void imAcqFree(ImAcq *);

#endif /* IMACQ_H_ */
