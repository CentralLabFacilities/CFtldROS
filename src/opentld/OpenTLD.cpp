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

/**
  * @author Georg Nebehay
*/

#include "Main.h"
#include "Config.h"
#include "ImAcq.h"
#include "Gui.h"
#include <random>

// ROS
#include <ros/ros.h>
#include "ros/ros_grabber.hpp"
#include "ros/ros_grabber_depth.hpp"

using tld::Config;
using tld::Gui;
using tld::Settings;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "cf_tld_ros", ros::init_options::AnonymousName);

    Main *main = new Main();
    Config config;
    ImAcq *imAcq = imAcqAlloc();
    Gui *gui = new Gui();

    main->gui = gui;
    main->imAcq = imAcq;

    if (config.init(argc, argv) == PROGRAM_EXIT)
    {
        return EXIT_FAILURE;
    }

    config.configure(main);

    if (main->isRosUsed) {
        ROSGrabber *ros_grabber = new ROSGrabber(config.m_settings.color_topic);
        ROSGrabberDepth *ros_grabber_depth = new ROSGrabberDepth(config.m_settings.depth_topic);
        main->ros_grabber = ros_grabber;
        main->ros_grabber_depth = ros_grabber_depth;
        ROS_DEBUG(">>> Finished ROS init");
    }

    main->tld->seed = main->seed;
    main->frame_modulo = config.m_settings.frame_modulo;
    imAcqInit(imAcq);

    if (main->showOutput)
    {
        ROS_DEBUG(">>> Initializing GUI");
        gui->init();
    }

    main->doWork();

    delete main;
    main = NULL;
    delete gui;
    gui = NULL;

    return EXIT_SUCCESS;
}
