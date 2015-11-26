/*
 *  Single Marker Pose Estimation using ARToolkit
 *  Copyright (C) 2013, I Heart Engineering
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  William Morris <bill@iheartengineering.com>
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://www.iheartengineering.com
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AR_POSE_AR_DOCKING_H
#define AR_POSE_AR_DOCKING_H

#include <string.h>
#include <stdarg.h>

#include <artoolkit/AR/param.h>
#include <artoolkit/AR/ar.h>
#include <artoolkit/AR/video.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <resource_retriever/retriever.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>


const double AR_TO_ROS = 0.001;

namespace ar_pose
{
  class ARDockingPublisher
  {
  public:
    ARDockingPublisher (ros::NodeHandle & n);
    ~ARDockingPublisher (void);
    void setCamParameter ();
    void getImage ();

  private:
    void arInit ();

    void computeCmdVel(double quat[4], double pos[3]);

    ros::NodeHandle n_;

    bool useHistory_;
    int threshold_;
    double markerWidth_;        // Size of the AR Marker in mm

    ARParam cam_param_;         // Camera Calibration Parameters
    int patt_id_;               // AR Marker Pattern
    char pattern_filename_[FILENAME_MAX];
    bool reverse_transform_;    // Reverse direction of transform marker -> cam

    double marker_center_[2];   // Physical Center of the Marker
    double marker_trans_[3][4]; // Marker Transform
    
    ros::Publisher vel_pub_;

    int contF;
    bool getCamInfo_;
    CvSize sz_;

    CvCapture * video_capture_;
  };                            // end class ARSinglePublisher
}                               // end namespace ar_pose

#endif
