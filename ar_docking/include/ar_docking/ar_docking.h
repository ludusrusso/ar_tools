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
#include <camera_info_manager/camera_info_manager.h>
#include <npb/MsgPowerInfo.h>
#include <ar_msgs/ARDocking.h>

#include <image_transport/image_transport.h>

#if ROS_VERSION_MINIMUM(1, 9, 0)
  // new cv_bridge API in Groovy
  #include <cv_bridge/cv_bridge.h>
  #include <sensor_msgs/image_encodings.h>
#else
  // Fuerte support for cv_bridge will be deprecated
  #if defined(__GNUC__)
    #warning "Support for the old cv_bridge API (Fuerte) is derecated and will be removed when Hydro is released."
  #endif
  #include <cv_bridge/CvBridge.h>
#endif


const double AR_TO_ROS = 0.001;

namespace ar_pose
{

    enum DockingState {
      SEARCHING,
      HOMING,
      CONNECTING,
      CONNECTED,
      NONE
    };



  class ARDockingPublisher
  {
  public:
    ARDockingPublisher (ros::NodeHandle & n);
    ~ARDockingPublisher (void);
    void setCamParameter ();
    void getImage ();
    void computeCmdVel(double quat[4], double pos[3]);

  private:

    void arInit ();

    void startDocking();
    void stopDocking();


    void getImageCb (const sensor_msgs::ImageConstPtr & image_msg);
    bool startStopCb(ar_msgs::ARDocking::Request &req, ar_msgs::ARDocking::Response &res);

    void powerInfoCb(const npb::MsgPowerInfo::ConstPtr& msg);
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
    CvSize sz_;


    std::string cmd_vel_topic_;
    std::string camera_topic_;
    std::string power_info_topic_;
    std::string start_stop_service_name_;

    std::string cam_info_file_;
    double kt_; // = -2.0f,
    double kx_; // = -0.05;

    ros::ServiceServer start_stop_service_;


    DockingState docking_state_;
    ros::Subscriber power_sub_;

    image_transport::ImageTransport it_;
    image_transport::Subscriber cam_sub_;
#if ! ROS_VERSION_MINIMUM(1, 9, 0)
    sensor_msgs::CvBridge bridge_;
#endif
    sensor_msgs::CameraInfo cam_info_;
#if ROS_VERSION_MINIMUM(1, 9, 0)
    cv_bridge::CvImagePtr capture_;
#else
    IplImage *capture_;
#endif


    ros::Publisher docker_state_pub_;
  };                            // end class ARSinglePublisher
}                               // end namespace ar_pose

#endif
