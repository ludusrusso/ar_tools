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

#include "ar_docking/ar_docking.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ar_docking");
  ros::NodeHandle n;
  ar_pose::ARDockingPublisher arSingle(n);
  arSingle.setCamParameter();
  ros::Rate r(10);
  while (ros::ok()) {
    arSingle.getImage();
    ros::spinOnce();
  }
  return 0;
}

namespace ar_pose
{
  ARDockingPublisher::ARDockingPublisher (ros::NodeHandle & n):n_ (n)
  {
    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
	  std::string default_path = "data/patt.hiro";
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

    ROS_INFO("Starting ArSinglePublisher");

    // **** get parameters

    if (!n_param.getParam("threshold", threshold_))
      threshold_ = 100;
    ROS_INFO ("\tThreshold: %d", threshold_);

    if (!n_param.getParam("marker_width", markerWidth_))
      markerWidth_ = 80.0;
    ROS_INFO ("\tMarker Width: %.1f", markerWidth_);


    // If mode=0, we use arGetTransMat instead of arGetTransMatCont
    // The arGetTransMatCont function uses information from the previous image
    // frame to reduce the jittering of the marker
    if (!n_param.getParam("use_history", useHistory_)) useHistory_ = true;
    ROS_INFO("\tUse history: %d", useHistory_);


	//modifications to allow patterns to be loaded from outside the package
	n_param.param ("marker_pattern", local_path, default_path);
	if (local_path.compare(0,5,"data/") == 0){
	  //to keep working on previous implementations, check if first 5 chars equal "data/"
	  sprintf (pattern_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
	}
	else{
	  //for new implementations, can pass a path outside the package_path
	  sprintf (pattern_filename_, "%s", local_path.c_str ());
	}

    n_param.param ("marker_center_x", marker_center_[0], 0.0);
    n_param.param ("marker_center_y", marker_center_[1], 0.0);
    ROS_INFO ("\tMarker Center: (%.1f,%.1f)", marker_center_[0], marker_center_[1]);

    n_param.param ("filter_lambda", lambda_, 0.2);
    n_param.param ("filter_kx", kx_, -2.0);
    n_param.param ("filter_kt", kt_, -0.05);
    n_param.param<std::string>("cam_info_file", cam_info_file_, "file:///home/ludovico/Desktop/camera.yaml");
    


    // **** subscribe

    ROS_INFO ("Subscribing to cmd_vel topic");
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/kobra/cmd_vel", 1000);

    power_sub_ = n_.subscribe<npb::MsgPowerInfo>("/npb/power_info", 1000, &ARDockingPublisher::powerInfoCb, this);
  }

  ARDockingPublisher::~ARDockingPublisher (void)
  {
    cvReleaseCapture(&video_capture_); //Don't know why but crash when release the image
    arVideoCapStop ();
    arVideoClose ();
  }

  void ARDockingPublisher::setCamParameter ()
  {
      camera_info_manager::CameraInfoManager cam_info(n_, "camera", cam_info_file_);
      sensor_msgs::CameraInfo cam = cam_info.getCameraInfo();

      cam_param_.xsize = cam.width;
      cam_param_.ysize = cam.height;
      
      cam_param_.mat[0][0] = cam.P[0];
      cam_param_.mat[1][0] = cam.P[4];
      cam_param_.mat[2][0] = cam.P[8];
      cam_param_.mat[0][1] = cam.P[1];
      cam_param_.mat[1][1] = cam.P[5];
      cam_param_.mat[2][1] = cam.P[9];
      cam_param_.mat[0][2] = cam.P[2];
      cam_param_.mat[1][2] = cam.P[6];
      cam_param_.mat[2][2] = cam.P[10];
      cam_param_.mat[0][3] = cam.P[3];
      cam_param_.mat[1][3] = cam.P[7];
      cam_param_.mat[2][3] = cam.P[11];
     
      cam_param_.dist_factor[0] = cam.K[2];       // x0 = cX from openCV calibration
      cam_param_.dist_factor[1] = cam.K[5];       // y0 = cY from openCV calibration
      if ( cam.distortion_model == "plumb_bob" && cam.D.size() == 5) {
        cam_param_.dist_factor[2] = -100*cam.D[0];// f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
      }
      else {
        cam_param_.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
      }

      cam_param_.dist_factor[3] = 1.0;                  // scale factor, should probably be >1, but who cares...

      arInit();
      video_capture_ = cvCaptureFromCAM(0);
      startDocking();
  }

  void ARDockingPublisher::arInit ()
  {   
    arInitCparam (&cam_param_);

    ROS_INFO ("*** Camera Parameter ***");
    arParamDisp (&cam_param_);

    // load pattern file
    ROS_INFO ("Loading pattern");
    patt_id_ = arLoadPatt (pattern_filename_);
    if (patt_id_ < 0)
    {
      ROS_ERROR ("Pattern file load error: %s", pattern_filename_);
      ROS_BREAK ();
    }

    sz_ = cvSize (cam_param_.xsize, cam_param_.ysize);
  }

  void ARDockingPublisher::startDocking() {
    docking_state_ = HOMING;
  }

  void ARDockingPublisher::stopDocking() {
    docking_state_ = NONE;
  }

  void ARDockingPublisher::computeCmdVel(double quat[4], double pos[3]) {
    static double cmd_t = 0.0;
    static double cmd_x = 0.0;
    
    geometry_msgs::Twist msg;

    static double last_lin = 0.0f;

    if (docking_state_ == HOMING && pos[2] > 0.01f) {
      msg.angular.z = lambda_ * cmd_t + kt_ * (pos[0]/pos[2]);
      msg.linear.x = lambda_ * cmd_x + kx_ * (pos[2]);
      if (pos[2] < 3.0f && pos[2] > 0.1f) {
        docking_state_ = CONNECTING;
        last_lin = msg.linear.x;
      }
    } else if (docking_state_ == CONNECTING) {
      last_lin = 0.7*last_lin + 0.1*-0.15f;
      msg.linear.x = last_lin;
    } else if (docking_state_ == CONNECTED) {
    } else {

    }
    vel_pub_.publish(msg);
  }

  void ARDockingPublisher::powerInfoCb(const npb::MsgPowerInfo::ConstPtr& msg) {
    if (msg->dock_state != 0 ){
      docking_state_ = CONNECTED;
    }
    ROS_INFO("state power: %d", msg->dock_state);
  }


  bool ARDockingPublisher::startStopCb(ar_msgs::ARDocking::Request &req, ar_msgs::ARDocking::Response &res) {
    if (req.cmd == 1) {
      startDocking();
    } else if {
      stopDocking();
    }
  res.res = true;
  return true;
  }


  void ARDockingPublisher::getImage ()
  {
    ARUint8 *dataPtr;
    ARMarkerInfo *marker_info;
    int marker_num;
    int i, k;

    dataPtr = (ARUint8 *) ((IplImage) *cvQueryFrame(video_capture_)).imageData;

    // detect the markers in the video frame 
    if (arDetectMarker (dataPtr, threshold_, &marker_info, &marker_num) < 0)
    {
      ROS_FATAL ("arDetectMarker failed");
      ROS_BREAK ();             // FIXME: I don't think this should be fatal... -Bill
    }

    // check for known patterns
    k = -1;
    for (i = 0; i < marker_num; i++)
    {
      if (marker_info[i].id == patt_id_)
      {
        ROS_DEBUG ("Found pattern: %d ", patt_id_);

        // make sure you have the best pattern (highest confidence factor)
        if (k == -1)
          k = i;
        else if (marker_info[k].cf < marker_info[i].cf)
          k = i;
      }
    }

    if (k != -1)
    {
      // **** get the transformation between the marker and the real camera
      double arQuat[4], arPos[3];

      if (!useHistory_ || contF == 0)
        arGetTransMat (&marker_info[k], marker_center_, markerWidth_, marker_trans_);
      else
        arGetTransMatCont (&marker_info[k], marker_trans_, marker_center_, markerWidth_, marker_trans_);

      contF = 1;

      //arUtilMatInv (marker_trans_, cam_trans);
      arUtilMat2QuatPos (marker_trans_, arQuat, arPos);

      // **** convert to ROS frame

      double quat[4], pos[3];
    
      pos[0] = arPos[0] * AR_TO_ROS;
      pos[1] = arPos[1] * AR_TO_ROS;
      pos[2] = arPos[2] * AR_TO_ROS;

      quat[0] = -arQuat[0];
      quat[1] = -arQuat[1];
      quat[2] = -arQuat[2];
      quat[3] = arQuat[3];

      ROS_INFO (" QUAT: Pos x: %3.5f  y: %3.5f  z: %3.5f", pos[0], pos[1], pos[2]);
      ROS_DEBUG ("     Quat qx: %3.5f qy: %3.5f qz: %3.5f qw: %3.5f", quat[0], quat[1], quat[2], quat[3]);

      computeCmdVel(quat, pos);

    }
    else
    {
      contF = 0;
      ROS_DEBUG ("Failed to locate marker");
      double quat[4] = {0}, pos[3] = {0};
      computeCmdVel(quat, pos);

    }
  }
}                               // end namespace ar_pose