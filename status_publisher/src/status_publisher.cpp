
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int8.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <ros/package.h>
#include <string.h>







class StatusPublisher
{
public:

  StatusPublisher();


private:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher mode_pub_ , navStatus_pub_;
  ros::Subscriber mode_sub_ , navStatus_sub_ ;

  std::string pathTeleop_, pathAuto_ ,pathStop_;
  std::string pathActive_, pathSucceeded_ ,pathAborted_;
  cv_bridge::CvImage cvModeAuto_, cvModeTeleop_ , cvModeStop_;                    // intermediate cv_bridge images
  cv_bridge::CvImage cvNavActive_, cvNavSucceeded_, cvNavAborted_;
  sensor_msgs::Image rosImageModeAuto_ , rosImageModeTeleop_ , rosImageModeStop_ ; // ROS msg images
  sensor_msgs::Image rosImageNavActive_, rosImageNavSucceeded_, rosImageNavAborted_;
  void modeCallBack(const std_msgs::Int8::ConstPtr& mode);
  void navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& status);

};


// Constractor
StatusPublisher::StatusPublisher(): it_(nh_)
{

  // Subscribers
  mode_sub_ = nh_.subscribe<std_msgs::Int8>("/control_mode", 1 , &StatusPublisher::modeCallBack, this);
  navStatus_sub_  = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1, &StatusPublisher::navStatusCallBack, this);

  // publishers
  mode_pub_ = it_.advertise("/robot_status/mode", 1);
  navStatus_pub_ = it_.advertise("/robot_status/nav",1);


  // Path where the mode images are
  pathTeleop_ = ros::package::getPath("status_publisher");
  pathTeleop_.append("/images/teleop.png");

  pathAuto_ = ros::package::getPath("status_publisher");
  pathAuto_.append("/images/auto.png");

  pathStop_ = ros::package::getPath("status_publisher");
  pathStop_.append("/images/stop.png");


  // Path where the nav status images are
  pathActive_ = ros::package::getPath("status_publisher");
  pathActive_.append("/images/active.png");

  pathSucceeded_ = ros::package::getPath("status_publisher");
  pathSucceeded_.append("/images/succeeded.png");

  pathAborted_ = ros::package::getPath("status_publisher");
  pathAborted_.append("/images/aborted.png");


  // Safety Check if actually the image is there and loaded
  if( cv::imread(pathTeleop_.c_str()).empty() )
    ROS_FATAL("Teleop image was not loaded. Could not be found on %s", pathTeleop_.c_str());

  else if (cv::imread(pathAuto_.c_str()).empty() )
    ROS_FATAL("Auto image was not loaded. Could not be found on %s", pathAuto_.c_str());

  else if (cv::imread(pathStop_.c_str()).empty() )
    ROS_FATAL("Stop image was not loaded. Could not be found on %s", pathStop_.c_str());

  else if (cv::imread(pathActive_.c_str()).empty()  )
    ROS_FATAL("Active image was not loaded. Could not be found on %s", pathActive_.c_str());

  else if ( cv::imread(pathSucceeded_.c_str()).empty() )
    ROS_FATAL("Succeded image was not loaded. Could not be found on %s", pathSucceeded_.c_str());

  else if ( cv::imread(pathAborted_.c_str()).empty() )
    ROS_FATAL("Aborted image was not loaded. Could not be found on %s", pathAborted_.c_str());

  else
    ROS_INFO("All images loaded successfuly");


  // Load auto image with openCv
  cvModeAuto_.image = cv::imread(pathAuto_.c_str());
  cvModeAuto_.encoding = sensor_msgs::image_encodings::BGR8;
  // Load teleop image with openCv
  cvModeTeleop_.image = cv::imread(pathTeleop_.c_str());
  cvModeTeleop_.encoding = sensor_msgs::image_encodings::BGR8;
  // Load Stop image with openCv
  cvModeStop_.image = cv::imread(pathStop_.c_str());
  cvModeStop_.encoding = sensor_msgs::image_encodings::BGR8;
  // Load Active image with openCv
  cvNavActive_.image = cv::imread(pathActive_.c_str());
  cvNavActive_.encoding = sensor_msgs::image_encodings::BGR8;
  // Load Succeeded image with openCv
  cvNavSucceeded_.image = cv::imread(pathSucceeded_.c_str());
  cvNavSucceeded_.encoding = sensor_msgs::image_encodings::BGR8;
  // Load Aborted image with openCv
  cvNavAborted_.image = cv::imread(pathAborted_.c_str());
  cvNavAborted_.encoding = sensor_msgs::image_encodings::BGR8;


  // convert to ROS image type (MODE)
  cvModeAuto_.toImageMsg(rosImageModeAuto_);
  cvModeTeleop_.toImageMsg(rosImageModeTeleop_);
  cvModeStop_.toImageMsg(rosImageModeStop_);

  // convert to ROS image type (NAV status)
  cvNavActive_.toImageMsg(rosImageNavActive_);
  cvNavSucceeded_.toImageMsg(rosImageNavSucceeded_);
  cvNavAborted_.toImageMsg(rosImageNavAborted_);


  // Publish the default mode
  mode_pub_.publish(rosImageModeStop_);

}

void StatusPublisher::modeCallBack(const std_msgs::Int8::ConstPtr& mode)
{
  if (mode->data == 0)
    mode_pub_.publish(rosImageModeStop_);
  if (mode->data == 1)
    mode_pub_.publish(rosImageModeTeleop_);
  if (mode->data == 2)
    mode_pub_.publish(rosImageModeAuto_);

}

void StatusPublisher::navStatusCallBack(const actionlib_msgs::GoalStatusArray::ConstPtr& status)
{
  if (!status->status_list.empty()) // First make you the vector is not empty to avoid memory allocation errors.
    {
      actionlib_msgs::GoalStatus goalStatus = status->status_list[0];

      if (goalStatus.status == 1)
        navStatus_pub_.publish(rosImageNavActive_);

      else if (goalStatus.status== 3)
        navStatus_pub_.publish(rosImageNavSucceeded_);


      else if (goalStatus.status == 4)
        navStatus_pub_.publish(rosImageNavAborted_);

      else {
          ROS_INFO("Status Something else?? Check /move_base/status for code");
        }

    }
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "status_publisher");

  StatusPublisher publishStatus;


  ros::Rate loop_rate(5);

  // The main Loop where everything is runing

  while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }


  return 0;
}
