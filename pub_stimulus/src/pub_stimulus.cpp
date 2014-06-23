
#include <string.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/package.h>
#include <pub_stimulus/TargetStimulus.h>






class StatusPublisher
{
public:

  StatusPublisher();
  void run();

private:

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher stimulus_pub_ ;
  ros::Publisher targetPublished_pub;
  ros::Timer timerTarget_;
  void timerTargetCallback(const ros::TimerEvent&);
  pub_stimulus::TargetStimulus targetPublishedMsg_;
  std::string pathBackground_, pathTarget_;
  cv_bridge::CvImage cvBackground_, cvTarget_ ;   // intermediate cv_bridge images
  sensor_msgs::Image rosImageBackground_ , rosImageTarget_  ; // ROS msg images

};


// Constractor
StatusPublisher::StatusPublisher(): it_(nh_)
{

  stimulus_pub_ = it_.advertise("/workload/img_stimulus", 1);
  targetPublished_pub = nh_.advertise<pub_stimulus::TargetStimulus>("/workload/target_published", 1);

  timerTarget_ = nh_.createTimer(ros::Duration(5), &StatusPublisher::timerTargetCallback, this);


  // Path where the image stimulus is
  pathBackground_ = ros::package::getPath("pub_stimulus");
  pathBackground_.append("/images/background.png");
  pathTarget_ = ros::package::getPath("pub_stimulus");
  pathTarget_.append("/images/target.png");


  // Check if actually the image is there
  if( cv::imread(pathBackground_.c_str()).empty() )
    ROS_FATAL("Background image was not loaded. Could not be found on %s", pathBackground_.c_str());
  if( cv::imread(pathTarget_.c_str()).empty() )
    ROS_FATAL("Stimulus target image was not loaded. Could not be found on %s", pathTarget_.c_str());


  // Load background image with openCv
  cvBackground_.image = cv::imread(pathBackground_.c_str());
  cvBackground_.encoding = sensor_msgs::image_encodings::BGR8; // extra info for ROS


  // Load target stimulus image with openCv
  cvTarget_.image = cv::imread(pathTarget_.c_str());
  cvTarget_.encoding = sensor_msgs::image_encodings::BGR8; // extra info for ROS


  // convert to ROS image type
  cvBackground_.toImageMsg(rosImageBackground_);
  cvTarget_.toImageMsg(rosImageTarget_);

}

void StatusPublisher::timerTargetCallback(const ros::TimerEvent&)
{
  stimulus_pub_.publish(rosImageTarget_);
  targetPublishedMsg_.published.data = true;
  targetPublishedMsg_.header.stamp = ros::Time::now();
  targetPublished_pub.publish(targetPublishedMsg_);
  ros::Duration(0.200).sleep(); // target remains for 200ms

}


// Definitiion of run() function that has the main functionality
void StatusPublisher::run()
{
  stimulus_pub_.publish(rosImageBackground_);
}


int main(int argc, char** argv)
{

  ros::init(argc, argv, "pub_stimulus");

  StatusPublisher publishstimulus;


  ros::Rate loop_rate(5);

  // The main Loop where everything is runing

  while (ros::ok())
    {
      publishstimulus.run();
      ros::spinOnce();
      loop_rate.sleep();
    }


  return 0;
}
