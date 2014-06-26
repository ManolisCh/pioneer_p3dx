
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <pub_stimulus/ReactionTime.h>
#include <pub_stimulus/TargetStimulus.h>
#include <sensor_msgs/Joy.h>



class WorkloadReactionTime
{
public:
  WorkloadReactionTime()
  {
    pressed_ = true; // must initialised like that for flag further down to work
    experiment_init_pub_ = nh_.advertise<std_msgs::Bool>("/experiment_started", 1);
    rt_pub_ = nh_.advertise<pub_stimulus::ReactionTime>("/workload/RT", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1 , &WorkloadReactionTime::joyPublishedCallBack,this);
    target_sub = nh_.subscribe<pub_stimulus::TargetStimulus>("/workload/target_published", 1 ,
                                                             &WorkloadReactionTime::targetPublishedCallBAck,this);
  }
private:

  ros::NodeHandle nh_;
  ros::Publisher rt_pub_ , experiment_init_pub_;
  ros::Subscriber joy_sub_, target_sub;
  pub_stimulus::ReactionTime reactionTime_;
  bool published_, pressed_;
  double timePublished_;

  void joyPublishedCallBack(const sensor_msgs::Joy::ConstPtr& msg);
  void targetPublishedCallBAck(const pub_stimulus::TargetStimulus::ConstPtr& msg);


};

void WorkloadReactionTime::targetPublishedCallBAck(const pub_stimulus::TargetStimulus::ConstPtr& msg)

{
  published_ = msg->published.data; // Store if published for use as a flag inside the class

  if (published_ == true && pressed_== false)
    {
      // ROS_INFO("Response pending to previous target...ignoring last target");
    }

  if (published_ == true && pressed_== true)
    {
      timePublished_ = msg->header.stamp.toSec() ; // Store the time that the target stimulus appeared/published
      pressed_ = false;
      //ROS_INFO("Target published");

    }
 }

void WorkloadReactionTime::joyPublishedCallBack(const sensor_msgs::Joy::ConstPtr& msg)
{

  if (msg->buttons[5] == true)
    {
      std_msgs::Bool started;
      started.data = true;
      experiment_init_pub_.publish(started);

    }


  if (published_  == true && pressed_ == false && msg->buttons[4] == true)
    {
      reactionTime_.reactionTime.data = msg->header.stamp.toSec() - timePublished_ ;
      reactionTime_.header.stamp = ros::Time::now();
      rt_pub_.publish(reactionTime_);
      //published_ = false ;
      pressed_ = true;
      //ROS_INFO("RT is: %f", reactionTime_.reactionTime.data);
    }
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "workload_rt");
  WorkloadReactionTime workloadReactionTime;

  ros::spin();

}

