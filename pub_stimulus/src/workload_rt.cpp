
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <pub_stimulus/TargetStimulus.h>
#include <sensor_msgs/Joy.h>



class WorkloadRactionTime
{
public:
  WorkloadRactionTime()
  {

    rt_pub_ = nh_.advertise<std_msgs::Float32>("/workload/RT", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1 , &WorkloadRactionTime::joyPublishedCallBack,this);
    target_sub = nh_.subscribe<pub_stimulus::TargetStimulus>("/workload/target_published", 1 ,
                                                             &WorkloadRactionTime::targetPublishedCallBAck,this);
  }
private:

  ros::NodeHandle nh_;
  ros::Publisher rt_pub_ ;
  ros::Subscriber joy_sub_, target_sub;
  std_msgs::Float32 reactionTime_;

  void joyPublishedCallBack(const sensor_msgs::Joy::ConstPtr& msg);
  void targetPublishedCallBAck(const pub_stimulus::TargetStimulus::ConstPtr& msg);


};



int main(int argc, char *argv[])
{

  return 0;
}
