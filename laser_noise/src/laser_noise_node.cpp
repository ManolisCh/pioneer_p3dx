#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>





class LaserNoise
{
public:
    LaserNoise()
    {

        laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("laserDriverScan", 50, &LaserNoise::laserReadCallBAck, this);
        scan_pub_ = n_.advertise<sensor_msgs::LaserScan>("scan", 50);
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber laser_sub_ ;
    ros::Publisher scan_pub_ ;
    sensor_msgs::LaserScan addedNoiseScan_;

    void laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& msg);

};






void LaserNoise::laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& msg)

{
    addedNoiseScan_ = *msg ;
    addedNoiseScan_.header.stamp = ros::Time::now();

    scan_pub_.publish(addedNoiseScan_);


}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_noise");
    LaserNoise lasernoise;

    ros::spin();
}
