#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>





class LaserNoise
{
public:
    LaserNoise()
    {

        laser_sub_ = n_.subscribe<sensor_msgs::LaserScan>("scan", 50, &LaserNoise::laserReadCallBAck, this);
        scan_pub_ = n_.advertise<sensor_msgs::LaserScan>("scan_with_noise", 50);
        timerNoise_ = n_.createTimer(ros::Duration(60) , &LaserNoise::timerNoiseCallback, this);
        noiseTriger_ = 0;
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber laser_sub_ ;
    ros::Publisher scan_pub_ ;
    sensor_msgs::LaserScan addedNoiseScan_;

    void laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& msg);
    double GaussianKernel(double mu,double sigma);
    void timerNoiseCallback(const ros::TimerEvent&);
    ros::Timer timerNoise_ ;

    bool noiseTriger_;
};






void LaserNoise::laserReadCallBAck(const sensor_msgs::LaserScan::ConstPtr& msg)

{

    double sigma;
    addedNoiseScan_ = *msg ;


    if (noiseTriger_ == 1)
    {
        for (int i=0; i < addedNoiseScan_.ranges.size() ; i++)

        {
            sigma = addedNoiseScan_.ranges[i] * 0.1; // Proportional standard deviation
            addedNoiseScan_.ranges[i] = addedNoiseScan_.ranges[i] + GaussianKernel(0,sigma);

            if (addedNoiseScan_.ranges[i] > addedNoiseScan_.range_max)
            { addedNoiseScan_.ranges[i] = addedNoiseScan_.range_max;}

            else if (addedNoiseScan_.ranges[i] < addedNoiseScan_.range_min)
            { addedNoiseScan_.ranges[i] = addedNoiseScan_.range_min;}
        }
    }



    addedNoiseScan_.header.stamp = ros::Time::now();

    scan_pub_.publish(addedNoiseScan_);


}

// Noise triger callback
void LaserNoise::timerNoiseCallback(const ros::TimerEvent&)
{

    // alternates between noise and no noise every 1:30 mins
    if (noiseTriger_ == 0)
        noiseTriger_ = 1;
    else
        noiseTriger_ = 0;

}


// Utility for adding noise
double LaserNoise::GaussianKernel(double mu,double sigma)
{
    // using Box-Muller transform to generate two independent standard normally disbributed normal variables

    double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
    double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
    double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
    //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
    // we'll just use X
    // scale to our mu and sigma
    X = sigma * X + mu;
    return X;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_noise");
    LaserNoise lasernoise;

    ros::spin();
}
