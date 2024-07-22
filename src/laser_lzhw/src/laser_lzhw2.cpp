#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
static double Ldist = -1;
static double Ldist_l = -1;
static double Ldist_r = -1;
static int range_x = 0 ;
void LidarCallback(const sensor_msgs::LaserScan msg)
{

    Ldist = msg.ranges[420+range_x];
    Ldist_l = msg.ranges[420+range_x+1];
    Ldist_r = msg.ranges[420+range_x-1];
    // ROS_INFO("前方距离为：%f 米", Ldist);
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");    
    ros::init(argc,argv,"lidar_node");
    ros::param::set("/dist",-1);
    ros::param::set("/dist_left",-1);
    ros::param::set("/dist_right",-1);
    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/scan",10,&LidarCallback);

    while(ros::ok())
    {
    // sleep(1);
    
        bool in_end_goal_e = false ;
        double the_x = 0;
        ros::param::get("/goal_laser",in_end_goal_e);
        ros::param::get("/the_x",the_x);
        range_x = round(the_x*2.33333);
        // ROS_INFO("ceshi前方距离为:%f 米", Ldist);
        if(Ldist>0)
        {
            if(in_end_goal_e&&Ldist>0)
            {
                ros::param::set("/dist",float(Ldist));
                ros::param::set("/dist_left",float(Ldist_l));
                ros::param::set("/dist_right",float(Ldist_r));
                // ROS_INFO("ceshi前方距离为:%f 米", Ldist);
                // ROS_INFO("ceshi前方左边距离为:%f 米", Ldist_l);
                // ROS_INFO("ceshi前方右边距离为:%f 米", Ldist_r);
            }
        }
        ros::spinOnce();
    }
    return 0;
}