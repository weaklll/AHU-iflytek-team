#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
static double Ldist = -1;
static double Ldist_l = -1;
// static double Ldist_r = -1;
static int range_x = 0 ;
void LidarCallback(const sensor_msgs::LaserScan msg)
{

    Ldist = msg.ranges[421+range_x];
    Ldist_l = msg.ranges[421+range_x+2];
    // Ldist_r = msg.ranges[421+range_x-2];
    ROS_INFO("前方距离为：%f 米", Ldist);
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");    
    ros::init(argc,argv,"lidar_node3");
    ros::param::set("/dist",-1);
    ros::param::set("/dist_left",-1);
    // ros::param::set("/dist_1_r",-1);
    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/scan",10,&LidarCallback);

    while(ros::ok())
    {
    // sleep(1);
    
    bool in_end_goal_e = false ;
    double the_x = 0;
    ros::param::get("/goal_laser",in_end_goal_e);
    ros::param::get("/the_x",the_x);
    range_x = 4+round(the_x*840/360);
    double x_the_c = range_x*360/840 ; 
    std::cout<<"目标所在角度归一到雷达坐标系中为:"<<x_the_c<<std::endl;
    // ROS_INFO("ceshi前方距离为:%f 米", Ldist);
    if(Ldist>0)
    {
        if(in_end_goal_e == true & Ldist*Ldist_l != 0)
        {
            ros::param::set("/dist",float(Ldist));
            ros::param::set("/dist_left",float(Ldist_l));
            // ros::param::set("/dist_1_r",float(Ldist_r));
            ROS_INFO("ceshi前方距离为:%f 米", Ldist);
            ROS_INFO("ceshi前方左边距离为:%f 米", Ldist_l);
            // ROS_INFO("ceshi前方右边距离为:%f 米", Ldist_r);
        }
    }
    ros::spinOnce();
    }
    return 0;
}