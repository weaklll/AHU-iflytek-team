#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

static double Ldist = -1;
static double Ldist_l = -1;
static double Ldist_r = -1;
static int range_x = 0 ;

static int num=0;

bool begin_detect_front;

void LidarCallback(const sensor_msgs::LaserScan msg)
{

    Ldist = msg.ranges[455+range_x];
    Ldist_l = msg.ranges[455+range_x+6];
    Ldist_r = msg.ranges[455+range_x-6];


    










    // ROS_INFO("前方距离为：%f 米", Ldist);
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");    
    ros::init(argc,argv,"lidar_node3");
    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/scan",10,&LidarCallback);
    bool in_end_goal_e = false ;
    double the_x = 0.0;

    int error=0;//error为雷达偏移量


    while(ros::ok())
    {
        // sleep(1);
        ros::param::get("/goal_laser",in_end_goal_e);
        ros::param::get("/the_x",the_x);

        ros::param::get("/begin_detect_front",begin_detect_front);

        while(!the_x)
        {
            ros::param::get("/the_x",the_x);
        }


        range_x = error+round(the_x*909/360);//error为雷达偏移量
        // double x_the_c = range_x*360/909 ; 
        // std::cout<<"目标所在角度归一到雷达坐标系中为:"<<x_the_c<<std::endl;

        if((range_x!=0)&&(350>Ldist*Ldist_l*Ldist_r>0)&&(in_end_goal_e==true))
        {
                ros::param::set("/dist",float(Ldist));
                ros::param::set("/dist_left",float(Ldist_l));
                ros::param::set("/dist_right",float(Ldist_r));
                ROS_INFO("ceshi前方距离为:%f 米", Ldist);
                ROS_INFO("ceshi前方左边距离为:%f 米", Ldist_l);
                ROS_INFO("ceshi前方右边距离为:%f 米", Ldist_r);  
              
        }


        ros::spinOnce();
    }
    return 0;
}