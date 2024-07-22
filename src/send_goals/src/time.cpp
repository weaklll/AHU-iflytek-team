#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

ros::Time last_cmd_vel_time;
ros::Duration timeout(3.0);  // 超时时间为2秒
bool cmd_vel_received = false;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_cmd_vel_time = ros::Time::now();
    cmd_vel_received = true;
}

void timerCallback(const ros::TimerEvent&)
{
    ros::Time current_time = ros::Time::now();
    if (cmd_vel_received && (current_time - last_cmd_vel_time > timeout))
    {
        ROS_INFO("No cmd_vel received for 2 seconds, executing other program...");
        // 在这里执行其他程序
        // 例如，发布一个停止命令或调用其他节点
        cmd_vel_received = false;  // 重置标志位
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_listener");
    ros::NodeHandle nh;

    // 订阅/cmd_vel主题
    ros::Subscriber sub = nh.subscribe("/cmd_vel", 1000, cmdVelCallback);

    // 设置定时器，每秒检查一次
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), timerCallback);

    // 初始化最后接收到/cmd_vel的时间
    last_cmd_vel_time = ros::Time::now();

    ros::spin();

    return 0;
}
