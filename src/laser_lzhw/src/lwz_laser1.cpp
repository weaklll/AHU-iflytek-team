/*
lwz_laser1.cpp
编写于2024.7.8
将laser_lzhw3.cpp 获取特定角度及其相邻点位的雷达信息
改为
获取特定角度的雷达信息并向左右各延伸50个单位
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define PI 3.141592

// #define fenbianlv (360/909*PI/180)

double theta=0.0;
double xielv_threshold=0.03;
bool in_end_goal_e = false ;

double Ldist = -1;
double Ldist_l = -1;
double Ldist_r = -1;

double Ldist_r_x = -1;
double Ldist_r_y = -1;
double Ldist_l_x = -1;
double Ldist_l_y = -1;
double Ldist_mid_x = -1;
double Ldist_mid_y = -1;

double xielv = -1;
double right_xielv = -1;
double left_xielv = -1;

double right_x = -1;
double right_y = -1;
double left_x = -1;
double left_y = -1;
double mid_x = -1;
double mid_y = -1;

double right_max_x = -1;
double right_max_y = -1;
double left_max_x = -1;
double left_max_y = -1;
int right_max_num = 50;
int left_max_num = 50;

double board_long = -1;

int range_x = 0 ;

int num=0;
double fenbianlv = 0.006912195; //360/909*PI/180

double ladar_distance[101]={0};   

void LidarCallback(const sensor_msgs::LaserScan msg)
{
    ros::param::get("/goal_laser",in_end_goal_e);

    ros::param::get("/the_x",theta);
    theta=theta/180*PI;
    Ldist = msg.ranges[455+range_x];
    Ldist_l = msg.ranges[455+range_x+6];
    Ldist_r = msg.ranges[455+range_x-6];

    Ldist_mid_x = Ldist * cos(theta);
    Ldist_mid_y = Ldist * sin(theta);
    Ldist_r_x = Ldist_r *cos(theta - 6 * fenbianlv);
    Ldist_r_y = Ldist_r *sin(theta - 6 * fenbianlv);
    Ldist_l_x = Ldist_l *cos(theta + 6 * fenbianlv);
    Ldist_l_y = Ldist_l *sin(theta + 6 * fenbianlv);

    xielv = (Ldist_r_y - Ldist_l_y)/(Ldist_r_x - Ldist_l_x);

    //获取板子中心向左右方向各50数据位的距离信息，计入到lidar_distance数组中
     //0-49 50 51-100
    for(int count =-50 ; count<=50 ; count++)
    {
        if(msg.ranges[455+range_x+count]>7)
        {
            ladar_distance[count+50]=msg.ranges[455+range_x+count];
        }
        else
        {
            ladar_distance[count+50]=msg.ranges[455+range_x+count-1];
        }

    }



    // printf("\n*********************\n");
    // for(int count =0 ; count<=100 ; count++)
    // {
    //     printf("ladar_distance[%d]=%f\n",count,ladar_distance[count]);
    // }
    // printf("\n*********************\n");

    //处理右侧的距离信息 51-100
    for(int right_count=44 ; right_count>=0 && (in_end_goal_e==true) ; right_count--)
    {
        printf("正在处理右侧的第%d个数据。。。。。。",right_count);
        right_x = ladar_distance[right_count] * cos(theta-((50-right_count)*fenbianlv));
        right_y = ladar_distance[right_count] * sin(theta-((50-right_count)*fenbianlv));

        right_xielv=(right_y - Ldist_l_y) / (right_x - Ldist_l_x);
        printf("此时right_xielv=%f,标准斜率=%f\n",right_xielv,xielv);
        if(abs(right_xielv-xielv) < xielv_threshold)
        {
            right_max_x = right_x;
            right_max_y = right_y;
            right_max_num = right_count;
        }
        else    //连续判断三次斜率差是否过大
        {
            printf("进入了右侧的第一次错误，此时right_count=%d",right_count);
            right_count--;
            right_x = ladar_distance[right_count] * cos(theta-((50-right_count)*fenbianlv));
            right_y = ladar_distance[right_count] * sin(theta-((50-right_count)*fenbianlv));
            right_xielv=(right_y - Ldist_l_y) / (right_x - Ldist_l_x);
            printf("此时right_xielv=%f,标准斜率=%f\n",right_xielv,xielv);
            if(abs(right_xielv-xielv) < xielv_threshold)
            {
                right_max_x = right_x;
                right_max_y = right_y;
                right_max_num = right_count;
            }
            else
            {
                printf("进入了右侧的第二次错误，此时right_count=%d",right_count);
                right_count--;
                right_x = ladar_distance[right_count] * cos(theta-((50-right_count)*fenbianlv));
                right_y = ladar_distance[right_count] * sin(theta-((50-right_count)*fenbianlv));
                right_xielv=(right_y - Ldist_l_y) / (right_x - Ldist_l_x);
                printf("此时right_xielv=%f,标准斜率=%f\n",right_xielv,xielv);
                if(abs(right_xielv-xielv) < xielv_threshold)
                {
                    right_max_x = right_x;
                    right_max_y = right_y;
                    right_max_num = right_count;
                }
                else
                {
                    printf("进入了右侧的第三次错误，此时right_count=%d\n",right_count);
                    break;
                }
            }
        }
    }

    //处理左侧的距离信息 0-49
    for(int left_count=56 ; left_count<=100 && (in_end_goal_e==true) ; left_count++)
    {
        printf("正在处左侧的第%d个数据。。。。。。",left_count);
        left_x = ladar_distance[left_count] * cos(theta+((left_count-50)*fenbianlv));
        left_y = ladar_distance[left_count] * sin(theta+((left_count-50)*fenbianlv));

        left_xielv = (left_y - Ldist_r_y) / (left_x - Ldist_r_x);
        printf("此时left_xielv=%f,标准斜率=%f\n",left_xielv,xielv);
        if(abs(left_xielv-xielv) < xielv_threshold)
        {
            left_max_x = left_x;
            left_max_y = left_y;
            left_max_num = left_count;
        }
        else    //连续判断三次斜率差是否过大
        {
            printf("进入了左侧的第一次错误，此时left_count=%d",left_count);
            left_count++;
            left_x = ladar_distance[left_count] * cos(theta+((left_count-50)*fenbianlv));
            left_y = ladar_distance[left_count] * sin(theta+((left_count-50)*fenbianlv));

            left_xielv = (left_y - Ldist_r_y) / (left_x - Ldist_r_x);
            printf("此时left_xielv=%f,标准斜率=%f\n",left_xielv,xielv);
            if(abs(left_xielv-xielv) < xielv_threshold)
            {
                left_max_x = left_x;
                left_max_y = left_y;
                left_max_num = left_count;
            }
            else
            {
                printf("进入了左侧的第二次错误，此时left_count=%d",left_count);
                left_count++;
                left_x = ladar_distance[left_count] * cos(theta+((left_count-50)*fenbianlv));
                left_y = ladar_distance[left_count] * sin(theta+((left_count-50)*fenbianlv));

                left_xielv = (left_y - Ldist_r_y) / (left_x - Ldist_r_x);
                printf("此时left_xielv=%f,标准斜率=%f\n",left_xielv,xielv);
                if(abs(left_xielv-xielv) < xielv_threshold)
                {
                    left_max_x = left_x;
                    left_max_y = left_y;
                    left_max_num = left_count;
                } 
                else
                {
                    printf("进入了左侧的第三次错误，此时left_count=%d\n",left_count);
                    break;
                }
            }
        }
    }

    board_long=sqrt((right_max_x-left_max_x)*(right_max_x-left_max_x)
                    +(right_max_y-left_max_y)*(right_max_y-left_max_y));

    mid_x=ladar_distance[(left_max_num+right_max_num)/2]*cos(theta+((left_max_num+right_max_num-100)/2)*fenbianlv);
    mid_y=ladar_distance[(left_max_num+right_max_num)/2]*sin(theta+((left_max_num+right_max_num-100)/2)*fenbianlv);
    
    
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");    
    ros::init(argc,argv,"lwz_lidar_node1");
    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/scan",10,&LidarCallback);
    double the_x = 0.0;
    
    while(ros::ok())
    {
        // sleep(1);
        ros::param::get("/goal_laser",in_end_goal_e);
        ros::param::get("/the_x",the_x);

        while(!the_x)
        {
            //得到物品所对应的角度
            ros::param::get("/the_x",the_x);
        }

        range_x = round(the_x*909/360); //the_x的单位是°

        if(in_end_goal_e==true)
        {
            if((board_long>0.25) && (board_long<100))
            {   
                ros::param::set("/dist",float(ladar_distance[(left_max_num+right_max_num)/2]));
                ros::param::set("/dist_left",float(ladar_distance[left_max_num]));
                ros::param::set("/dist_right",float(ladar_distance[right_max_num]));
                ros::param::set("/left_max_num",50-left_max_num);
                ros::param::set("/right_max_num",right_max_num-50);
                printf("****************************************\n");
                printf("****************************************\n");
                ROS_INFO("测得的板长：%f\n",board_long);
                printf("标准斜率=%f\n",xielv);
                printf("直接测的中间值=%f，ladar_distance[50]=%f\n",Ldist,ladar_distance[50]);

                printf("fenbianlv=%f\n",fenbianlv);

                ROS_INFO("left_max_num=%d,right_max_num=%d\n",left_max_num,right_max_num);

                printf("theta=%f\n",theta);

                printf("50-left_max_num=%d\n",50-left_max_num);
                printf("(50-left_max_num)*fenbianlv=%f\n",(50-left_max_num)*fenbianlv);
                printf("(50-left_max_num)*0.006912195=%f\n",(50-left_max_num)*0.006912195);

                printf("theta+(50-left_max_num)*fenbianlv=%f\n",theta+(50-left_max_num)*fenbianlv);
                printf("theta-(right_max_num-50)*fenbianlv=%f\n",theta-(right_max_num-50)*fenbianlv);

                printf("right_distance=%f,left_distance=%f\n",ladar_distance[right_max_num],ladar_distance[left_max_num]);
                ROS_INFO("右点坐标：(%f,%f)",right_max_x,right_max_y);

                ROS_INFO("左点坐标：(%f,%f)",left_max_x,left_max_y);
                ROS_INFO("直接得到的板中心坐标：(%f,%f)\n",Ldist_mid_x,Ldist_mid_y);
                ROS_INFO("向两侧拓展得到的板中心坐标：(%f,%f)\n",mid_x,mid_y);
                printf("****************************************\n");
                printf("****************************************\n");
            }
            else 
            {
                printf("****************************************\n");
                printf("****************************************\n");
                ROS_INFO("测得的板长：%f\n",board_long);
                printf("标准斜率=%f\n",xielv);
                printf("直接测的中间值=%f，ladar_distance[50]=%f\n",Ldist,ladar_distance[50]);
                printf("fenbianlv=%f\n",fenbianlv);

                ROS_INFO("left_max_num=%d,right_max_num=%d\n",left_max_num,right_max_num);

                printf("theta=%f\n",theta);

                printf("50-left_max_num=%d\n",50-left_max_num);
                printf("(50-left_max_num)*fenbianlv=%f\n",(50-left_max_num)*fenbianlv);
                printf("(50-left_max_num)*0.006912195=%f\n",(50-left_max_num)*0.006912195);

                printf("theta+(50-left_max_num)*fenbianlv=%f\n",theta+(50-left_max_num)*fenbianlv);
                printf("theta-(right_max_num-50)*fenbianlv=%f\n",theta-(right_max_num-50)*fenbianlv);

                printf("right_distance=%f,left_distance=%f\n",ladar_distance[right_max_num],ladar_distance[left_max_num]);
                ROS_INFO("右点坐标：(%f,%f)",right_max_x,right_max_y);

                ROS_INFO("左点坐标：(%f,%f)",left_max_x,left_max_y);
                ROS_INFO("直接得到的板中心坐标：(%f,%f)\n",Ldist_mid_x,Ldist_mid_y);
                ROS_INFO("向两侧拓展得到的板中心坐标：(%f,%f)\n",(right_max_x+left_max_x)/2,(right_max_y+left_max_y)/2);
                printf("****************************************\n");
                printf("****************************************\n");

                if((range_x!=0)&&(350>Ldist*Ldist_l*Ldist_r>0)&&(in_end_goal_e==true))
                {
                    ros::param::set("/dist",float(Ldist));
                    ros::param::set("/dist_left",float(Ldist_l));
                    ros::param::set("/dist_right",float(Ldist_r));
                    ros::param::set("/left_max_num",6);
                    ros::param::set("/right_max_num",6);
                }
    
            }
        
        }
        

        ros::spinOnce();
       
    }
    return 0;
}