#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// double Ldist = -1;
// double Ldist_l = -1;
// double Ldist_r = -1;
// int range_x = 0 ;

// int num=0;

// bool begin_detect_left=false;
// bool begin_detect_right=false;
// bool begin_detect_front=false;

// float min_left_dis=0.0;
// float min_right_dis=0.0;
// float min_front_dis=0.0;
// float min_forward=0.0;



static double Ldist = -1;
static double Ldist_l = -1;
static double Ldist_r = -1;
static int range_x = 0 ;

double fenbianlv = 0.006912195; //360/909*PI/180

bool begin_detect_left=false;
bool begin_detect_right=false;
bool begin_detect_front=false;

double car_left_dis[455]={0};
double car_right_dis[456]={0};
double car_front_dis[228]={0};

double min_left_dis=10;
double min_right_dis=10;
double min_front_dis=10;

double left_dis=10;
double right_dis=10;
double front_dis=10;

static int num=0;





void LidarCallback(const sensor_msgs::LaserScan msg)
{

    Ldist = msg.ranges[455+range_x];
    Ldist_l = msg.ranges[455+range_x+6];
    Ldist_r = msg.ranges[455+range_x-6];

    if(begin_detect_front==true)
    {
        min_front_dis=10;   //初始化一下
        //获取雷达的341-568的数据位，中间为455，共228个数据
        car_front_dis[0]=msg.ranges[341];   //car_front_dis以车头向右45°的朝向为0，逆时针旋转增加，以车头向左45°的朝向为227
        for(int front_count=1 ; front_count<228 ; front_count++)
        {
            if(msg.ranges[341+front_count]>10)
            {
                car_front_dis[front_count]=car_front_dis[front_count-1];
            }
            else
            {
                car_front_dis[front_count]=msg.ranges[341+front_count];
            }
        }    
            for(int front_count=0 ; front_count<228 ; front_count++)
            {
                if((car_front_dis[front_count]*sin(abs(front_count-114)*fenbianlv))<0.15)
                {
                front_dis= car_front_dis[front_count]*cos(abs(front_count-114)*fenbianlv);
                if(front_dis<min_front_dis)
                {
                    min_front_dis=front_dis;
                }
                }
            }

        // ros::param::set("/min_front_dis",min_front_dis);
        // ros::param::set("/begin_detect_front",false);
        // begin_detect_front=false;
    }
    if(begin_detect_left==true)
    {
        printf("已进入左侧测距\n");
        min_left_dis=10;   //初始化一下

        // 909/4=227.25 455-227=228 455+227=682
        //左侧 455-909,中间是682，共455个数据
        car_left_dis[0]=msg.ranges[455];    //car_left_dis以车头朝前为0，逆时针旋转增加，车尾朝向为455
        for(int left_count=1 ; left_count<455 ; left_count++)
        {
            if(msg.ranges[455+left_count]>10)
            {
                car_left_dis[left_count]=car_left_dis[left_count-1];
            }
            else
            {
                car_left_dis[left_count]=msg.ranges[455+left_count];
            }
        }

        printf("已获取到左侧的雷达数据\n");

        for(int left_count=0 ; left_count<227 ; left_count++)  
        {
            if((car_left_dis[left_count]*sin(abs(left_count-227)*fenbianlv))<0.15)  //0.33是车长
            {
                left_dis=car_left_dis[left_count]*cos(abs(left_count-227)*fenbianlv);
                if(left_dis<min_left_dis)
                {
                    min_left_dis=left_dis;
                }
            }
        }


        for(int left_count=228 ; left_count<455 ; left_count++)   //454
        {
            if((car_left_dis[left_count]*sin(abs(left_count-227)*fenbianlv))<0.33)  //0.33是车长
            {
                left_dis=car_left_dis[left_count]*cos(abs(left_count-227)*fenbianlv);
                if(left_dis<min_left_dis)
                {
                    min_left_dis=left_dis;
                    printf("min_left_dis=%f\n",min_left_dis);
                }
            }
        }

        printf("已获取到min_left_dis=%f\n",min_left_dis);

        // ros::param::set("/min_left_dis",min_left_dis);
        // ros::param::set("/begin_detect_left",false);
        // begin_detect_left=false;
    }
    if(begin_detect_right==true)
    {
        min_right_dis=10;   //初始化一下

        // 909/4=227.25 455-227=228 455+227=682
        //右侧0-455，中间是228，共456个数据
        car_right_dis[0]=msg.ranges[0];     //car_left_dis以车尾朝向为0，逆时针旋转增加，车头朝向为455
        for(int right_count=1 ; right_count<456 ; right_count++)
        {
            if(msg.ranges[right_count]>10)
            {
                car_right_dis[right_count]=car_right_dis[right_count-1];
            }
            else
            {
                car_right_dis[right_count]=msg.ranges[right_count];
            }
        }

        for(int right_count=0 ; right_count<228 ; right_count++)
        {
            if((car_right_dis[right_count]*sin(abs(right_count-228)*fenbianlv))<0.33)  //0.33是车长
            {
                right_dis=car_right_dis[right_count]*cos(abs(right_count-228)*fenbianlv);
                if(right_dis<min_right_dis)
                {
                    min_right_dis=right_dis;
                }
            }
        }
        for(int right_count=229 ; right_count<456 ; right_count++)
        {
            if((car_right_dis[right_count]*sin(abs(right_count-228)*fenbianlv))<0.15)  //0.33是车长
            {
                right_dis=car_right_dis[right_count]*cos(abs(right_count-228)*fenbianlv);
                if(right_dis<min_right_dis)
                {
                    min_right_dis=right_dis;
                }
            }
        }

        // ros::param::set("/min_right_dis",min_right_dis);
        // ros::param::set("/begin_detect_right",false);
        // begin_detect_right=false;
    }
    
    // ROS_INFO("前方距离为：%f 米", Ldist);
}

int main(int argc,char *argv[])
{
    setlocale(LC_ALL,"");    
    ros::init(argc,argv,"lidar_node5");
    ros::NodeHandle n;
    ros::Subscriber lidar_sub = n.subscribe("/scan",10,&LidarCallback);
    bool in_end_goal_e = false ;
    double the_x = 0.0;

    int error=0;//error为雷达偏移量

    

    while(ros::ok())
    {
        // sleep(1);
        ros::param::get("/goal_laser",in_end_goal_e);      
        
        while(!the_x)
        {
            ros::param::get("/the_x",the_x);
            
            ros::param::get("/begin_detect_left",begin_detect_left);
            ros::param::get("/begin_detect_right",begin_detect_right);
            ros::param::get("/begin_detect_left",begin_detect_left);
            if(begin_detect_left||begin_detect_right||begin_detect_front)
            {
                if(begin_detect_left&&(min_left_dis<5.0&&min_left_dis>0.0))
                {
                    ros::param::set("/min_left_dis",min_left_dis);
                }
                if(begin_detect_right&&(min_right_dis<5.0&&min_right_dis>0.0))
                {
                    ros::param::set("/min_right_dis",min_right_dis);
                }
                if(begin_detect_front&&(min_front_dis<5.0&&min_front_dis>0.0))
                {
                    ros::param::set("/min_front_dis",min_front_dis);
                }
                
                ros::spinOnce();
            }
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