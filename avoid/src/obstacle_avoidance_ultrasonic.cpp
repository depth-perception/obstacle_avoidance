/**************************************************************
Author:double yang

Time:2018.11.24

FileName: obstacle_avoidance_ultrasonic.cpp

Instruction:this code is used to avoid obstacle by ultrasonic, there is 6 Ultrasonic sensors in npu robots,

i just uses the 1,3,5 sensors.

**************************************************************/


#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

#include "std_msgs/Float32MultiArray.h"

#define  setbit(x, y)  x|=(1<<y)

#define  clrbit(x, y)  x&=~(1<<y)

#define  STATUS_A   0x04  // v x x
#define  STATUS_B   0x02  // x v x
#define  STATUS_C   0x01  // x x v
#define  STATUS_D   0x07  // v v v
#define  STATUS_E   0x06  // v v x
#define  STATUS_F   0x03  // x v v
#define  STATUS_G   0x05  // v x v

geometry_msgs::Twist twist_cmd;

ros::Publisher twist_pub;

//safe distance
const double warn_range =30;

double default_period_hz = 10;

double default_linear_x = 0.2;

double default_yaw_rate = 0.2;

//save three sonar value
double range_array[3];

void sonar_callback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    ROS_INFO("get ultrasonic data ");
    
    range_array[0] = msg->data[2];
    range_array[1] = msg->data[3];
    range_array[2] = msg->data[4];
}

void publishTwistCmd(double linear_x, double angular_z)
{
    twist_cmd.linear.x = linear_x;
    twist_cmd.linear.y = 0.0;
    twist_cmd.linear.z = 0.0;

    twist_cmd.angular.x = 0.0;
    twist_cmd.angular.y = 0.0;
    twist_cmd.angular.z = angular_z;

    twist_pub.publish(twist_cmd);
}


void checkSonarRange(double sonar_l, double sonar_f, double sonar_r)
{
   unsigned char flag = 0;

   if(sonar_l < warn_range)
   {
       setbit(flag, 2);
   }
   else
   {
       clrbit(flag, 2);
   }

   if(sonar_f < warn_range)
   {
       setbit(flag, 1);
   }
   else
   {
       clrbit(flag, 1);
   }

   if(sonar_r < warn_range)
   {
       setbit(flag, 0);
   }
   else
   {
       clrbit(flag, 0);
   }

   ROS_INFO("CheckSonarRange get status:0x%x", flag);
   switch(flag)
   {
    case STATUS_A:
        ROS_WARN("left warn,turn right");
        publishTwistCmd(0, -default_yaw_rate);
        break;

    case STATUS_B:
        ROS_WARN("front warn, left and right ok, compare left and right value to turn");
        if(sonar_l > sonar_r)
        {
            ROS_WARN("turn left");
            publishTwistCmd(0, default_yaw_rate);
        }
        else
        {
            ROS_WARN("turn right");
            publishTwistCmd(0, -default_yaw_rate);
        }
        break;

    case STATUS_C: //turn left
        ROS_WARN("left ok, front ok, right warn, turn left");
        publishTwistCmd(0, default_yaw_rate);
        break;

    case STATUS_D:
        ROS_WARN("left,front,right all warn, turn back");
        publishTwistCmd(0, 10*default_yaw_rate);
        break;

    case STATUS_E:
        ROS_WARN("left warn, front warn, right ok, turn right");
        publishTwistCmd(0, (-default_yaw_rate*2));
        break;

    case STATUS_F:
        ROS_WARN("left ok, front warn, right warn, turn left");
        publishTwistCmd(0, (default_yaw_rate*2));
        break;

    case STATUS_G:
        ROS_WARN("left and right warn, front ok, speed up");
        publishTwistCmd(2*default_linear_x, 0);
        break;

    default: //go forward straight line
        publishTwistCmd(default_linear_x, 0);
        break;
   }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ultrasonic_obstacle_avoidance_node");

    ros::NodeHandle handle;

    ros::Rate loop_rate = default_period_hz;

    ros::Subscriber sub_sonar0 = handle.subscribe("/ultrasonic_distance", 50, sonar_callback);


    twist_pub = handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    while(ros::ok())
    {
       checkSonarRange(range_array[0], range_array[1], range_array[2]);

       ros::spinOnce();

       loop_rate.sleep();
    }

    return 0;
}
