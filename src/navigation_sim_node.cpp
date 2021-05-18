#include <boost/bind.hpp> // timer
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>  //move base goal msg type
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>  //cmd_vel
#include <ros_acml/isReached.h>
#include <ros_acml/acml_goal.h>     //5_17

#define PI 3.141592
#define StopBand 0.04    //0.04
#define StopBand_Narrow 0.04
#define StopBand_Wide 1
#define StopBandAngle 0.2

#define NOTYET 0
#define CLOSE 1
#define ARRIVE 2

//sub
ros_acml::acml_goalConstPtr robot_goal;     //5_17
geometry_msgs::TwistConstPtr robot_pose;

//global var - gain
// 0.8 , 1.5 ,-1.4
float k_rho = 0.35;  //0.3
float k_alpha = 2;  //1.45
float k_beta = -1.5;   //-1.3

//global var - cmd
float v = 0.0;
float omega = 0.0; 

//global var - rho,alpha,beta
float rho = 0.0;
float alpha = 0.0;
float beta = 0.0;

//global var - other
float delta_x = 0.0;
float delta_y = 0.0;
float theta = 0.0;
float theta_goal = 0.0;

//global var - test
float theta_tmp = 0.0;
float theta_delta_p = 0.0;
float theta_delta_n = 0.0;
float theta_delta = 0.0;

void poseCallback(const geometry_msgs::TwistConstPtr &pose)
{
  robot_pose = pose;
}

void goalCallback(const ros_acml::acml_goalConstPtr &goal)  //5_17
{
  robot_goal = goal;
}

int8_t hasReachedGoal(void)
{
    float result = sqrtf(delta_x*delta_x + delta_y*delta_y);
    if (result <= StopBand_Narrow)
        return ARRIVE;

    else if(result <= StopBand_Wide)
        return CLOSE;

    return NOTYET;
}

float orientation2theta(void)
{
    /*tf formula*/
    float x = robot_goal->pose.orientation.x;
    float y = robot_goal->pose.orientation.y;
    float z = robot_goal->pose.orientation.z;
    float w = robot_goal->pose.orientation.w;

    return atan2(2*(w*z+x*y),1-2*(z*z+y*y));
}

void updateState(void)
{
    delta_x = robot_goal->pose.position.x-robot_pose->linear.x;
    delta_y = robot_goal->pose.position.y-robot_pose->linear.y;
    theta = robot_pose->angular.z;
    theta_goal = orientation2theta();//tf from qutenion

    /*test*/
    theta_tmp = theta;
    while(theta_tmp > PI)
    {
        theta_tmp -= 2*PI;
    }
    theta_delta = theta_goal - theta_tmp;


    rho = sqrt( pow(delta_x,2) + pow(delta_y,2) );
    alpha = -theta_tmp + atan2(delta_y,delta_x);
    beta = theta_goal - theta_tmp - alpha;

    while(alpha > PI)
    {
        alpha -= 2*PI;
    }
    while(alpha <= -PI)
    {
        alpha += 2*PI;
    }

    while(beta > PI)
    {
        beta -= 2*PI;
    }
    while(beta <= -PI)
    {
        beta += 2*PI;
    }
}

void controller(ros::Publisher reach_pub)
{
    updateState();
    /*rho-alpha-beta controller*/
    /*tell a_star arrived*/
    ros_acml::isReached myReached;
    int8_t r = hasReachedGoal();
    if(r>NOTYET)
    {
        /*abort stop*/
        // v = 0;
        // omega = 0;
        v = k_rho * rho;
        omega = k_alpha * alpha + k_beta * beta;
        /*pub here*/
        if(r == CLOSE)
        {
            myReached.Reached = CLOSE;
            ROS_INFO("Close to the Goal\n");
        }
        else if(r == ARRIVE)
        {
            myReached.Reached = CLOSE;
            ROS_INFO("Reach to the Goal\n");
        }
            
    }
    else
    {
        /*update v omega*/
        v = k_rho * rho;
        omega = k_alpha * alpha + k_beta * beta;
        myReached.Reached = NOTYET;
    }
    reach_pub.publish(myReached);
}

void commandCar(ros::Publisher vel_pub)
{
    //pub
    geometry_msgs::Twist vel_cmd;

    vel_cmd.linear.x = v;
    vel_cmd.angular.z = omega;
    vel_pub.publish(vel_cmd);
}

void printGoalInfo()
{
    ROS_INFO("goal:%f,%f,%f\n",robot_goal->pose.position.x,robot_goal->pose.position.y,theta_goal);
}

void printPoseInfo()
{
    ROS_INFO("pose:%f,%f,%f\n",robot_pose->linear.x,robot_pose->linear.y,theta_tmp);
}

void printDeltaInfo()
{
    ROS_INFO("delta:%f,%f,%f\n",delta_x,delta_y,theta_delta);
}

void printCmdInfo()
{
    ROS_INFO("cmd:%f,%f\n",v,omega);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Navigation_Sim");
    ros::NodeHandle nh;
    ros::Subscriber robot_pose_sub = nh.subscribe("/robot_pose", 1, poseCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback);   /*5_17*/
    //ros::Subscriber goal_sub = nh.subscribe("/goal", 1, goalCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Publisher reach_pub = nh.advertise<ros_acml::isReached>("/isReached",100);
    ros::Rate rate(10);
    sleep(1);

    while (ros::ok())
    {
        
        /*check goal is not empty , or it will cause px!=0 err*/
        if(!robot_goal)
        {
            ros_acml::isReached myReached;
            myReached.Reached = false;
            reach_pub.publish(myReached);
            
        }
            
        else
        {
            controller(reach_pub);
            commandCar(vel_pub);
            printGoalInfo();
            printPoseInfo();
            printDeltaInfo();
            printCmdInfo();
            ROS_INFO("\n");
        }
        ros::spinOnce();         
        rate.sleep();
    }

    return 0;

}


// msg 
/*
geometry_msgs::PoseStamped
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    geometry_msgs/Pose pose
        geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
        geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
geometry_msgs::Twist
    geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
    geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
*/