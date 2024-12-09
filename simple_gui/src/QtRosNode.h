#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/GetPlan.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "sound_play/SoundRequest.h"
#include "tf/transform_listener.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::Publisher pubCmdVel;
    ros::Publisher pubGoalLoc;
    
    ros::Publisher pubTorso;
    ros::Publisher pubLaGoalQ;
    ros::Publisher pubRaGoalQ;
    ros::Publisher pubLaGoalTraj;
    ros::Publisher pubRaGoalTraj;
    ros::Publisher pubHdGoalQ;
    ros::Publisher pubLaGoalGrip;
    ros::Publisher pubRaGoalGrip;
    ros::Publisher pubSpeechGen;
    ros::Publisher pubFakeSpeechRecog;
    ros::Publisher pubHumanPoseEnable;
    ros::Publisher pubLegFinderEnable;
    ros::Publisher pubFollowHumanEnable;
    ros::Publisher pubTakeObject;

    ros::Subscriber subLaCurrentQ;
    ros::Subscriber subLaVoltage;
    ros::Subscriber subRaCurrentQ;
    ros::Subscriber subRaVoltage;
    ros::Subscriber subRecogSpeech;
    
    tf::TransformListener tf_listener;
    
    geometry_msgs::Twist cmd_vel;
    bool publishing_cmd_vel;
    bool gui_closed;
    std::vector<double> la_current_q;
    std::vector<double> ra_current_q;
    std::vector<double> la_current_cartesian;
    std::vector<double> ra_current_cartesian;
    double la_voltage;
    double ra_voltage;
    std::string spr_recognized;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void publish_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void publish_goal_location(float x, float y, float a);
    bool call_known_location(std::string location, float& x, float& y, float& a);
    void start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular);
    void stop_publishing_cmd_vel();
    void get_robot_pose(float& robot_x, float& robot_y, float& robot_a);

    void publish_torso_position(float tr);
    void publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7);
    void publish_la_goal_trajectory(trajectory_msgs::JointTrajectory Q);
    void publish_ra_goal_trajectory(trajectory_msgs::JointTrajectory Q);
    void publish_la_grip_angles(float a);
    void publish_ra_grip_angles(float a);
    void publish_head_angles(double pan, double tilt);
    // void callback_la_current_q(const std_msgs::Float64MultiArray::ConstPtr& msg);
    // void callback_la_voltage(const std_msgs::Float64::ConstPtr& msg);
    // void callback_ra_current_q(const std_msgs::Float64MultiArray::ConstPtr& msg);
    // void callback_ra_voltage(const std_msgs::Float64::ConstPtr& msg);
    
    void say(std::string text_to_say);
    
    void publish_enable_human_pose_detection(bool enable);
    void publish_enable_human_following(bool enable);
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
