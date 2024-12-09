#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    publishing_cmd_vel = false;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;
    la_current_q.resize(7);
    ra_current_q.resize(7);
    la_voltage = 0;
    // la_voltage_bar = 0;
    ra_voltage = 0;
    //ra_voltage_bar = 0;
    la_current_cartesian.resize(6);
    ra_current_cartesian.resize(6);
    /* QString danger = QProgressBar::chunk
        {
            background-color: rgb(115, 210, 22);
            width: 20px;
        };
    QString safe= QProgressBar::chunk
        {
            background-color: rgb(204, 0, 0);
            width: 20px;
         }*/
    spr_recognized = "";
}
QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(30);
    pubCmdVel     = n->advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    pubGoalLoc    = n->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    
    pubTorso      = n->advertise<std_msgs::Float64>("/hardware/torso/goal_pose", 1);
    pubLaGoalQ    = n->advertise<std_msgs::Float64MultiArray>("/hardware/left_arm/goal_pose", 1);
    pubRaGoalQ    = n->advertise<std_msgs::Float64MultiArray>("/hardware/right_arm/goal_pose", 1);
    pubLaGoalTraj = n->advertise<trajectory_msgs::JointTrajectory>("/manipulation/la_q_trajectory",1);
    pubRaGoalTraj = n->advertise<trajectory_msgs::JointTrajectory>("/manipulation/ra_q_trajectory",1);
    pubHdGoalQ    = n->advertise<std_msgs::Float64MultiArray>("/hardware/head/goal_pose", 1);
    pubLaGoalGrip = n->advertise<std_msgs::Float64>("/hardware/left_arm/goal_gripper", 1);
    pubRaGoalGrip = n->advertise<std_msgs::Float64>("/hardware/right_arm/goal_gripper", 1);
    // subLaCurrentQ = n->subscribe("/hardware/left_arm/current_pose" , 1, &QtRosNode::callback_la_current_q, this);
    // subLaVoltage  = n->subscribe("/hardware/left_arm_voltage",1, &QtRosNode::callback_la_voltage,this);
    // subRaCurrentQ = n->subscribe("/hardware/right_arm/current_pose", 1, &QtRosNode::callback_ra_current_q, this);
    // subRaVoltage  = n->subscribe("/hardware/right_arm_voltage",1, &QtRosNode::callback_ra_voltage,this);

    pubSpeechGen       = n->advertise<sound_play::SoundRequest>("/hri/speech_generator", 1);
      
    pubLegFinderEnable     = n->advertise<std_msgs::Bool>("/hri/leg_finder/enable", 1);
    pubFollowHumanEnable   = n->advertise<std_msgs::Bool>("/hri/human_following/enable", 1);
    
    int pub_zero_counter = 5;
    while(ros::ok() && !this->gui_closed)
    {
        if(publishing_cmd_vel)
        {
            pubCmdVel.publish(cmd_vel);
            pub_zero_counter = 5;
        }
        else if(--pub_zero_counter > 0)
        {
            if(pub_zero_counter <= 0)
                pub_zero_counter = 0;
            pubCmdVel.publish(cmd_vel);
        }
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::publish_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    pubCmdVel.publish(cmd_vel);
}

void QtRosNode::publish_goal_location(float x, float y, float a)
{
    geometry_msgs::PoseStamped msg;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.w = cos(a/2);
    msg.pose.orientation.z = sin(a/2);
    pubGoalLoc.publish(msg);
}

bool QtRosNode::call_known_location(std::string location, float& x, float& y, float& a)
{
    // planning_msgs::GetLocation srv;
    // srv.request.name = location;
    // if(!cltKnownLoc.call(srv))
    //     return false;
    // x = srv.response.location.pose.position.x;
    // y = srv.response.location.pose.position.y;
    // a = atan2(srv.response.location.pose.orientation.z, srv.response.location.pose.orientation.w)*2;
    // return true;
}

void QtRosNode::start_publishing_cmd_vel(float linear_frontal, float linear_lateral, float angular)
{
    cmd_vel.linear.x = linear_frontal;
    cmd_vel.linear.y = linear_lateral;
    cmd_vel.angular.z = angular;
    publishing_cmd_vel = true;
}

void QtRosNode::stop_publishing_cmd_vel()
{
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    publishing_cmd_vel = false;
}

void QtRosNode::get_robot_pose(float& robot_x, float& robot_y, float& robot_a)
{
    tf::StampedTransform t;
    tf::Quaternion q;
    tf_listener.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
    tf_listener.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
}

void QtRosNode::publish_torso_position(float tr)
{
    std_msgs::Float64 msg;
    msg.data = tr;
    pubTorso.publish(msg);
}

void QtRosNode::publish_la_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(7);
    msg.data[0] = a1;
    msg.data[1] = a2;
    msg.data[2] = a3;
    msg.data[3] = a4;
    msg.data[4] = a5;
    msg.data[5] = a6;
    msg.data[6] = a7;
    pubLaGoalQ.publish(msg);
}

void QtRosNode::publish_ra_goal_angles(float a1, float a2, float a3, float a4, float a5, float a6, float a7)
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(7);
    msg.data[0] = a1;
    msg.data[1] = a2;
    msg.data[2] = a3;
    msg.data[3] = a4;
    msg.data[4] = a5;
    msg.data[5] = a6;
    msg.data[6] = a7;
    pubRaGoalQ.publish(msg);
}

void QtRosNode::publish_la_goal_trajectory(trajectory_msgs::JointTrajectory Q)
{
    pubLaGoalTraj.publish(Q);
}

void QtRosNode::publish_ra_goal_trajectory(trajectory_msgs::JointTrajectory Q)
{
    pubRaGoalTraj.publish(Q);
}

void QtRosNode::publish_la_grip_angles(float a)
{
    std_msgs::Float64 msg;
    msg.data = a;
    pubLaGoalGrip.publish(msg);
}

void QtRosNode::publish_ra_grip_angles(float a)
{
    std_msgs::Float64 msg;
    msg.data = a;
    pubRaGoalGrip.publish(msg);
}

void QtRosNode::publish_head_angles(double pan, double tilt)
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = pan;
    msg.data[1] = tilt;
    pubHdGoalQ.publish(msg);
}


void QtRosNode::say(std::string text_to_say)
{
    sound_play::SoundRequest msg;
    msg.sound   = -3;
    msg.volume  = 1.0;
    msg.command = 1;
    msg.arg     = text_to_say;
    msg.arg2    = "voice_cmu_us_slt_arctic_hts";
    pubSpeechGen.publish(msg);
}


void QtRosNode::publish_enable_human_pose_detection(bool enable)
{
    std_msgs::Bool msg;
    msg.data = enable;
    pubHumanPoseEnable.publish(msg);
}

void QtRosNode::publish_enable_human_following(bool enable)
{
    std_msgs::Bool msg;
    msg.data = enable;
    pubLegFinderEnable.publish(msg);
    pubFollowHumanEnable.publish(msg);
}
