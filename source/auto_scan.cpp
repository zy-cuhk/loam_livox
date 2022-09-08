#include <iostream>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <string>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <fstream>
#include <assert.h>
#include <jsoncpp/json/json.h> 
#include <stdlib.h> 
#include <time.h>
#include <stdio.h>
#include "std_msgs/String.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <geometry_msgs/Pose.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/math/Utils.h>
#include <octomap/OcTreeBase.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef octomap::point3d point3d;

#define pi 3.1415926
vector<double> viewpoint_position(7);
const double CLK_TCK = 1000.0;

void aubo_fake_MoveJ(double &angle1, bool &success,
                    moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    std::vector<double> joint_values;
    joint_values = move_group_interface.getCurrentJointValues();
    joint_values[0]=angle1/180.0*M_PI;
    move_group_interface.setJointValueTarget(joint_values);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    trajectory=my_plan.trajectory_;
    move_group_interface.execute(trajectory);
}

void getRobotstates(std::vector<double> &joint_values,
                    moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
    joint_values = move_group_interface.getCurrentJointValues();
    // for (std::size_t i = 0; i < 6; ++i)
    // {
    //     ROS_INFO("Joint: %f",  joint_values[i]);
    // }
}

void generateArmrotations(std::vector<double> &stages, double &step, 
                          moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
    for (size_t i = 0; i < ceil(174*2/step)+1; i++)
    {
        std::vector<double> joint_values;
        getRobotstates(joint_values, move_group_interface);
        if(joint_values[0] < 0)
        {
            double stage = -174.0 + i*step;
            if(stage > 174)
            {
                stage = 174;    
            }
            stages.push_back(stage);
        }
        else
        {
            double stage = 174.0 - i*step;
            if(stage < -174)
            {
                stage = -174;    
            }
            stages.push_back(stage);
        }
    }
    std::cout<<"the stage planning results are: ";
    for (int i=0; i<stages.size(); i++)
    {
        std::cout<<stages[i]/180.0*M_PI<<",";
    }    
    std::cout<<std::endl;
}

void viewpoint_callback( const geometry_msgs::PoseStamped::ConstPtr& msg )
{
    if (msg!=NULL)
    {   
        // std::cout<<msg->pose.position.x<<","<<msg->pose.position.y<<","<<msg->pose.position.z<<std::endl;
        viewpoint_position[0]=msg->pose.position.x;
        viewpoint_position[1]=msg->pose.position.y;
        viewpoint_position[2]=msg->pose.position.z;
        viewpoint_position[3]=msg->pose.orientation.x;
        viewpoint_position[4]=msg->pose.orientation.y;
        viewpoint_position[5]=msg->pose.orientation.z;
        viewpoint_position[6]=msg->pose.orientation.w;
        std::cout<<"the received viewpoint position is: "<<std::endl;
        for (int i=0; i<7; i++)
            std::cout<<viewpoint_position[i]<<",";
        std::cout<<std::endl;
        ros::param::set("pose_receive",true);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aubo_moveit_gazebo");
    ros::NodeHandle nh;
    ros::Subscriber viewpoint_sub = nh.subscribe("viewpoint_position", 10, viewpoint_callback);

    ros::Time t1 = ros::Time::now();
    time_t start = std::time(NULL);

    // construct the moveit object for manipulator 
    ros::AsyncSpinner spinner(1);
    spinner.start();
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    bool success;

    ros::Time t2 = ros::Time::now();
    std::cout << "The time cost of setting up moveit environment is : " << (t2-t1).toNSec()/1000000000.0<<" s"<< std::endl;

    // std::vector<double> aabb;
    // robot_state->computeAABB(aabb);
    // for (int i=0; i<aabb.size(); i++)
    //     std::cout<<aabb[i]<<" ";
    // default_joint_position = {174*M_PI/180, -0.24435, 2.75, -0.07, -1.4835, -1.57};
    // default_joint_position = {2.96751, -0.230594, 2.71129, -0.199712, -1.74488, -1.5708};

    double manipulatorjoints_vel, manipulatorjoints_maxvel;
    double scale_factor = 0.05;
    int waypoints_num;
    
    std::vector<double> default_joint_position1(6);
    default_joint_position1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;

    // move_group_interface.setStartStateToCurrentState();
    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    move_group_interface.setJointValueTarget(default_joint_position1);
    move_group_interface.setMaxAccelerationScalingFactor(scale_factor);
    move_group_interface.setMaxVelocityScalingFactor(scale_factor);

    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    trajectory = my_plan.trajectory_;

    double manipulator_duration=0;
    manipulatorjoints_maxvel = 0;
    waypoints_num = trajectory.joint_trajectory.points.size();
    for (int i=0; i<waypoints_num; i++){
        for (int j = 0; j<6; j++){
            manipulatorjoints_vel = abs(trajectory.joint_trajectory.points[i].positions[j]);
            if (manipulatorjoints_vel>manipulatorjoints_maxvel)
                manipulatorjoints_maxvel = manipulatorjoints_vel;
        }
        manipulator_duration =trajectory.joint_trajectory.points[i].time_from_start.toSec();
    }
    ROS_INFO("The maximal manipulator joint velocity is: %f m/s", manipulatorjoints_maxvel);
    cout<<"the duration of trajectory is: "<<manipulator_duration<<" s"<<endl;


    ros::Time t3 = ros::Time::now();
    std::cout << "the time cost of planning the first manipulator trajectory is: " << (t3-t2).toNSec()/1000000000.0<<" s"<< std::endl;

    time_t end = std::time(NULL); 
    double elapse = end - start;
    
    std::cout << "the time cost of setting up moveit environment and planning the first trajectory is: " << elapse<<" s" << std::endl;
    
    time_t start1 = std::time(NULL);
    // move_group_interface.execute(trajectory);   
    ROS_INFO_NAMED("planning result is: %s", success ? "planning result is successful" : "planning result is failed");
    time_t end1 = std::time(NULL); 
    double elapse1 = end1 - start1;
    std::cout << "the time cost of executing manipulator trajectory is: " << elapse1 << std::endl;

    ros::Time t4 = ros::Time::now();
    std::cout << "the time cost of executing manipulator trajectory is:  " << (t4-t3).toNSec()/1000000000.0<<" s"<< std::endl;

    std::vector<double> default_joint_position2(6);
    default_joint_position2 = {-1.5708, -0.230594, 2.71129, -0.199712, -1.74488, -1.5708};


    move_group_interface.setStartState(*move_group_interface.getCurrentState());
    move_group_interface.setJointValueTarget(default_joint_position2);
    move_group_interface.setMaxAccelerationScalingFactor(scale_factor);
    move_group_interface.setMaxVelocityScalingFactor(scale_factor);
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    trajectory = my_plan.trajectory_;
    
    cout<<"the duration of trajectory is: "<<trajectory.joint_trajectory.header.stamp.toSec()<<endl;

    manipulatorjoints_maxvel = 0;
    waypoints_num = trajectory.joint_trajectory.points.size();
    for (int i=0; i<waypoints_num; i++){
        for (int j = 0; j<6; j++){
            manipulatorjoints_vel = abs(trajectory.joint_trajectory.points[i].positions[j]);
            if (manipulatorjoints_vel>manipulatorjoints_maxvel)
                manipulatorjoints_maxvel = manipulatorjoints_vel;
        }
    }
    ROS_INFO("The maximal manipulator joint velocity is: %f m/s", manipulatorjoints_maxvel);

    // move_group_interface.execute(trajectory);  
    ros::Rate loop_rate(0.2);
    // double step;
    // int count = 0;
    // bool scan_start;
    // std::vector<double> stages;
    // nh.param<double>("scan_step", step, 20.0);

    // // the first manipulator motion at the original position of mobile base 
    // ros::param::set("scan_start",true);
    // generateArmrotations(stages, step, move_group_interface);
    // while (ros::ok)
    // {
    //     std::cout << "------------------------------------" << std::endl;
    //     // ros::param::set("global_scanning",true);
    //     ros::param::set("scan_pend",false);
    //     aubo_fake_MoveJ(stages[count], success, move_group_interface);
    //     std::vector<double> joint_values1;
    //     getRobotstates(joint_values1, move_group_interface);
    //     if(std::fabs(joint_values1[0]-stages[count]*M_PI/180) < 0.001) 
    //     {
    //         std::cout << "arived !!! joint is: " << joint_values1[0] << std::endl;
    //         ros::param::set("scan_pend",true);
    //         count++;
    //         sleep(2);
    //     }
    //     if(count == stages.size())
    //     {
    //         // ros::param::set("global_scanning",false);
    //         ros::param::set("scan_pend",false);
    //         ros::param::set("scan_start",false);
    //         count = 0;
    //         break;
    //     }
    // }


    // // construct the movebase object for mobile base
    // MoveBaseClient ac("move_base", true);
    
    // bool receive_viewpoint, reach_goal;
    // // the motion of mobile base and manipulator 
    // while (ros::ok())
    // {   
    //     // firstly: the motion of mobile base
    //     ros::param::get("pose_receive", receive_viewpoint);
    //     if (receive_viewpoint==true)
    //     {
    //         ac.cancelAllGoals();
    //         while(!ac.waitForServer(ros::Duration(5.0)))
    //         {
    //             ROS_INFO("Waiting for the move_base action server to come up");
    //         }
    //         move_base_msgs::MoveBaseGoal goal;
    //         goal.target_pose.header.frame_id = "map";
    //         goal.target_pose.header.stamp = ros::Time::now();
    //         goal.target_pose.pose.position.x = viewpoint_position[0];
    //         goal.target_pose.pose.position.y = viewpoint_position[1];
    //         goal.target_pose.pose.position.z = viewpoint_position[2];
    //         goal.target_pose.pose.orientation.x = viewpoint_position[3];
    //         goal.target_pose.pose.orientation.y = viewpoint_position[4];
    //         goal.target_pose.pose.orientation.z = viewpoint_position[5];
    //         goal.target_pose.pose.orientation.w = viewpoint_position[6];
    //         ac.sendGoal(goal);
    //         ac.waitForResult(ros::Duration(120.0));
    //         if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //         {
    //             ROS_INFO("Successfully drive to the goal");
    //             for (int i=0; i<7; i++)
    //                 std::cout<<viewpoint_position[i]<<",";
    //             std::cout<<"-----------------------------------------"<<std::endl;
    //             ros::param::set("scan_start",true);
    //             ros::param::set("pose_receive",false);
    //             reach_goal=true;
    //         }
    //         else
    //         {
    //             ROS_WARN("Failed to drive to the goal");
    //             reach_goal=false;
    //         }
    //     }
    //     if (reach_goal=false)
    //     {
    //         break;
    //     }
    //     // secondly, the rotation planning and motion of manipulator 
    //     stages.clear();
    //     ros::param::get("scan_start",scan_start);
    //     if (scan_start)
    //     {
    //         generateArmrotations(stages, step, move_group_interface);
    //     }
    //     while(scan_start)
    //     {
    //         std::cout << "------------------------------------" << std::endl;
    //         // ros::param::set("global_scanning",true);
    //         ros::param::set("scan_pend",false);
    //         aubo_fake_MoveJ(stages[count], success, move_group_interface);
    //         std::vector<double> joint_values1;
    //         getRobotstates(joint_values1, move_group_interface);
    //         if(std::fabs(joint_values1[0]-stages[count]*M_PI/180) < 0.001) 
    //         {
    //             std::cout << "arived !!! joint is: " << joint_values1[0] << std::endl;
    //             sleep(0.25);
    //             ros::param::set("scan_pend",true);
    //             count++;
    //             sleep(2);
    //             // std::cout << "next !!!" << std::endl;
    //         }
    //         if(count == stages.size())
    //         {
    //             // ros::param::set("global_scanning",false);
    //             ros::param::set("scan_pend",false);
    //             ros::param::set("scan_start",false);
    //             count = 0;
    //             break;
    //         }
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    
    return 0;
}


// 1. FCL (flexible collision library) to detect the self-occlusion problem (can be transferred into a self-collision problem
// thus, we can use the moveit to address this problem)
// 2. moveit to generate collision-free trajectory (the collision and self-collision detection are contained)
// 3. aubo kinematic library to generate the best joint solution

