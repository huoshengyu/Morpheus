#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/PlanningScene.h>

// Define a callback to update the Planning Scene when a Planning Scene Message is received
void planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr& msg, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
{
    // Lock the Planning Scene Monitor to update the Planning Scene
    planning_scene_monitor->getPlanningScene()->setCurrentState(msg->robot_state);
    planning_scene_monitor->getPlanningScene()->usePlanningSceneMsg(*msg);
}

int subscriber(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "planning_scene_monitor_subscriber");
    ros::NodeHandle nh;

    // Create a PlanningSceneMonitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
        new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    // Start the PlanningSceneMonitor
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startSceneMonitor();

    // Create a subscriber to receive Planning Scene Messages
    ros::Subscriber planning_scene_subscriber = nh.subscribe<moveit_msgs::PlanningScene>(
        "/planning_scene",
        1,
        boost::bind(planningSceneCallback, _1, planning_scene_monitor)
    );

    // Spin the ROS node
    ros::spin();

    return 0;
}
