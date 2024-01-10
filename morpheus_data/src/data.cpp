#include <filesystem>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>
#include <format>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <morpheus_msgs/ContactMap.h>

// name of the robot description (a param name, so it can be changed externally)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";

// Set of names of links included in the robot for collision detection
static const std::set<std::string> A_BOT_LINK_SET
{
    "shoulder_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_1_link",
    "wrist_2_link",
    "wrist_3_link",
    "tcp_link",
    "tcp_collision_link",
    "d415_mount_link",
    "camera_link",
    "coupler",
    "cable_protector",
    "gripper_body",
    "left_outer_knuckle",
    "left_outer_finger",
    "left_inner_finger",
    "left_inner_finger_pad",
    "left_inner_knuckle",
    "right_outer_knuckle",
    "right_outer_finger",
    "right_inner_finger",
    "right_inner_finger_pad",
    "right_inner_knuckle"
};

// Set of names of links included in the obstacles for collision detection
static const std::set<std::string> OBSTACLE_SET
{
    "teapot",
    "cylinder"
};

namespace data
{

};

class DataNode
{
    public:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;
        std::chrono::high_resolution_clock::time_point t1;
        std::ofstream g_file;
        std::string g_filename;
        std::stringstream g_next_line;

        ros::Subscriber g_scene_subscriber;
        ros::Subscriber g_contactmap_subscriber;

        DataNode(int argc, char** argv)
        {
            // Initialize ROS node
            ros::init(argc, argv, "data");
            ros::NodeHandle nh;
            ros::AsyncSpinner spinner(0);
            spinner.start();
            
            // Start clock
            t1 = std::chrono::high_resolution_clock::now();

            // Write a header for the file
            // TODO: make header strings dynamic, based on rostopics or arguments
            std::string header;
            std::string robot = "UR5e";
            std::string task = "test1";
            std::string id = "0";
            std::string datetime = std::format("{:%F%T}", std::chrono::system_clock::now());
            header << robot << "_" << task << "_" << id << "_" << datetime;
            g_filename = header;
            g_file.open(g_filename);
            g_file << header << "\n";
            g_file.close();

            // Retrieve preexisting PlanningSceneMonitor, if possible
            g_planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(ROBOT_DESCRIPTION);

            // Ensure the PlanningSceneMonitor is ready
            if (g_planning_scene_monitor->requestPlanningSceneState("/get_planning_scene"))
            {
                ROS_INFO("Planning Scene Monitor is active and ready.");
            }
            else
            {
                ROS_ERROR("Failed to set up Planning Scene Monitor.");
            }
            
            // Start the PlanningSceneMonitor
            g_planning_scene_monitor->startSceneMonitor("/move_group/monitored_planning_scene"); // Get scene updates from topic
            g_planning_scene_monitor->startWorldGeometryMonitor();
            g_planning_scene_monitor->startStateMonitor("/joint_states");

            g_scene_subscriber = nh.subscribe("/move_group/monitored_planning_scene", 1);
            g_contactmap_subcriber = nh.subscribe("/collision/contactmap", 1);
        }

        void spin()
        {
            // Loop collision requests and publish at specified rate
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                update();
                publish();
                loop_rate.sleep();
            }

            // Spin the ROS node
            ros::spin();
        }

        void update()
        {
            g_next_line = new std::stringstream;

            // Get time since start
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> time_elapsed = t2 - t1;
            g_next_line << time_elapsed << ", ";

            // Get link positions
            // TODO: Listener should request from monitored planning scene topic
            // Should include a column for every link

            // Get contact map
            // TODO: Listener should request from collision/contactmap topic
            // Should include a column for every link pair

        }

        void publish()
        {
            g_file.open(g_filename);
            g_file << g_next_line << "\n";
            g_file.close();
        }

};

int main(int argc, char** argv)
{
    DataNode data_node(argc, argv);
    data_node.spin();
    return 0;
}
