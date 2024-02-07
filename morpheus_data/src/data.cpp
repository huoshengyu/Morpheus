#include <filesystem>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>

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
#include <morpheus_msgs/StringPair.h>

// name of the robot description (a param name, so it can be changed externally)
static const std::string ROBOT_DESCRIPTION =
    "robot_description";

// Vector of names of links included in the robot for collision detection
static const std::vector<std::string> A_BOT_LINK_VECTOR
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

// Vector of names of links included in the obstacles for collision detection
static const std::vector<std::string> OBSTACLE_VECTOR
{
    "teapot",
    "cylinder",
    "test_wall_1",
    "wall",
    "foam"
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

        ros::Subscriber g_contactmap_subscriber;
        inline static morpheus_msgs::ContactMap g_latest_contactmap;
        inline static bool received_contactmap;

        std::vector<std::string> g_robot_links;
        std::vector<std::string> g_obstacle_links;
        std::vector<std::string> g_column_label_vector;

        DataNode(int argc, char** argv)
        {
            // Select links to track
            g_robot_links = A_BOT_LINK_VECTOR;
            g_obstacle_links = OBSTACLE_VECTOR;
            
            // Initialize ROS node
            ros::init(argc, argv, "data");
            ros::NodeHandle nh;
            ros::AsyncSpinner spinner(0);
            spinner.start();
            
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

            g_contactmap_subscriber = nh.subscribe("/collision/contactmap", 1, contactMapCallback);

            // Write a header for the file
            // TODO: make header strings dynamic, based on rostopics or arguments
            std::stringstream header;
            std::string robot = "UR5e";
            std::string task = "test1";
            std::string id = "0";
            std::time_t datetime_t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
            struct tm * timeinfo;
            timeinfo = localtime (&datetime_t);
            char datetime_buffer [80];
            strftime(datetime_buffer,80,"%F_%T",timeinfo);
            header << robot << "_" << task << "_" << id << "_" << datetime_buffer;
            g_filename = header.str();

            // Get directory of data folder. Note: cwd is /root/.ros/ by default
            std::string data_dir = "/root/catkin_ws/src/morpheus/morpheus_data/data/";

            // Add the header line to the file
            ROS_INFO_STREAM("Creating file with name: " << g_filename);
            std::stringstream filepath;
            filepath << data_dir << g_filename << ".csv";
            g_file = std::ofstream(filepath.str());
            if ( (g_file.rdstate() & std::ofstream::failbit ) != 0 )
                std::cerr << "Error opening file\n";
            g_file << header.str() << std::endl;

            // Add column labels to the file
            std::stringstream column_label_ss;
            g_column_label_vector.push_back("Time");
            g_column_label_vector.push_back("Distance");
            // Add robot links to labels, for tracking positions
            std::map<std::string, double> current_state_values = getCurrentStateValues();
            for (auto const& key_value : current_state_values)
            {
                g_column_label_vector.push_back(key_value.first);
            }
            // Add robot links to labels, for tracking distances
            for (std::string link : g_robot_links)
            {
                g_column_label_vector.push_back(link);
            }

            // Add all labels to stringstream
            for (std::string label : g_column_label_vector)
            {
                column_label_ss << label << ", ";
            }
            column_label_ss << std::endl;
            g_file << column_label_ss.str();

            received_contactmap = false;
        }

        void spin()
        {
            // Start clock
            t1 = std::chrono::high_resolution_clock::now();

            // Loop collision requests and publish at specified rate
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                if (received_contactmap)
                {
                    update();
                    publish();
                }
                else
                {
                    ROS_INFO_STREAM("Waiting for contactmap topic...");
                }
                
                loop_rate.sleep();
            }

            // Spin the ROS node
            ros::spin();

            // Close when complete
            g_file.close();
        }

        void update()
        {
            g_next_line.str(""); // Reset g_next_line

            // Get time since start
            std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> time_elapsed = t2 - t1;
            g_next_line << time_elapsed.count() << ", ";

            // Get joint angles
            // Should include a column for every robot joint
            std::map<std::string, double> current_state_values = getCurrentStateValues();
            for (auto const& key_value : current_state_values)
            {
                g_next_line << key_value.second << ", ";
            }

            // Get contact map
            // TODO: Listener should request from collision/contactmap topic
            // Should include a column for every robot link
            std::vector<moveit_msgs::ContactInformation> contact_info_vector = g_latest_contactmap.values;
            std::vector<double> sorted_distances;
            for (moveit_msgs::ContactInformation contact_info : contact_info_vector)
            {
                std::string link_1 = contact_info.contact_body_1;
                std::string link_2 = contact_info.contact_body_2;
                
                auto it = std::find(g_robot_links.begin(), g_robot_links.end(), link_1);
                if (it != g_robot_links.end())
                {
                    int index = it - g_robot_links.begin();
                    sorted_distances[index] = contact_info.depth;
                }
                else
                {
                    it = std::find(g_robot_links.begin(), g_robot_links.end(), link_2);
                    if (it != g_robot_links.end())
                    {
                        int index = it - g_robot_links.begin();
                        sorted_distances[index] = contact_info.depth;
                    }
                }
                
            }

        }

        void publish()
        {
            ROS_INFO_STREAM("Adding line: " << g_next_line.str());
            g_file << g_next_line.str() << std::endl;
        }

        std::map<std::string, double> getCurrentStateValues()
        {
            return g_planning_scene_monitor->getStateMonitor()->getCurrentStateValues();
        }

        static void emptyCallback(std_msgs::String msg)
        {
            
        }

        static void contactMapCallback(morpheus_msgs::ContactMap msg)
        {
            g_latest_contactmap = msg;
            received_contactmap = true;
        }


};

int main(int argc, char** argv)
{
    DataNode data_node(argc, argv);
    data_node.spin();
    return 0;
}
