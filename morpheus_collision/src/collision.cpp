#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

namespace collision
{

};

class CollisionNode
{
    public:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;
        ros::Publisher g_distance_publisher;
        ros::Publisher g_contacts_publisher;
        collision_detection::CollisionResult g_c_res;
        collision_detection::CollisionRequest g_c_req;

        ros::Publisher* g_marker_array_publisher = nullptr;
        visualization_msgs::MarkerArray g_collision_points;

        CollisionNode(int argc, char** argv)
        {
            // Initialize ROS node
            ros::init(argc, argv, "collision");
            ros::NodeHandle nh;
            ros::AsyncSpinner spinner(0);
            spinner.start();

            // Create a RobotModelLoader to load the robot's URDF and SRDF
            // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            // robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

            // Create a PlanningScene object and set the robot model
            // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

            // Create a PlanningSceneMonitor around the PlanningScene
            // planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(
            //     new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

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

            // Request the PlanningScene itself and change collision detection engine to Bullet
            // planning_scene::PlanningScenePtr planning_scene;
            // Get read/write pointer to planning_scene
            try
            {   
                // Change the PlanningScene's collision detector to Bullet
                // Bullet supports distance vectors, as well as distances to multiple obstacles
                g_planning_scene_monitor->getPlanningScene()->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(), 
                                                    true /* exclusive */);
                
                if (strcmp((g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName()).c_str(), "Bullet") == 0)
                {
                    ROS_INFO("Planning Scene is active and ready.");
                }    
                else
                {
                    ROS_INFO("Collision detector incorrect");
                    // ROS_INFO(g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName());
                    std::string collision_detector_name = g_planning_scene_monitor->getPlanningScene()->getActiveCollisionDetectorName();
                    throw collision_detector_name;
                }
            }
            catch (std::string collision_detector_name)
            {
                ROS_ERROR("Failed to retrieve PlanningScene.");
            }
            
            // Start the PlanningSceneMonitor
            g_planning_scene_monitor->startSceneMonitor("/move_group/monitored_planning_scene"); // Get scene updates from topic
            // g_planning_scene_monitor->startWorldGeometryMonitor();
            // g_planning_scene_monitor->startStateMonitor();

            // Prepare collision result and request objects
            g_c_req.contacts = true;
            g_c_req.distance = true;
            g_c_req.max_contacts = 20;
            g_c_req.max_contacts_per_pair = 1;
            // Update the collision result
            // planning_scene->checkCollision(g_c_req, g_c_res);

            // Create publishers
            g_distance_publisher = nh.advertise<std_msgs::Float64>("distance", 10);
            g_contacts_publisher = nh.advertise<std_msgs::String>("contacts", 10);
            
            // Create a marker array publisher for publishing contact points
            g_marker_array_publisher =
                new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>("interactive_robot_marray", 100));

            // Add callback which dictates behavior after each scene update
            // planning_scene_monitor->addUpdateCallback
            
            // Get a list of all links in the robot so we can check them for collisions


            // ros::shutdown();
        }

        void spin()
        {
            // Loop collision requests and publish at specified rate
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                update();
                publish();
                // Get all contact vectors which correspond to robot<->obstacle pairs
                //for (int i : contact_map)
                //{

                //}
                loop_rate.sleep();
            }

            // Spin the ROS node
            ros::spin();
        }

        void update()
        {
            // Update the planning scene monitor, in case new collision objects have been added (may be slow)
            g_planning_scene_monitor->requestPlanningSceneState("/get_planning_scene");
            // Update the collision result, based on the collision request
            g_planning_scene_monitor->getPlanningScene()->checkCollision(g_c_req, g_c_res);
        }

        void publish()
        {
            // Publish minimum distance
            std_msgs::Float64 distance_msg;
            distance_msg.data = g_c_res.distance;
            g_distance_publisher.publish(distance_msg);

            // Publish contact map
            std_msgs::String contacts_msg;
            contacts_msg.data = contactMapToString(g_c_res.contacts);
            g_contacts_publisher.publish(contacts_msg);
        }

        std::string contactMapToString(collision_detection::CollisionResult::ContactMap contact_map)
        {
            // Convert the key list to a string
            std::stringstream key_value_list_str;
            // Iterate over key-value pairs of contact_map
            for (const auto& key_value : contact_map) 
            {
                // Separate keys from values for readability
                const std::pair<std::string, std::string>& key = key_value.first;
                const std::vector<collision_detection::Contact>& value = key_value.second;

                // Enforce condition
                if (key.first == "teapot" or key.second == "teapot")
                {
                    // First add the keys, each of which is a pair of link names, for links in contact
                    key_value_list_str << "Contact: (" << key.first << ", " << key.second << "), Vector: [";
                    // Optionally add the depth, normal, and position associated of the Contact object, i.e. a distance vector
                    // for (const collision_detection::Contact& contact : value) 
                    // {
                    //     key_value_list_str << "{depth: " << contact.depth << ", normal: " << contact.normal << ", pos: " << contact.pos << "}, ";
                    // }
                    // Just publish the first (smallest) depth so we can see the pairwise nearest distances
                    const collision_detection::Contact& contact = value[0];
                    key_value_list_str << contact.depth;
                    // End entry
                    key_value_list_str << "]" << '\\';
                }
            }

            // Convert result to string type
            std::string result = key_value_list_str.str();

            return result;
        }

        void visualizeContacts(collision_detection::CollisionResult::ContactMap contact_map)
        {
            for (const auto& key_value : contact_map)
            {
                // Set a color for the visualization markers
                std_msgs::ColorRGBA color;
                color.r = 1.0;
                color.g = 0.0;
                color.b = 0.0;
                color.a = 0.5;

                // Get the contact points and display them as markers
                visualization_msgs::MarkerArray markers;
                collision_detection::getCollisionMarkersFromContacts(markers, "teapot", contact_map, color,
                                                                    ros::Duration(),  // remain until deleted
                                                                    0.01);            // radius
            }
        }

        void publishMarkers(visualization_msgs::MarkerArray& markers)
        {
            // delete old markers
            if (!g_collision_points.markers.empty())
            {
                for (auto& marker : g_collision_points.markers)
                marker.action = visualization_msgs::Marker::DELETE;

                g_marker_array_publisher->publish(g_collision_points);
            }

            // move new markers into g_collision_points
            std::swap(g_collision_points.markers, markers.markers);

            // draw new markers (if there are any)
            if (!g_collision_points.markers.empty())
                g_marker_array_publisher->publish(g_collision_points);
        }
        
    private:
        // Define a callback to update to be called when the PlanningSceneMonitor receives an update
        void planningSceneMonitorCallback(const moveit_msgs::PlanningScene::ConstPtr& planning_scene, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
        {
            // Check collisions
            // planning_scene->checkCollision(g_c_req, g_c_res);
        }


};

int main(int argc, char** argv)
{
    CollisionNode collision_node(argc, argv);
    collision_node.spin();
    return 0;
}
