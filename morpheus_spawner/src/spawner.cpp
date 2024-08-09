// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shapes
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

// Eigen
#include <Eigen/Geometry>

// Import other files from module
#include "collision_object.h"
#include "attached_collision_object.h"
#include "morpheus_spawner/SpawnerService.h"

class SpawnerNode
{
  public:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_move_group_interface;
    moveit::planning_interface::PlanningSceneInterface g_planning_scene_interface;

    ros::Publisher g_collision_object_publisher; // Unnecessary, as objects can be spawned via the planning scene interface
    std::vector<moveit_msgs::CollisionObject> g_collision_object_vector;

    ros::Publisher g_attached_collision_object_publisher;
    std::vector<moveit_msgs::AttachedCollisionObject> g_attached_collision_object_vector;

    ros::Subscriber g_collision_object_subscriber;

    ros::Publisher g_planning_scene_diff_publisher;

    ros::ServiceServer g_spawner_service;

    SpawnerNode(int argc, char** argv)
    {
      // Init node handle
      ros::NodeHandle nh;
      ros::AsyncSpinner spinner(1);
      spinner.start();

      // Joints to plan for, from srdf file
      static const std::string PLANNING_GROUP = "arm";
      
      
      // Instantiate a move group interface so objects can be attached
      g_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

      // Instantiate publisher for attached collision objects (unattached collision objects are added by the planning scene interface)
      g_attached_collision_object_publisher = nh.advertise<moveit_msgs::AttachedCollisionObject>("/planning_scene/attached_collision_objects", 0);
    
      // Instantiate a move group interface so objects can be attached
      g_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

      // Instantiate publisher for attached collision objects (unattached collision objects are added by the planning scene interface)
      g_attached_collision_object_publisher = nh.advertise<moveit_msgs::AttachedCollisionObject>("/planning_scene/attached_collision_objects", 0);
    
      // Instantiate planning scene diff publisher
      g_planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

      // Instantiate collision object spawner service;
      g_spawner_service = nh.advertiseService("spawner", spawner_service);
      
      // Debug printouts
      ROS_INFO_STREAM("SpawnerNode ready");
    }

    moveit_msgs::CollisionObject createCollisionObject(std::string mesh_path, 
          std::vector<double> position,
          std::vector<double> quaternion)
    {
      // Debug printouts
      ROS_INFO_STREAM("Creating object");
      std::stringstream position_ss;
      position_ss << "Position:";
      for (double val : position)
      {
        position_ss << " ";
        position_ss << std::to_string(val);
      }
      ROS_INFO_STREAM(position_ss.str());
      std::stringstream quaternion_ss;
      quaternion_ss << "Quaternion:";
      for (double val : quaternion)
      {
        quaternion_ss << " ";
        quaternion_ss << std::to_string(val);
      }
      ROS_INFO_STREAM(quaternion_ss.str());

      // Create collision object
      moveit_msgs::CollisionObject collision_object =
        createCollisionObject(mesh_path,
                              position,
                              quaternion);

      // Create attached collision object to keep indices matching
      moveit_msgs::AttachedCollisionObject attached_collision_object =
        createAttachedCollisionObject(collision_object,
                                      "camera_link");

      // Save both versions of object to vector
      g_collision_object_vector.push_back(collision_object);
      g_attached_collision_object_vector.push_back(attached_collision_object);

      ROS_INFO_STREAM("Creating object complete");

      return collision_object;
    }

    void spawn(int index = 0)
    {
      // Debug printouts
      ROS_INFO_STREAM("Spawning object");

      // Select collision object to spawn
      moveit_msgs::CollisionObject collision_object = g_collision_object_vector[index];

      // Specify that object is to be added
      collision_object.operation = collision_object.ADD;

      // Publish planning scene diff
      moveit_msgs::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(collision_object);
      planning_scene.is_diff = true;
      g_planning_scene_diff_publisher.publish(planning_scene);

      // Debug printouts
      ROS_INFO_STREAM("Spawning object complete");
    }    

    void despawn(int index = 0)
    {
      // Debug printouts
      ROS_INFO_STREAM("Despawning object");

      // Select collision object to despawn
      moveit_msgs::CollisionObject collision_object = g_collision_object_vector[index];

      // Specify that object is to be removed
      collision_object.operation = collision_object.REMOVE;

      // Publish planning scene diff
      moveit_msgs::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(collision_object);
      planning_scene.is_diff = true;
      g_planning_scene_diff_publisher.publish(planning_scene);

      // Debug printouts
      ROS_INFO_STREAM("Despawning object complete");
    }

    void attach(int index = 0)
    {
      // Debug printouts
      ROS_INFO_STREAM("Attaching object");

      // Select collision object to attach
      moveit_msgs::CollisionObject collision_object = g_collision_object_vector[index];
      // Select attached collision object
      moveit_msgs::AttachedCollisionObject attached_collision_object = g_attached_collision_object_vector[index];
      
      // Specify that original object is to be removed, and the attached version added
      collision_object.operation = collision_object.REMOVE;
      attached_collision_object.object.operation = attached_collision_object.object.ADD;

      // Save collision object to vector
      g_attached_collision_object_vector.push_back(attached_collision_object);
      
      // Publish planning scene diff
      moveit_msgs::PlanningScene planning_scene;
      planning_scene.world.collision_objects.clear();
      planning_scene.world.collision_objects.push_back(collision_object);
      planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
      planning_scene.is_diff = true;
      g_planning_scene_diff_publisher.publish(planning_scene);

      // Debug printouts
      ROS_INFO_STREAM("Attaching object complete");
    }

    void detach(int index = 0, std::string link_name = "camera_link")
    {
      // Debug printouts
      ROS_INFO_STREAM("Detaching object");

      // Select collision object to detach
      moveit_msgs::CollisionObject collision_object = g_collision_object_vector[index];
      // Select attached collision object
      moveit_msgs::AttachedCollisionObject attached_collision_object = g_attached_collision_object_vector[index];
      
      // Specify that original object is to be added, and the attached version removed
      collision_object.operation = collision_object.ADD;
      attached_collision_object.object.operation = attached_collision_object.object.REMOVE;
      
      // Publish planning scene diff
      moveit_msgs::PlanningScene planning_scene;
      planning_scene.world.collision_objects.clear();
      planning_scene.world.collision_objects.push_back(collision_object);
      planning_scene.robot_state.attached_collision_objects.push_back(attached_collision_object);
      planning_scene.is_diff = true;
      g_planning_scene_diff_publisher.publish(planning_scene);

      // Debug printouts
      ROS_INFO_STREAM("Detaching object complete");
    }

    // Publish an incoming collision object message without edits
    void publish(moveit_msgs::CollisionObject collision_object)
    {
      // g_attached_collision_object_publisher.publish(collision_object);
      return;
    }

  private:
    // Define the service call function
    static bool spawner_service(morpheus_spawner::SpawnerService::Request &req,
                                                 morpheus_spawner::SpawnerService::Response &res)
    {
      std::string mesh_param = req.preset_name.append("/mesh_path");
      std::string mesh_path = "";
      ros::param::get(mesh_param, mesh_path);
      // TODO
      res.success = true;
      return true;
    }
};

std::vector<double> eulerToQuaternion(std::vector<double> euler)
{
  // Convert euler angles to quaternion
  Eigen::Quaternionf quaternion_eigen;
  quaternion_eigen = Eigen::AngleAxisf(euler[0], Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(euler[1], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(euler[2], Eigen::Vector3f::UnitZ());
  std::vector<double> quaternion;
  quaternion.resize(4);
  for (int i = 0; i < quaternion_eigen.coeffs().size(); i++)
  {
    double val = quaternion_eigen.coeffs()[i];
    quaternion[i] = val;
  }
  return quaternion;
}

int main(int argc, char** argv)
{

  // Start ROS node
  ros::init(argc, argv, "morpheus_spawner");

  // Parse arguments
  std::vector<std::string> arguments(argv, argv + argc);
  std::string mesh_path;
  std::vector<double> position;
  position.resize(3);
  // std::fill(position.begin(), position.end(), 0);
  std::vector<double> euler;
  euler.resize(3);
  // std::fill(euler.begin(), euler.end(), 0);
  std::string mode = "spawn";
  for (int i = 0; i < arguments.size(); i++) {
    std::string s = arguments[i];
    if (s == "-mesh_path") {
      mesh_path = arguments[i+1];
    }
    if (s == "-x") {
      position[0] = std::stod(arguments[i+1]);
    }
    if (s == "-y") {
      position[1] = std::stod(arguments[i+1]);
    }
    if (s == "-z") {
      position[2] = std::stod(arguments[i+1]);
    }
    if (s == "-R") {
      euler[0] = std::stod(arguments[i+1]);
    }
    if (s == "-P") {
      euler[1] = std::stod(arguments[i+1]);
    }
    if (s == "-Y") {
      euler[2] = std::stod(arguments[i+1]);
    }
    if (s == "-mode") {
      mode = std::stod(arguments[i+1]);
    }
    if (s == "-index") {
      mode = std::stod(arguments[i+1]);
    }
  }

  std::vector<double> quaternion = eulerToQuaternion(euler);

  SpawnerNode spawner_node(argc, argv);

  if (mode == "spawn") {
    moveit_msgs::CollisionObject collision_object = spawner_node.createCollisionObject(mesh_path, position, quaternion);
    spawner_node.spawn();
  }
  if (mode == "despawn") {
    spawner_node.despawn();
  }
  if (mode == "attach") {
    spawner_node.attach();
  }
  if (mode == "detach") {
    spawner_node.detach();
  }

  // ros::spin();

  return 0;
}
