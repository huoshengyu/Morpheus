// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>

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

    ros::Publisher g_planning_scene_diff_publisher;

    ros::Subscriber g_spawner_subscriber;

    ros::ServiceServer g_spawner_service;

    SpawnerNode(int argc, char** argv)
    {
      // Instantiate node handle
      ros::NodeHandle nh;
      ros::AsyncSpinner spinner(1);
      spinner.start();

      // Joints to plan for, from srdf file
      static const std::string PLANNING_GROUP = "arm";
      
      // Instantiate a move group interface so objects can be attached
      g_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

      // Move group interface does not need further instantiating

      // Instantiate publisher for attached collision objects (unattached collision objects are added by the planning scene interface)
      g_collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 0);

      // Instantiate publisher for attached collision objects (unattached collision objects are added by the planning scene interface)
      g_attached_collision_object_publisher = nh.advertise<moveit_msgs::AttachedCollisionObject>("/planning_scene/attached_collision_objects", 0);
    
      // Instantiate planning scene diff publisher
      g_planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 0);

      // Instantiate a subscriber to receive names of object presets to spawn
      g_spawner_subscriber = nh.subscribe<std_msgs::String>("/spawner/spawn_queue", 1, &SpawnerNode::spawner_subscriber_callback, this);

      // Instantiate collision object spawner service
      g_spawner_service = nh.advertiseService("spawner", &SpawnerNode::spawner_service_callback, this);
      
      // Debug printouts
      ROS_INFO_STREAM("SpawnerNode ready");
    }

    moveit_msgs::CollisionObject create(std::string mesh_path, 
          std::vector<double> scale = {1, 1, 1},
          std::vector<double> position = {0, 0, 0},
          std::vector<double> quaternion = {0, 0, 0, 1})
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
                              scale,
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

    // Second implementation for map type inputs
    moveit_msgs::CollisionObject create(std::string mesh_path, 
          std::map<std::string, double> scale = {{"x", 1}, {"y", 1}, {"z", 1}},
          std::map<std::string, double> position = {{"x", 0}, {"y", 0}, {"z", 0}},
          std::map<std::string, double> quaternion = {{"x", 0}, {"y", 0}, {"z", 0}, {"w", 0}})
    {
      // Reformat maps as vectors
      std::vector<double> scale_vec = {scale["x"], scale["y"], scale["z"]};
      std::vector<double> position_vec = {position["x"], position["y"], position["z"]};
      std::vector<double> quaternion_vec = {quaternion["x"], quaternion["y"], quaternion["z"], quaternion["w"]};

      // Call the vector-based implementation
      moveit_msgs::CollisionObject collision_object = create(mesh_path, scale_vec, position_vec, quaternion_vec);

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

    void spawn(moveit_msgs::CollisionObject collision_object)
    {
      // Debug printouts
      ROS_INFO_STREAM("Spawning object");

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

    // Spin node to continue handling services
    void spin()
    {
      ros::spin();
    }

    // Define the subscriber callback function
    void spawner_subscriber_callback(std_msgs::String msg)
    {
      // Get name of the param which holds the mesh path
      std::string mesh_param = msg.data.append("/mesh_path");
      // Instantiate a string to hold the mesh path
      std::string mesh_path = "";
      // Retrieve the mesh path from param
      ros::param::get(mesh_param, mesh_path);
      // Specify that the path is relative to the morpheus_teleop package
      mesh_path = "file://" + ros::package::getPath("morpheus_teleop") + mesh_path;

      // Get the name of the param which holds the scale
      std::string scale_param = msg.data.append("/scale");
      // Instantiate a map to hold the scale
      std::map<std::string, double> scale_map;
      //Retrieve the scale from param
      ros::param::get(scale_param, scale_map);

      // Get the names of the params holding the object's coordinates
      std::string pos_param = msg.data.append("/position");
      std::string quat_param = msg.data.append("/quaternion");
      // Instantiate maps to hold the coordinates
      std::map<std::string, double> pos_map, quat_map;
      // Retrieve the coordinates from param
      ros::param::get(pos_param, pos_map);
      ros::param::get(quat_param, quat_map);
      // Convert maps to vectors (unnecessary since collision_object.cpp now supports maps as well as vectors as input)
      // std::vector<double> pos_vector = {pos_map["x"], pos_map["y"], pos_map["z"]};
      // std::vector<double> quat_vector = {quat_map["x"], quat_map["y"], quat_map["z"], quat_map["w"]};

      // Create the collision object (and save it to g_collision_object_vector, g_attached_collision_object_vector)
      moveit_msgs::CollisionObject collision_object = create(mesh_path, scale_map, pos_map, quat_map);
      
      // Spawn the created object
      spawn(g_collision_object_vector.size() - 1);
    }

    // Define the service call function
    bool spawner_service_callback(morpheus_spawner::SpawnerService::Request &req,
                                  morpheus_spawner::SpawnerService::Response &res)
    {
      // Set response/output to false in case failure occurs along the way
      res.success = false;

      // Get name of the param which holds the mesh path
      std::string mesh_param = req.preset_name.append("/mesh_path");
      // Instantiate a string to hold the mesh path
      std::string mesh_path = "";
      // Retrieve the mesh path from param
      ros::param::get(mesh_param, mesh_path);
      // Specify that the path is relative to the morpheus_teleop package
      mesh_path = "file://" + ros::package::getPath("morpheus_teleop") + mesh_path;

      // Get the name of the param which holds the scale
      std::string scale_param = req.preset_name.append("/scale");
      // Instantiate a map to hold the scale
      std::map<std::string, double> scale_map;
      //Retrieve the scale from param
      ros::param::get(scale_param, scale_map);

      // Get the names of the params holding the object's coordinates
      std::string pos_param = req.preset_name.append("/position");
      std::string quat_param = req.preset_name.append("/quaternion");
      // Instantiate maps to hold the coordinates
      std::map<std::string, double> pos_map, quat_map;
      // Retrieve the coordinates from param
      ros::param::get(pos_param, pos_map);
      ros::param::get(quat_param, quat_map);
      // Convert maps to vectors (unnecessary since collision_object.cpp now supports maps as well as vectors as input)
      // std::vector<double> pos_vector = {pos_map["x"], pos_map["y"], pos_map["z"]};
      // std::vector<double> quat_vector = {quat_map["x"], quat_map["y"], quat_map["z"], quat_map["w"]};

      // Create the collision object (and save it to g_collision_object_vector, g_attached_collision_object_vector)
      moveit_msgs::CollisionObject collision_object = create(mesh_path, scale_map, pos_map, quat_map);

      // Spawn the created object
      spawn(g_collision_object_vector.size() - 1);

      // Set response/output to true and exit
      res.success = true;
      return res.success;
    }

  private:
    
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
  std::string mode = "none";
  int index = 0;
  for (int i = 0; i < arguments.size(); i++) {
    std::string s = arguments[i];
    ROS_INFO_STREAM(s);
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
      mode = arguments[i+1];
    }
    if (s == "-index") {
      index = std::stoi(arguments[i+1]);
    }
  }

  std::vector<double> quaternion = eulerToQuaternion(euler);

  SpawnerNode spawner_node(argc, argv);

  std::vector<double> scale = {0.0254, 0.0254, 0.0254};

  if (mode == "spawn") {
    moveit_msgs::CollisionObject collision_object = spawner_node.create(mesh_path, scale=scale, position=position, quaternion=quaternion);
    spawner_node.spawn(spawner_node.g_collision_object_vector.size() - 1);
  }
  if (mode == "despawn") {
    spawner_node.despawn(spawner_node.g_collision_object_vector.size() - 1);
  }
  if (mode == "attach") {
    spawner_node.attach(spawner_node.g_collision_object_vector.size() - 1);
  }
  if (mode == "detach") {
    spawner_node.detach(spawner_node.g_collision_object_vector.size() - 1);
  }

  spawner_node.spin();

  return 0;
}
