/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Shapes
#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

// Eigen
#include <Eigen/Geometry>

moveit_msgs::CollisionObject createCollisionObject(std::string mesh_path, 
                                                  std::vector<double> position,
                                                  std::vector<double> orientation)
{
  // Create a collision object.
  moveit_msgs::CollisionObject collision_object;

  // Import mesh.
  const Eigen::Vector3d scale(0.0393700787, 0.0393700787, 0.0393700787); // mm/inch
  ROS_INFO_STREAM("Creating mesh from " << mesh_path);
  shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path, scale);
  ROS_INFO_STREAM("Mesh loading done");

  // find the center of the mesh
  int vertex_count = mesh->vertex_count;
  double sx = 0.0, sy = 0.0, sz = 0.0;
  for (unsigned int i = 0; i < vertex_count; ++i)
  {
    unsigned int i3 = i * 3;
    sx += mesh->vertices[i3];
    sy += mesh->vertices[i3 + 1];
    sz += mesh->vertices[i3 + 2];
  }
  sx /= (double)vertex_count;
  sy /= (double)vertex_count;
  sz /= (double)vertex_count;

  // mesh->scaleAndPadd(0.1, 0.0); // Scale correction
  shapes::ShapeMsg shape_msg;
  shapes::constructMsgFromShape(mesh, shape_msg);
  shape_msgs::Mesh shape_msgs_mesh;
  shape_msgs_mesh = boost::get<shape_msgs::Mesh>(shape_msg);

  // Initialize the object id and frame.
  collision_object.id = mesh_path + std::to_string(ros::Time::now().toSec());
  collision_object.header.frame_id = "world";

  // Add the mesh and its dimensions.
  collision_object.meshes.resize(1);
  collision_object.meshes[0] = shape_msgs_mesh;

  // Define the pose of the mesh.
  collision_object.mesh_poses.resize(1);
  collision_object.mesh_poses[0].position.x = position[0] - sx;
  collision_object.mesh_poses[0].position.y = position[1] - sy;
  collision_object.mesh_poses[0].position.z = position[2] - sz;

  // collision_object.operation = collision_object.ADD;

  // planning_scene_interface.applyCollisionObjects(collision_objects);

  return collision_object;
}

moveit_msgs::CollisionObject addCollisionObject(moveit::planning_interface::PlanningSceneInterface planning_scene_interface,
                                                  std::string mesh_path, 
                                                  std::vector<double> position,
                                                  std::vector<double> orientation)
{
  moveit_msgs::CollisionObject collision_object = createCollisionObject(mesh_path, position, orientation);
  
  collision_object.operation = collision_object.ADD;
  planning_scene_interface.applyCollisionObject(collision_object);
  
  return collision_object;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "morpheus_spawner");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> arguments(argv, argv + argc);
  std::string mesh_path;
  std::vector<double> position;
  position.resize(3);
  std::vector<double> orientation;
  orientation.resize(3);
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
      orientation[0] = std::stod(arguments[i+1]);
    }
    if (s == "-P") {
      orientation[1] = std::stod(arguments[i+1]);
    }
    if (s == "-Y") {
      orientation[2] = std::stod(arguments[i+1]);
    }
  }

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  addCollisionObject(planning_scene_interface, mesh_path, position, orientation);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  ros::waitForShutdown();
  return 0;
}
