#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const std::string ROBOT_DESCRIPTION =
    "robot_description";  // name of the robot description (a param name, so it can be changed externally)

static const std::string GOAL_NAME_DEFAULT =
    "keyhole"; // name of the scene associated with the goal pose

namespace trajectory
{

};

class TrajectoryNode
{
    public:
        std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_planning_scene_monitor;

        ros::Publisher g_marker_array_publisher;
        ros::Publisher g_trajectory_publisher;
        ros::Publisher g_forward_publisher;
        visualization_msgs::MarkerArray g_trajectory_marker_array;
        moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

        // std::shared_ptr<moveit_cpp::PlanningComponent> g_planning_components;
        // std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters> g_plan_request_parameters;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> g_move_group_interface;
        moveit::planning_interface::PlanningSceneInterface g_planning_scene_interface;
        moveit::planning_interface::MoveGroupInterface::Plan g_plan;
        std::shared_ptr<robot_trajectory::RobotTrajectory> g_trajectory;
        Eigen::Affine3d g_nearest;
        Eigen::Affine3d g_forward;
        geometry_msgs::PoseStamped g_target;
        moveit_msgs::OrientationConstraint g_constraint;

        std::string g_goal_name;

        TrajectoryNode(int argc, char** argv)
        {
            // Initialize ROS node
            ros::NodeHandle nh;
            ros::AsyncSpinner spinner(0);
            spinner.start();

            // Joints to plan for, from srdf file
            static const std::string PLANNING_GROUP = "arm";

            // Prep moveit_cpp_ptr to allow planning
            // Prep options for moveit_cpp_ptr
            /*
            moveit_cpp::MoveItCpp::Options options(nh);

            moveit_cpp::MoveItCpp::PlanningSceneMonitorOptions psm_options;
            psm_options.name = "trajectory_monitor";
            psm_options.robot_description = ROBOT_DESCRIPTION;
            psm_options.joint_state_topic = "/joint_states";
            psm_options.attached_collision_object_topic = "/attached_collision_object";
            psm_options.monitored_planning_scene_topic = "/planning_scene";
            psm_options.publish_planning_scene_topic = "/move_group/monitored_planning_scene";
            options.planning_scene_monitor_options = psm_options;

            moveit_cpp::MoveItCpp::PlanningPipelineOptions ppl_options;
            ppl_options.pipeline_names.push_back("ompl");
            ppl_options.parent_namespace = "/move_group";
            options.planning_pipeline_options = ppl_options;
            */
            // auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(options, nh);
            // moveit_cpp_ptr->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
            
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
            // g_planning_scene_monitor = moveit_cpp_ptr->getPlanningSceneMonitorNonConst();
            // Instantiate planning components so a trajectory can be generated for comparison
            // g_planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, nh);
            
            // Instantiate a move group interface so a trajectory can be generated
            g_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

            // Set plan request parameters to select planning pipeline etc
            /*
            g_plan_request_parameters->max_acceleration_scaling_factor = 0.1;
            g_plan_request_parameters->max_velocity_scaling_factor = 0.1;
            g_plan_request_parameters->planner_id = "MotionPlanning";
            g_plan_request_parameters->planning_attempts = 10;
            g_plan_request_parameters->planning_pipeline = "MotionPlanning";
            g_plan_request_parameters->planning_time = 5;
            */

            // Set update callback
            // g_planning_scene_monitor->addUpdateCallback(planningSceneMonitorCallback);

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
            g_planning_scene_monitor->startWorldGeometryMonitor();
            g_planning_scene_monitor->startStateMonitor("/joint_states");
            
            /*
            // Edit the allowed collision matrix to focus only on robot-obstacle collisions
            collision_detection::AllowedCollisionMatrix allowed_collision_matrix = 
                g_planning_scene_monitor->getPlanningScene().getAllowedCollisionMatrix();
            allowed_collision_matrix.setEntry(true); // Allow all collisions
            allowed_collision_matrix.setEntry("teapot", false); // Register collisions involving teapot
            */

            // Create trajectory publisher
            g_trajectory_publisher = nh.advertise<std_msgs::String>("trajectory", 10);
            
            // Create a marker array publisher for publishing shapes to Rviz
            g_marker_array_publisher = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
            
            // Generate a target pose for trajectory planning
            geometry_msgs::PoseStamped g_target;
            g_target.header.stamp = ros::Time::now();
            g_target.header.frame_id = "world";
            g_target.pose.position.x = 0.4;
            g_target.pose.position.y = -0.2;
            g_target.pose.position.z = 1.0;
            g_target.pose.orientation.x = 0.5;
            g_target.pose.orientation.y = -0.5;
            g_target.pose.orientation.z = -0.5;
            g_target.pose.orientation.w = -0.5;
            // Get target vector from ros server, if possible
            if (ros::param::get("/goal/name", g_goal_name))
            {
                ROS_INFO("Using GOAL_NAME from parameter server");
            }
            else
            {
                g_goal_name = GOAL_NAME_DEFAULT;
                ROS_INFO("Using GOAL_NAME_DEFAULT");
            }
            if (ros::param::has("/goal/transform/" + g_goal_name))
            {
                ros::param::get("/goal/transform/" + g_goal_name + "/position/x", g_target.pose.position.x);
                ros::param::get("/goal/transform/" + g_goal_name + "/position/y", g_target.pose.position.y);
                ros::param::get("/goal/transform/" + g_goal_name + "/position/z", g_target.pose.position.z);
                ros::param::get("/goal/transform/" + g_goal_name + "/orientation/x", g_target.pose.orientation.x);
                ros::param::get("/goal/transform/" + g_goal_name + "/orientation/y", g_target.pose.orientation.y);
                ros::param::get("/goal/transform/" + g_goal_name + "/orientation/z", g_target.pose.orientation.z);
                ros::param::get("/goal/transform/" + g_goal_name + "/orientation/w", g_target.pose.orientation.w);
                ROS_INFO("Using GOAL_TRANSFORM from parameter server");
            }
            else
            {
                ROS_INFO("Using GOAL_TRANSFORM_DEFAULT");
            }

            // Generate constraints for trajectory planning
            moveit_msgs::OrientationConstraint g_constraint;
            g_constraint.link_name = "tcp_link";
            g_constraint.header.frame_id = "world";
            g_constraint.orientation.w = 1.0;
            g_constraint.absolute_x_axis_tolerance = 0.1;
            g_constraint.absolute_y_axis_tolerance = 0.1;
            g_constraint.absolute_z_axis_tolerance = 0.1;
            g_constraint.weight = 1.0;


            // Get robot model from the current planning scene
            const robot_model::RobotModelConstPtr robot_model = g_planning_scene_monitor->getRobotModel();

            // Get the current robot state once so that it does not vary over time
            // A LockedPlanningSceneRO is used to avoid modifying the planning scene.
            // Alternatively, the robot state can be set from a given joint state or from a given pose (using IK)
            robot_state::RobotStatePtr robot_state(
                new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(g_planning_scene_monitor)->getCurrentState()));

            // Create a joint model group for tracking the current robot pose and planning group
            const moveit::core::JointModelGroup* joint_model_group =
                robot_state->getJointModelGroup(PLANNING_GROUP);
            
            // Set planning parameters
            g_move_group_interface->setPlanningTime(60);
            g_move_group_interface->setPoseTarget(g_target, "tcp_link");
            // g_move_group_interface->setPositionTarget(0.2, 0.2, 0.8, "tcp_link");
            g_move_group_interface->setStartState(*robot_state);
            
            // Create plan
            bool success = (g_move_group_interface->plan(g_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            g_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, joint_model_group);
            g_trajectory->setRobotTrajectoryMsg(*robot_state, g_plan.start_state_, g_plan.trajectory_);
            //planning_interface::MotionPlanResponse plan = getPlan(g_target);

            // Instantiate visual tools for visualizing markers in Rviz
            visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("/world", "visualization_marker_array", g_planning_scene_monitor);

            // Add callback which dictates behavior after each scene update
            // planning_scene_monitor->addUpdateCallback
            
            // Get a list of all links in the robot so we can check them for collisions

            spin();
            // ros::shutdown();
        }

        void spin()
        {
            // Loop collision requests and publish at specified rate
            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                // updateCollision();
                // publishCollision();
                // visualizeCollision(g_c_res.contacts);
                
                auto current_state = g_planning_scene_monitor->getPlanningScene()->getCurrentState();

                // auto plan = g_planning_components->getLastMotionPlanResponse();
                auto waypoints = getWaypoints(*g_trajectory);
                auto transform_deque = getTransforms(waypoints);
                current_state.updateLinkTransforms();
                Eigen::Affine3d tcp_transform = current_state.getGlobalLinkTransform("tcp_link");
                std::vector<Eigen::Affine3d> transform_vector = getNearestTransform(transform_deque, tcp_transform);
                g_nearest = transform_vector[0];
                g_forward = transform_vector[1];
                publishTrajectory(transform_deque);
                visualizeTrajectory(transform_deque);

                // Update the nearest plan point


                // Get all contact vectors which correspond to robot<->obstacle pairs
                //for (int i : contact_map)
                //{

                //}
                loop_rate.sleep();
            }

            // Spin the ROS node
            ros::spin();
        }

        // Generate target poses for planning
        geometry_msgs::PoseStamped getPose(double x=0, double y=0, double z=0, 
                                double w=0, double rx=0, double ry=0, double rz=0)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "world";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.w = w;
            pose.pose.orientation.x = rx;
            pose.pose.orientation.y = ry;
            pose.pose.orientation.z = rz;

            return pose;
        }

        // Plan trajectories for comparison
        /*
        planning_interface::MotionPlanResponse getPlan(geometry_msgs::PoseStamped target_pose)
        {
            // Use current state as starting pose
            g_planning_components->setStartStateToCurrentState();

            // Set tcp_link (middle of gripper) to target the target pose
            g_planning_components->setGoal(target_pose, "tcp_link");

            // Generate plan
            auto plan = g_planning_components->plan(*g_plan_request_parameters); // plan is a planning_interface::MotionPlanResponse

            return plan;
        }
        */

        // Get waypoints from plan
        std::deque<moveit::core::RobotState> getWaypoints(planning_interface::MotionPlanResponse& plan) // returns a std::deque< robot_state::RobotStatePtr >
        {
            std::deque<moveit::core::RobotState> waypoints;
            for (int i = 0; i < plan.trajectory_->getWayPointCount(); i++)
            {
                waypoints.push_back(plan.trajectory_->getWayPoint(i));
            }
            return waypoints;
        }

        // Get waypoints from trajectory
        std::deque<moveit::core::RobotState> getWaypoints(robot_trajectory::RobotTrajectory trajectory) // returns a std::deque< robot_state::RobotStatePtr >
        {   
            std::deque<moveit::core::RobotState> waypoints;
            for (int i = 0; i < trajectory.getWayPointCount(); i++)
            {
                waypoints.push_back(trajectory.getWayPoint(i));
            }
            return waypoints;
        }

        // Get global transforms of target link from plan
        std::deque<Eigen::Affine3d> getTransforms(std::deque<moveit::core::RobotState> waypoints, std::string link = "tcp_link")
        {
            std::deque<Eigen::Affine3d> transform_deque;
            for (auto robot_state : waypoints) // robot_state::RobotStatePtr
            {
                Eigen::Affine3d transform;
                bool success = false;
                while (!success)
                {
                    try
                    {
                        transform = robot_state.getGlobalLinkTransform(link); // moveit::core::RobotState
                        success = true;
                    }
                    catch (...)
                    {
                        continue;
                    }
                }
                transform_deque.push_back(transform);
            }
            return transform_deque;
        }

        // Get global transforms of target link from plan
        std::deque<Eigen::Affine3d> getTransforms(planning_interface::MotionPlanResponse plan, std::string link = "tcp_link")
        {
            auto waypoints = getWaypoints(plan); // std::deque< robot_state::RobotStatePtr >

            std::deque<Eigen::Affine3d> transform_deque;
            for (auto robot_state : waypoints) // robot_state::RobotStatePtr
            {
                Eigen::Affine3d transform = robot_state.getGlobalLinkTransform(link); // moveit::core::RobotState
                transform_deque.push_back(transform);
            }
            return transform_deque;
        }

        // Calculate transform on trajectory transform_deque nearest to a transform P
        // Return a pair of the transform and its translational distance from P
        std::vector<Eigen::Affine3d> getNearestTransform(std::deque<Eigen::Affine3d> transform_deque, Eigen::Affine3d P)
        {
            // Instantiate loop variables
            int best_index;
            double min_distance = std::numeric_limits<double>::infinity();
            Eigen::Vector3d translation;
            double interpolation_param;
            Eigen::Affine3d A;
            Eigen::Affine3d B;
            // Loop over transform_deque to find where the trajectory is nearest to P
            for (int i = 1; i < transform_deque.size(); i++)
            {
                // Use loop to walk through the deque, trying every consecutive segment AB
                Eigen::Affine3d A_loop = transform_deque[i-1];
                Eigen::Affine3d B_loop = transform_deque[i];

                // Find where this segment AB is nearest to P
                std::pair<Eigen::Vector3d, double> pair = getNearestTranslation(A_loop, B_loop, P);
                double this_distance = pair.first.norm();
                // If closest segment yet, replace best
                if (this_distance < min_distance)
                {
                    best_index = i;
                    min_distance = this_distance;
                    translation = pair.first;
                    interpolation_param = pair.second;
                    A = A_loop;
                    B = B_loop;
                }
            }

            // Include rotations when calcuating the full interpolated transform
            Eigen::Affine3d ABinterp = getInterpolatedTransform(A, B, interpolation_param);
            // Find the relative translation from P to ABinterp
            Eigen::Affine3d nearest = getRelativeTransform(P, ABinterp);

            // Find the vector from P to a point forward of the nearest point by 1 waypoint's distance
            ROS_INFO_STREAM(std::to_string(interpolation_param));
            Eigen::Affine3d forward;
            // Get the next waypoint along the trajectory
            int C_index = best_index+1;
            if ((C_index + 1) > transform_deque.size())
            {
                C_index = best_index;
            }
            Eigen::Affine3d C = transform_deque[C_index];
            // Interpolate the forward point along the trajectory
            Eigen::Affine3d BCinterp = getInterpolatedTransform(B, C, interpolation_param);
            // Find the relative translation from P to BCinterp
            forward = getRelativeTransform(P, BCinterp);

            // Package result into vector
            std::vector<Eigen::Affine3d> out;
            out.push_back(nearest);
            out.push_back(forward);
            return out;
        }

        std::pair<Eigen::Vector3d, double> getNearestTranslation(Eigen::Affine3d A, Eigen::Affine3d B, Eigen::Affine3d P)
        {
            // Use translations for calculating nearest point
            Eigen::Vector3d At = A.translation();
            Eigen::Vector3d Bt = B.translation();
            Eigen::Vector3d Pt = P.translation();
            // Calculate relative translations
            Eigen::Vector3d ABt = Bt - At;
            Eigen::Vector3d APt = Pt - At;
            double interpolation_param = APt.dot(ABt) / ABt.squaredNorm();
            // Clamp interpolation to the bounds of the line segment
            if (interpolation_param < 0)
            {
                interpolation_param = 0;
            }
            else if (interpolation_param > 1)
            {
                interpolation_param = 1;
            }

            Eigen::Vector3d translation = (interpolation_param * ABt + At) - Pt;
            std::pair<Eigen::Vector3d, double> out;
            out.first = translation;
            out.second = interpolation_param;
            return out;
        }

        Eigen::Affine3d getInterpolatedTransform(Eigen::Affine3d A, Eigen::Affine3d B, double interpolation_param)
        {
            Eigen::Affine3d interp;
            // Use translations for calculating the interpolated translation
            Eigen::Vector3d At(A.translation());
            Eigen::Vector3d Bt(B.translation());
            interp.translation() = interpolation_param * Bt + (1 - interpolation_param) * At;
            // Use Eigen's slerp for calculating the interpolated rotation
            Eigen::Quaterniond Ar(A.linear());
            Eigen::Quaterniond Br(B.linear());
            interp.linear() = Ar.slerp(interpolation_param, Br).toRotationMatrix();
            return interp;
        }

        Eigen::Affine3d getRelativeTransform(Eigen::Affine3d A, Eigen::Affine3d B)
        {
            Eigen::Affine3d R;
            // Use translations for calculating difference of translations
            Eigen::Vector3d At(A.translation());
            Eigen::Vector3d Bt(B.translation());
            R.translation() = Bt - At;
            // Invert to calculate relative rotation, since RA = B --> R = B(A^-1)
            Eigen::Quaterniond Ar(A.linear());
            Eigen::Quaterniond Br(B.linear());
            R.linear() = (Br * (Ar.inverse())).toRotationMatrix();
            return R;
        }

        void publishTrajectory(std::deque<Eigen::Affine3d> transform_deque)
        {
            // Convert transform deque to string
            std::stringstream ss;
            for (Eigen::Affine3d transform : transform_deque)
            {
                Eigen::Vector3d translation = transform.translation();
                ss << '[';
                ss << translation[0] << ", ";
                ss << translation[1] << ", ";
                ss << translation[2];
                ss << ']';
            }
            std_msgs::String trajectory_msg;
            trajectory_msg.data = ss.str();

            // Publish
            g_trajectory_publisher.publish(trajectory_msg);
        }

        void publishMarkers(visualization_msgs::MarkerArray& markers)
        {
            // delete old markers
            if (!g_trajectory_marker_array.markers.empty())
            {
                for (auto& marker : g_trajectory_marker_array.markers)
                marker.action = visualization_msgs::Marker::DELETE;

                g_marker_array_publisher.publish(g_trajectory_marker_array);
            }

            // move new markers into g_trajectory_marker_array
            std::swap(g_trajectory_marker_array.markers, markers.markers);

            // draw new markers (if there are any)
            if (!g_trajectory_marker_array.markers.empty())
                g_marker_array_publisher.publish(g_trajectory_marker_array);
        }

        void visualizeTrajectory(std::deque<Eigen::Affine3d> transform_deque)
        {
            //// Visualize the Trajectory itself /////

            // Set a color for the visualization markers
            std_msgs::ColorRGBA color;
            color.r = 0.0;
            color.g = 1.0;
            color.b = 0.0;
            color.a = 0.5;

            // Instantiate marker array for holding the markers to be visualized
            visualization_msgs::MarkerArray markers;
            std::map<std::string, unsigned> ns_counts;

            // Loop over transform_deque to visualize each segment on the trajectory
            for (int i = 1; i < transform_deque.size(); i++)
            {
                // Use loop to walk through the deque, visualizing every consecutive segment AB
                Eigen::Affine3d transform_A = transform_deque[i-1];
                Eigen::Vector3d translation_A = transform_A.translation();
                geometry_msgs::Point point_A;
                point_A.x = translation_A[0];
                point_A.y = translation_A[1];
                point_A.z = translation_A[2];
                Eigen::Affine3d transform_B = transform_deque[i];
                Eigen::Vector3d translation_B = transform_B.translation();
                geometry_msgs::Point point_B;
                point_B.x = translation_B[0];
                point_B.y = translation_B[1];
                point_B.z = translation_B[2];

                // Create a marker for this segment AB
                std::vector<geometry_msgs::Point> points; // Put points in the array type accepted by Marker
                points.push_back(point_A);
                points.push_back(point_B);
                
                std::stringstream ss;
                ss << (i - 1) << "=" << i;
                std::string ns_name = ss.str(); // String name
                if (ns_counts.find(ns_name) == ns_counts.end())
                    ns_counts[ns_name] = 0;
                else
                    ns_counts[ns_name]++;
                visualization_msgs::Marker mk; // Instantiate marker
                mk.header.stamp = ros::Time::now(); // Timestamp
                mk.header.frame_id = "world"; // Reference frame id
                mk.ns = ns_name; // String name
                mk.id = ns_counts[ns_name]; // Unique number id
                mk.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
                mk.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
                mk.points = points; // Start and end points of arrow
                mk.scale.x = 0.005; // Arrow shaft diameter
                mk.scale.y = 0.015; // Arrow head diameter
                mk.scale.z = 0.015; // Arrow head length
                mk.color = color; // Color specified above
                mk.lifetime = ros::Duration(1); // Remain for 1 second or until updated
                markers.markers.push_back(mk); // Add to MarkerArray markers
            }

            //// Visualize the tcp -> Trajectory transform
            
            // Set a different color for the tcp -> Trajectory transform
            std_msgs::ColorRGBA tcp_color;
            tcp_color.r = 0.0;
            tcp_color.g = 0.0;
            tcp_color.b = 1.0;
            tcp_color.a = 0.5;

            // Add a marker for the tcp -> Trajectory transform
            auto current_state = g_planning_scene_monitor->getPlanningScene()->getCurrentState();
            Eigen::Vector3d tcp_translation = current_state.getGlobalLinkTransform("tcp_link").translation();
            Eigen::Vector3d nearest_translation = g_nearest.translation();
            geometry_msgs::Point tcp_point;
            tcp_point.x = tcp_translation[0];
            tcp_point.y = tcp_translation[1];
            tcp_point.z = tcp_translation[2];
            geometry_msgs::Point nearest_point;
            nearest_point.x = tcp_translation[0] + nearest_translation[0];
            nearest_point.y = tcp_translation[1] + nearest_translation[1];
            nearest_point.z = tcp_translation[2] + nearest_translation[2];

            std::stringstream ss;
            ss << nearest_translation[0] << " " << nearest_translation[1] << " " << nearest_translation[2];
            ROS_INFO_STREAM(ss.str());

            std::vector<geometry_msgs::Point> points;
            points.push_back(tcp_point);
            points.push_back(nearest_point);

            std::string ns_name = "tcp_marker"; // String name
            if (ns_counts.find(ns_name) == ns_counts.end())
                ns_counts[ns_name] = 0;
            else
                ns_counts[ns_name]++;
            visualization_msgs::Marker mk; // Instantiate marker
            mk.header.stamp = ros::Time::now(); // Timestamp
            mk.header.frame_id = "world"; // Reference frame id
            mk.ns = ns_name; // String name
            mk.id = ns_counts[ns_name]; // Unique number id
            mk.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
            mk.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
            mk.points = points; // Start and end points of arrow
            mk.scale.x = 0.01; // Arrow shaft diameter
            mk.scale.y = 0.015; // Arrow head diameter
            mk.scale.z = 0.015; // Arrow head length
            mk.color = tcp_color; // Color specified above
            mk.lifetime = ros::Duration(1); // Remain for 1 second or until updated
            markers.markers.push_back(mk); // Add to MarkerArray markers

            //// Visualize the tcp --> forward transform
            
            // Set a different color for the tcp --> forward transform
            std_msgs::ColorRGBA forward_color;
            forward_color.r = 1.0;
            forward_color.g = 0.0;
            forward_color.b = 0.0;
            forward_color.a = 0.5;

            // Add a marker for the tcp --> forward transform
            Eigen::Vector3d forward_translation = g_forward.translation();
            geometry_msgs::Point forward_point;
            forward_point.x = tcp_translation[0] + forward_translation[0];
            forward_point.y = tcp_translation[1] + forward_translation[1];
            forward_point.z = tcp_translation[2] + forward_translation[2];

            std::vector<geometry_msgs::Point> points_forward;
            points_forward.push_back(tcp_point);
            points_forward.push_back(forward_point);

            std::string ns_name_forward = "forward_marker"; // String name
            if (ns_counts.find(ns_name_forward) == ns_counts.end())
                ns_counts[ns_name_forward] = 0;
            else
                ns_counts[ns_name_forward]++;
            visualization_msgs::Marker mk_forward; // Instantiate marker
            mk_forward.header.stamp = ros::Time::now(); // Timestamp
            mk_forward.header.frame_id = "world"; // Reference frame id
            mk_forward.ns = ns_name_forward; // String name
            mk_forward.id = ns_counts[ns_name_forward]; // Unique number id
            mk_forward.type = visualization_msgs::Marker::ARROW; // Arrow marker shape
            mk_forward.action = visualization_msgs::Marker::ADD; // Add shape to Rviz
            mk_forward.points = points_forward; // Start and end points of arrow
            mk_forward.scale.x = 0.01; // Arrow shaft diameter
            mk_forward.scale.y = 0.015; // Arrow head diameter
            mk_forward.scale.z = 0.015; // Arrow head length
            mk_forward.color = forward_color; // Color specified above
            mk_forward.lifetime = ros::Duration(1); // Remain for 1 second or until updated
            markers.markers.push_back(mk_forward); // Add to MarkerArray markers


            //// Update markers to be published ////
            publishMarkers(markers);
        }
        
    private:
        // Define a callback to update to be called when the PlanningSceneMonitor receives an update
        static void planningSceneMonitorCallback(const moveit_msgs::PlanningScene::ConstPtr& planning_scene, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor)
        {
            ROS_INFO("Updating...");
        }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory");
    TrajectoryNode trajectory_node(argc, argv);
    trajectory_node.spin();
    return 0;
}
