/*
Primitive that generates a robot trajectory between actual robot state and one or several given points, with a cartesian or articular mouvement.
*/
#include "ros/ros.h"
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>
#include "skills_msgs/GenerateTrajectoryAction.h"
#include <iostream>
#include <fstream>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
using namespace Eigen;
using namespace std;
#include <sstream>
string GetConfig(string file_name)
{
	ifstream config;
	string line;
	string path = ros::package::getPath("skills") + "/config/" + file_name;
	config.open(path);
	if (config.is_open())
	{ // checking whether the file is open
		string tp;
		getline(config, line); // read data from file object and put it into string.
		config.close();		   // close the file object.
	}
	return line;
}
class GenerateTrajectoryAction
{
private:
	std::string PLANNING_GROUP;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const moveit::core::JointModelGroup *joint_model_group;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	double rate;
	bool success;

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<skills_msgs::GenerateTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
	std::string action_name_;
	// create messages that are used to published feedback/result
	skills_msgs::GenerateTrajectoryFeedback feedback_;
	skills_msgs::GenerateTrajectoryResult result_;

public:
	GenerateTrajectoryAction(std::string name) : as_(nh_, name, boost::bind(&GenerateTrajectoryAction::executeCB, this, _1), false),
																			action_name_(name)
	{
		as_.start();
		ROS_INFO("GenerateTrajectory server started");
	}
	~GenerateTrajectoryAction(void) {}

	void executeCB(const skills_msgs::GenerateTrajectoryGoalConstPtr &goal)
	{	
		ROS_INFO("Goal received");
		if ((goal->points.empty() && goal->articular_pose.empty()) || (!goal->points.empty() && !goal->articular_pose.empty()))
		{
			if (goal->points.empty() && goal->articular_pose.empty())
			{
				ROS_WARN("No pose or angle list given");
				ROS_INFO("%s: Aborted", action_name_.c_str());
				as_.setAborted();
			}
			if (!goal->points.empty() && !goal->articular_pose.empty())
			{
				ROS_WARN("Pose AND angle list given");
				ROS_INFO("%s: Aborted", action_name_.c_str());
				as_.setAborted();
			}
		}
		else{
			success = false;
			if (goal->planning_group.empty())
			{
				ROS_WARN("No planning group given, taking 'arm' as default one");
				PLANNING_GROUP="arm";
			}
			else
			{
				PLANNING_GROUP = goal->planning_group;
			}
			moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
			joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

			move_group_interface.setMaxVelocityScalingFactor(goal->velocity_factor);
			move_group_interface.setMaxAccelerationScalingFactor(goal->acceleration_factor);
			move_group_interface.limitMaxCartesianLinkSpeed(goal->cartesian_speed_limit, move_group_interface.getLinkNames()[6]);

			string frame_id = goal->frame_id;
			if (frame_id.empty())
			{
				ROS_WARN("frame_id not specified -> set to base_link");
				std::cout << "link name: " << std::endl
						<< move_group_interface.getPlanningFrame() << std::endl;
				frame_id = move_group_interface.getPlanningFrame();
			}
			move_group_interface.setPoseReferenceFrame(frame_id);
			if (goal->cartesian && !goal->points.empty())
			{
				// check if endeffector is defined, else abort
				if (move_group_interface.setEndEffector(goal->endeffector))
				{
					ROS_INFO("Cartesian movement selected");
					std::vector<geometry_msgs::Pose> waypoints;
					for (auto &pose : goal->points)
					{
						waypoints.push_back(pose);
					}
					// We want the Cartesian path to be interpolated at a resolution of 1 cm
					// which is why we will specify 0.01 as the max step in Cartesian
					// translation. Specifying the jump threshold as 0.0, will effectively disable it.
					// Warning - disabling the jump threshold while operating real hardware can cause
					// large unpredictable motions of redundant joints and could be a safety issue
					moveit_msgs::RobotTrajectory trajectory;
					const double jump_threshold = 10;
					const double eef_step = 0.01;
					double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
					if (fraction == 1)
					{
						success = true;
						result_.trajectory = trajectory;
					}
				}
				else
				{
					if (goal->endeffector.empty()){
						ROS_WARN("endeffector not specified in goal message");
					}
					else{
						ROS_WARN("endeffector not defined in moveit config");
					}
				}
			}
			else
			{
				if (goal->cartesian)
				{
					ROS_WARN("Cartesian movement specified but no pose given");
				}
				ROS_INFO("Articular movement");
				if (!goal->articular_pose.empty())
				{
					moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
					std::vector<double> joint_group_positions;
					current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
					for (int i = 0; i < joint_group_positions.size(); i++)
					{
						joint_group_positions[i] = goal->articular_pose[i];
					}
					move_group_interface.setJointValueTarget(joint_group_positions);
					if (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
					{
						success = true;
						result_.trajectory = my_plan.trajectory_;
					}
				}
				else
				{
					// If points given as input :
					// reverse kinematics to recalculate articular pose -> 
					// movement as above
					
					ROS_INFO("Converting cartesian target");
					for (int i=0; i<goal->points.size(); i++)
					{
						move_group_interface.setPoseTarget(goal->points[i]);
						if (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
						{
							result_.trajectory = my_plan.trajectory_;
						}
					}
					success = true; 
				}
			}
			if (success)
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				as_.setSucceeded(result_);
			}
			else
			{
				ROS_INFO("%s: Aborted", action_name_.c_str());
				as_.setAborted();
			}
		}
	}
};
int main(int argc, char **argv)
{
	ros::init(argc, argv, "GenerateTrajectory");

	GenerateTrajectoryAction GenTraj("GenerateTrajectory");
	ros::spin();

	return 0;
}