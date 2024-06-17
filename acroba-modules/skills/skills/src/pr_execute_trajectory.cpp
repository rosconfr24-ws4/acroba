/*
Primitive that execute a given trajectory and check the proper execution.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <skills_msgs/ExecuteTrajectoryAction.h>


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

class ExecuteTrajectoryAction
{
private:
	std::string PLANNING_GROUP;
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const moveit::core::JointModelGroup *joint_model_group;
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success;
	bool wait_result;
	double rate;
	moveit_msgs::ExecuteTrajectoryActionResult result_trajectory;

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<skills_msgs::ExecuteTrajectoryAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
	std::string action_name_;
	// create messages that are used to published feedback/result
	skills_msgs::ExecuteTrajectoryFeedback feedback_;
	skills_msgs::ExecuteTrajectoryResult result_;

	ros::Subscriber sub;

public:
	ExecuteTrajectoryAction(const std::string name) : as_(nh_, name, boost::bind(&ExecuteTrajectoryAction::executeCB, this, _1), false),
																				 action_name_(name)
	{
		as_.start();
		rate = 50;
		// subscriber to obtain execute trajectory result from the moveit action server
		sub = nh_.subscribe<moveit_msgs::ExecuteTrajectoryActionResult>("execute_trajectory/result", 1, &ExecuteTrajectoryAction::Sub_callback, this);
	}
	~ExecuteTrajectoryAction(void) {}
	void Sub_callback(moveit_msgs::ExecuteTrajectoryActionResult msg)
	{
		wait_result = false;
		result_trajectory = msg;
	}

	void executeCB(const skills_msgs::ExecuteTrajectoryGoalConstPtr &goal)
	{	
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
		ROS_INFO("Goal received");
		ros::Rate loop_rate(rate);
		success = true;
		wait_result = true;
		move_group_interface.asyncExecute(goal->trajectory);
		/// if moveit shutdown it will never stop because ne result will come
		while (wait_result) // wait until the end execution of the trajectory
		{
			if (as_.isPreemptRequested() || !ros::ok()) // handle cancel order from client
			{
				ROS_INFO("%s: Preempted", action_name_.c_str());
				move_group_interface.stop(); // stop movement
				success = false;
				as_.setPreempted(); // set the action state to preempted
				break;
			}
			loop_rate.sleep();
		}
		if (success)
		{
			if (result_trajectory.result.error_code.val == 1)
			{
				ROS_INFO("%s: Succeeded", action_name_.c_str());
				// set the action state to succeeded
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
	ros::init(argc, argv, "ExecuteTrajectory");
	ExecuteTrajectoryAction ExecTraj("ExecuteTrajectory");
	ros::spin();

	return 0;
}
