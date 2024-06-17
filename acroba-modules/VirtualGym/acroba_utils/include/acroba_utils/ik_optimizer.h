#ifndef IK_OPTIMIZER_H
#define IK_OPTIMIZER_H

#include <boost/date_time.hpp>

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "tf_conversions/tf_kdl.h"
#include "geometry_msgs/Pose.h"
#include <algorithm>

#ifdef USE_TRAC_IK
	#include <trac_ik/trac_ik.hpp>
#else
	#include "kdl_conversions/kdl_msg.h"
	#include "kdl_parser/kdl_parser.hpp"
	#include "kdl/chainiksolverpos_lma.hpp"
#endif //USE_TRAC_IK
//#include <ros/ros.h>
//#define IK_DEBUGGER

class IKOptimizer
{
public:

	IKOptimizer(const std::string &chain_start,const std::string &chain_end,const std::string& urdf_param, int num_seeds);
	~IKOptimizer();

	//Actualize one seed as the actual pose
	void updateSeed(std::vector<double> &joints);
	//Look for the closest solution to the updated seed 
	bool getIKSol(geometry_msgs::Pose &pose,std::vector<double> &sol);
	//get sorted list of solution and costs
	bool getIKReport(geometry_msgs::Pose &pose,std::vector<std::vector<double> > &sols);
	//
	std::vector<double> getSortedCosts();
	std::vector<std::vector<double> > getSortedSolutions();
	
private:
#ifdef USE_TRAC_IK
	void initializeSolver();
#endif
	void initializeSeeds();
	void getAllSols(KDL::Frame end_pose);
	double eval_closeness(KDL::JntArray q1,KDL::JntArray q2);
	void closest_joint_conf(KDL::JntArray &prev,std::vector<KDL::JntArray> &sols,KDL::JntArray &qout);
	void jntArrayToVector(KDL::JntArray &q_arr,std::vector<double> &q_vec);
	int getIdxToInsert(std::vector<double> &sorted, double value);

	#ifdef USE_TRAC_IK
		TRAC_IK::TRAC_IK *_tracik_solver;
	#else
		KDL::ChainIkSolverPos_LMA *_kdl_solver;
	#endif
	std::vector<KDL::JntArray> _solutions;
	std::vector<std::vector<double> >_sortedSolutions;
	std::vector<double> _solCosts,_sortedCosts;
	std::vector<double>_qout;
	std::vector<KDL::JntArray> _seeds;
	KDL::Chain _chain;
	KDL::JntArray _ll, _ul; 
	KDL::JntArray _prev_q;
	KDL::Frame _end_effector_pose;
	int _num_seed;

};


#endif