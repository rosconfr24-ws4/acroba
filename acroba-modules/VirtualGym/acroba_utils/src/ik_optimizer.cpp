#include "acroba_utils/ik_optimizer.h"



namespace ikoptimizerCpp
{
	void printJoints(std::vector<double> &joints)
	{
		printf("[");
		for (int i=0;i<joints.size();++i)
		{
			printf("%3.3f,",joints[i]);
		}
		printf("]");
	}

	void printArray(KDL::JntArray  joints)
	{
		printf("[");
		for (int i=0;i<joints.rows();++i)
		{
			printf("%3.3f,",joints(i));
		}
		printf("]");
	}

	bool pairCompare(std::pair<double,int> p1,std::pair<double,int> p2)
	{
		return p1.first<p2.first;
	}
}

#ifndef USE_TRAC_IK
	void printLink(const KDL::SegmentMap::const_iterator & link, const std::string & prefix)
{
 std::cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() <<
   " has " << GetTreeElementChildren(link->second).size() << " children" << std::endl;
 for (unsigned int i = 0; i < GetTreeElementChildren(link->second).size(); i++) {
   printLink(GetTreeElementChildren(link->second)[i], prefix + "  ");
 }
}

#endif

	IKOptimizer::IKOptimizer(const std::string &chain_start,const std::string &chain_end,const std::string& urdf_param, int num_seeds)
	{
		//initialize solver
		double time_out=0.05;
		double eps=1e-5;
		ROS_INFO("[IK OPTIMIZER] Constructing solver with args: %s, %s, %s, %i",chain_start.c_str(),chain_end.c_str(),urdf_param.c_str(),num_seeds);

#ifdef USE_TRAC_IK
		ROS_INFO("[IK OPTIMIZER] Using TRAC IK SOLVER");
		_tracik_solver=new TRAC_IK::TRAC_IK(chain_start, chain_end, urdf_param, time_out, eps);
			//lower joint limits, upper joint limits
  	initializeSolver();
#else
		ROS_INFO("[IK OPTIMIZER] Using KDL IK SOLVER");
 		KDL::Tree my_tree;
 		if (!kdl_parser::treeFromParam(urdf_param, my_tree)) 
			 return;
		//KDL::SegmentMap::const_iterator root = my_tree.getRootSegment();
 		//printLink(root, "");
		//KDL::Chain robot_chain;
 		bool isok=my_tree.getChain("world", "end_tool", _chain);
		//_chain=robot_chain;
		_kdl_solver= new KDL::ChainIkSolverPos_LMA(_chain);

#endif
	_num_seed=num_seeds;
  	//_seeds.resize(num_seeds);
  	initializeSeeds();
	}

	IKOptimizer::~IKOptimizer()
	{

	}

	void IKOptimizer::updateSeed(std::vector<double> &joints)
	{
#ifdef IK_DEBUGGER
	ROS_INFO("[IK OPTIMIZER] updating seeds");
#endif
		if (_seeds.size()>0)
		{
			_seeds[0].data=Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(joints.data(),joints.size());
			_prev_q=_seeds[0];
#ifdef IK_DEBUGGER
		printf("Main seed (previous pose) is");
		ikoptimizerCpp::printArray(_prev_q);
		printf("\n");
#endif //IK_DEBUGGER
		}
	}

	bool IKOptimizer::getIKSol(geometry_msgs::Pose &pose,std::vector<double> &sol)
	{
		tf::PoseMsgToKDL(pose,_end_effector_pose);
		
		//get all solutions
     getAllSols(_end_effector_pose);

     if(_solutions.size()<1)
        return false;

		KDL::JntArray qout;
		closest_joint_conf(_prev_q,_solutions,qout);

		//KDL to vectro
		jntArrayToVector(qout,sol);
		return true;

	}

	bool IKOptimizer::getIKReport(geometry_msgs::Pose &pose,std::vector<std::vector<double> > &sols)
	{
#ifdef IK_DEBUGGER
		ROS_INFO("[IKOPTIMIZER] Received pose for IK REPORT %f,%f,%f\n",pose.position.x,pose.position.y,pose.position.z);
		
#endif
		tf::PoseMsgToKDL(pose,_end_effector_pose);
#ifdef IK_DEBUGGER
		printf("[IKOPTIMIZER] Pose Msg to KDL done");
		
#endif
		//get all solutions
    getAllSols(_end_effector_pose);

    if(_solutions.size()<1)
      return false;

    _solCosts.resize(_solutions.size());
    sols.resize(_solutions.size());
   
    std::vector<std::pair<double,int> >v_cost_idx;
    //v_cost_idx.resize(_solutions.size());
    v_cost_idx.clear();
    for(int i=0;i<_solutions.size();++i)
		{
			_solCosts[i]=eval_closeness(_prev_q,_solutions[i]);
			jntArrayToVector(_solutions[i],sols[i]);
			v_cost_idx.push_back(std::make_pair(_solCosts[i],i));
		}

		//sort v_cost_idx
		std::sort(v_cost_idx.begin(),v_cost_idx.end(),ikoptimizerCpp::pairCompare);

		 _sortedCosts.resize(_solutions.size());
    _sortedSolutions.resize(_solutions.size());
		for(uint32_t i=0;i<v_cost_idx.size();++i)
		{
			_sortedCosts[i]=v_cost_idx[i].first;
			_sortedSolutions[i]=sols[v_cost_idx[i].second];
#ifdef IK_DEBUGGER
			printf("Sorted Cost is %f for",_sortedCosts[i]);
			ikoptimizerCpp::printJoints(_sortedSolutions[i]);
			printf("\n");
#endif //IK_DEBUGGER
		}
		sols=_sortedSolutions;

		return true;
	}

	std::vector<double> IKOptimizer::getSortedCosts()
	{
		return _sortedCosts;
	}

	std::vector<std::vector<double> > IKOptimizer::getSortedSolutions()
	{
		return _sortedSolutions;
	}

#ifdef USE_TRAC_IK
	void IKOptimizer::initializeSolver(){
	 bool valid = _tracik_solver->getKDLChain(_chain);

	  if (!valid)
	  {
	    ROS_ERROR("There was no valid KDL chain found");
	  }
	  valid = _tracik_solver->getKDLLimits(_ll, _ul);

	  if (!valid)
	  {
	    ROS_ERROR("There were no valid KDL joint limits found");
	  }

		assert(_chain.getNrOfJoints() == _ll.data.size());
		assert(_chain.getNrOfJoints() == _ul.data.size());

	}
#endif
	void IKOptimizer::initializeSeeds(){
		
		//Initialize seeds 
		#ifdef IK_DEBUGGER
		ROS_INFO("initializing seeds");
		#endif
		_seeds.clear();
		for(int i=0;i<_num_seed;++i)
		{
			KDL::JntArray q_aux(_chain.getNrOfJoints());
			for (uint32_t j=0;j<_chain.getNrOfJoints();++j)
			{
			  if (i==1)
			    q_aux(j)=0;
			  else
			  {
				  #ifdef USE_TRAC_IK
				  q_aux(j)=_ll(j)+static_cast<float> (rand())/(static_cast <float> (RAND_MAX/(_ul(j)-_ll(j))));
				  #else 
				  q_aux(j)=-5+static_cast<float> (rand())/(static_cast <float> (RAND_MAX/(5-(-5))));
				  #endif
			  }
			}
			_seeds.push_back(q_aux);
		}
	}

	void IKOptimizer::getAllSols(KDL::Frame end_pose)
	{
#ifdef IK_DEBUGGER
		printf("initialize results");
#endif
		//initialize with right size
		KDL::JntArray result(_seeds[0].rows());
		int rc;
		_solutions.clear();


		 for (uint32_t i=0;i<_seeds.size();++i)
        {
#ifdef USE_TRAC_IK
            rc = _tracik_solver->CartToJnt(_seeds[i], end_pose, result);
#else
			rc=_kdl_solver->CartToJnt(_seeds[i], end_pose, result);
#endif
#ifdef IK_DEBUGGER
            printf("resul is %d\n",rc);
            ikoptimizerCpp::printArray(_seeds[i]);
            ikoptimizerCpp::printArray(result);
#endif //IK_DEBUGGER
            if (rc >= 0)
            {
              _solutions.push_back(result);
            }
        }

	}

	//This closeness function penalize movement on the primary joints
	double IKOptimizer::eval_closeness(KDL::JntArray q1,KDL::JntArray q2)
	{
	//TODO check same lenght 
		double cost=0;
		for(uint32_t i=0;i<q1.rows();++i)
		{
			cost+=(q1.rows()-i)*pow(q1(i)-q2(i),2);
		}
		return cost;
	}
	// double IKOptimizer::eval_closeness(KDL::JntArray q1,KDL::JntArray q2)
	// {
	// //TODO check same lenght 
	// 	double cost=0;
	// 	//consider only the first 3 joints trajectory length
	// 	for(uint i=0;i<q1.rows()-3;++i)
	// 	{
	// 		cost+=(q1.rows()-i)*abs(q1(i)-q2(i));
	// 	}
	// 	return cost;
	// }
	void IKOptimizer::closest_joint_conf(KDL::JntArray &prev,std::vector<KDL::JntArray> &sols,KDL::JntArray &qout)
	{
		double max_cost=DBL_MAX;
		double cost;
		uint32_t idx=0;
		for(uint32_t i=0;i<sols.size();++i)
		{
			cost=eval_closeness(prev,sols[i]);
			//print joint and cost
#ifdef IK_DEBUGGER
        	printf("joints:");
        	ikoptimizerCpp::printArray(sols[i]);
        	printf(" with cost %4.3f\n",cost);


#endif //IK_DEBUGGER

			if (cost<max_cost)
			{
			max_cost=cost;
			idx=i;
			}
		}
		qout=sols[idx];
		return;
	}

	void IKOptimizer::jntArrayToVector(KDL::JntArray &q_arr,std::vector<double> &q_vec)
	{
		//KDL to vectro
		q_vec.resize(q_arr.data.size());
		Eigen::VectorXd::Map(&q_vec[0],q_arr.data.size())=q_arr.data;
	}

	int IKOptimizer::getIdxToInsert(std::vector<double> &sorted, double value)
	{

			int middle=floor(sorted.size()/2);
			printf("middle is: %d, sorted %f and value %f\n,",middle,sorted[middle],value);
			if(middle==0 && value>sorted[middle])
			{
				printf("return 1");
				return 1;
			}
			else if(middle==0 && value<=sorted[middle])
			{
				printf("return 0");
				return 0;
			}
			else if (value==sorted[middle])
			{
				printf("return middle");
				return middle;
			}
			else if (value>sorted[middle])
			{
				printf("return upper half");
				std::vector<double> subVec(sorted.begin()+middle,sorted.end());
				return middle+getIdxToInsert(subVec,value);
			}
			else 
			{
				printf("return down half");
				std::vector<double> subVec(sorted.begin(),sorted.begin()+middle);
				return getIdxToInsert(subVec,value);
			} 
	}



