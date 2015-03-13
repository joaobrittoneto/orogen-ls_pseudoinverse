/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace ls_pseudoinverse;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
	delete ls_method;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    else
    {
    	adap_parameters_estimator::DOFS dof = _dof.get();
    	ls_method = new LS_Pseudo(dof);

    	return true;
    }
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    static bool first_time = true;

   // static std::queue<base::samples::RigidBodyState>		queueOfRBS;
   // static std::queue<base::samples::RigidBodyAcceleration>	queueOfRBA;
   // static std::queue<base::samples::Joints> 				queueOfForces;
    static std::queue<adap_samples_input::DynamicAUV> 		queueOfDyn;

   // base::samples::RigidBodyState 		 RBS;
   // base::samples::RigidBodyAcceleration RBA;
   // base::samples::Joints 				 force;
    adap_samples_input::DynamicAUV dynamic;
    adap_samples_input::DynamicAUV last_dynamic;


   // static base::Time new_vel;
   // static base::Time new_acc;
   // static base::Time new_for;
    static base::Time new_dyn;

    static bool doit = false;
    base::MatrixXd parameters;
    parameters.resize(4,1);
    double error;

    if(first_time)
    {
    	while(!queueOfDyn.empty())
    		queueOfDyn.pop();
    //	while(!queueOfForces.empty())
    //		queueOfForces.pop();
    //	while(!queueOfRBA.empty())
    //		queueOfRBA.pop();
    //	while(!queueOfRBS.empty())
    //		queueOfRBS.pop();
    	first_time = false;
    }

/*
	 while (_speed_samples.read(RBS) == RTT::NewData)
       {
       	queueOfRBS.push(RBS);
       	new_vel = base::Time::now();
       	doit = true;
       }


    while (_acceleration_samples.read(RBA) == RTT::NewData)
       {
       queueOfRBA.push(RBA);
       new_acc = base::Time::now();
       }

    while (_forces_samples.read(force) == RTT::NewData)
       {
       queueOfForces.push(force);
       new_for = base::Time::now();
       }

	*/
 /*   static int samples = 0;
    while (_speed_samples.read(RBS) == RTT::NewData && _acceleration_samples.read(RBA) == RTT::NewData && _forces_samples.read(force) == RTT::NewData)
    {
    	queueOfRBS.push(RBS);
    	queueOfRBA.push(RBA);
    	queueOfForces.push(force);

    	new_vel = base::Time::now();
    	doit = true;

    	samples++;

    }
*/
    static int dyn_samples = 0;
    while (_dynamic_samples.read(dynamic) == RTT::NewData )
       {
       	queueOfDyn.push(dynamic);

       	new_dyn = base::Time::now();
       	doit = true;

       	dyn_samples++;
       //	std::cout<<std::endl<< "queuOfDyn.size: "<< queueOfDyn.size() <<std::endl;

       }

    base::Time t_5 	 = base::Time::fromSeconds(5);
    base::Time t_10  = base::Time::fromSeconds(10);
    base::Time t_now = base::Time::now();

    if(doit && (t_now-new_dyn)>t_10)
       {
    	//std::cout<<std::endl<< "queuOfDyn.size: "<< queueOfDyn.size() <<std::endl;
    	last_dynamic = queueOfDyn.front();
    	static int dyn_diff = 0;

    	ls_method->ls_solution(queueOfDyn, parameters, error);

    	while(!queueOfDyn.empty())
    	    	{

    	    		dynamic = queueOfDyn.front();
    	    		queueOfDyn.pop();
    	    		if ((dynamic.time-last_dynamic.time) > base::Time::fromMilliseconds(100))
    	    			{
    	    				dyn_diff++;

    	    			}
    	    		last_dynamic = dynamic;




    	    	}


  /*  if(doit && (t_now-new_vel)>t_5)// && (t_now-new_acc)>t_5 && (t_now-new_for)>t_5 )
    {
    	static int rba_rbs = 0;
    	static int rba_force = 0;
    	static int force_rbs = 0;
    	while(!queueOfForces.empty() && !queueOfRBA.empty() && !queueOfRBS.empty())
    	{

    		RBS = queueOfRBS.front();
    		RBA = queueOfRBA.front();
    		force = queueOfForces.front();
    		queueOfForces.pop();
    		queueOfRBA.pop();
    		queueOfRBS.pop();

    		if(RBS.time!=RBA.time)
    			rba_rbs++;
    		if(RBS.time!=force.time)
    			force_rbs++;
    		if(RBA.time!=force.time)
    			rba_force++;
    		std::cout<<std::endl<< "RBA.time: "<< RBA.time <<std::endl;
    		std::cout<<std::endl<< "RBS.time: "<< RBS.time <<std::endl;
    		std::cout<<std::endl<< "force.time: "<< force.time <<std::endl;

    	}
	*/

/*
    	if (queueOfRBS.size()!=queueOfForces.size() || queueOfRBA.size()!=queueOfForces.size() || queueOfRBA.size()!=queueOfRBS.size())
    		{
    		std::cout<<std::endl<< "Number of samples of velocity, acceleration and force should be equal" <<std::endl;
    		std::cout<<std::endl<< "queueRBS.size: "<< queueOfRBS.size() <<std::endl;
    		std::cout<<std::endl<< "queueRBA.size: "<< queueOfRBA.size() <<std::endl;
    		std::cout<<std::endl<< "queueforces.size: "<< queueOfForces.size() <<std::endl;
    		}
    	else
    	{
    		std::cout<<std::endl<< "number of samples in queue: "<< queueOfRBS.size() <<std::endl;
    	//	ls_method->ls_solution(queueOfForces,queueOfRBS, queueOfRBA, parameters, error);

    	//	std::cout<<std::endl<< "parameters: "<< parameters <<std::endl;
    	//	std::cout<<std::endl<< "error: "<< error <<std::endl;
    	}

*/
    	//std::cout<<std::endl<< "dyn_diff: "<< dyn_diff <<std::endl;
    	//std::cout<<std::endl<< "dyn_samples: "<< dyn_samples <<std::endl;

    	//std::cout<<std::endl<< "force_rbs: "<< force_rbs <<std::endl;


    	doit = false;
    }


}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
