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
    	dof = _dof.get();

    	ls_method = new LS_Pseudo(dof);

    	first_time = true;
    	doit = false;

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

    adap_samples_input::DynamicAUV dynamic;
    adap_samples_input::DynamicAUV last_dynamic;

    base::MatrixXd parameters;
    parameters.resize(4,1);
    double error;

    if(first_time)
    {
    	while(!queueOfDyn.empty())
    		queueOfDyn.pop();
    	first_time = false;
    }


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
    	//static int dyn_diff = 0;

    	ls_method->ls_solution(queueOfDyn, parameters, error);

    	adap_parameters_estimator::Parameters modelParameters;

    	// Initialize output parameters
    	for (int i=0; i<6; i++)
    						{
    						 modelParameters.inertiaCoeff[i].positive = 1;
    						 modelParameters.inertiaCoeff[i].negative = 1;
    						 modelParameters.quadraticDampingCoeff[i].positive = 0;
    						 modelParameters.quadraticDampingCoeff[i].negative = 0;
    						 modelParameters.linearDampingCoeff[i].positive = 0;
    						 modelParameters.linearDampingCoeff[i].negative = 0;
    						}
    	modelParameters.gravityAndBuoyancy = base::VectorXd::Zero(6);
    	modelParameters.coriolisCentripetalMatrix = base::MatrixXd::Zero(6,6);

    	modelParameters.inertiaCoeff[dof].positive           = parameters(0,0);
    	modelParameters.quadraticDampingCoeff[dof].positive  = parameters(1,0);
    	modelParameters.linearDampingCoeff[dof].positive     = parameters(2,0);
    	modelParameters.gravityAndBuoyancy[dof]              = parameters(3,0);

    	_parameters.write(modelParameters);
    	_relativ_error.write(error);


/*    	while(!queueOfDyn.empty())
    	    	{
    	    		dynamic = queueOfDyn.front();
    	    		queueOfDyn.pop();
    	    		if ((dynamic.time-last_dynamic.time) > base::Time::fromMilliseconds(100))
    	    			{
    	    				dyn_diff++;
    	    			}
    	    		last_dynamic = dynamic;
    	    	}
*/
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
