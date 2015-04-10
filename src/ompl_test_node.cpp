#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <fstream>

//Create the namespaces
namespace ob = ompl::base ;
namespace og = ompl::geometric ;

//Class to check the validity of the states
class ValidityChecker : public ob::StateValidityChecker
{
public:
    ValidityChecker(const ob::SpaceInformationPtr& si) :
        ob::StateValidityChecker(si) {}

    //Function to check if state is valid
    bool isValid(const ob::State* state) const
    {
        return this->clearance(state) > 0.0 ;
    }

    //Function to calculate distance of state from obstacle
    double clearance(const ob::State* state) const
    {
        const ob::RealVectorStateSpace::StateType* state3D = state->as<ob::RealVectorStateSpace::StateType>() ;

        //Extract robot position
        double x = state3D->values[0] ;
        double y = state3D->values[1] ;
        double z = state3D->values[2] ;

        //Calculate Distance
        return sqrt((x-0.5)*(x-0.5) + (y-0.5)*(y-0.5) + (z-0.5)*(z-0.5) - 0.25) ;
    }
};

//Function to plan
void plan(int argc, char** argv)
{
    //Construct the robot state space
    ob::StateSpacePtr r_space(new ob::RealVectorStateSpace(3)) ;

    //Set the bounds of the space
    r_space->as<ob::RealVectorStateSpace>()->setBounds(0.0,1.0) ;

    //Construct the space information instance
    ob::SpaceInformationPtr si_ptr(new ob::SpaceInformation(r_space)) ;

    //Use the object to check for validity of states
    si_ptr->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si_ptr))) ;

    si_ptr->setup() ;

    //Set robot start state
    ob::ScopedState<> start(r_space) ;
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0 ;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.0 ;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0.0 ;

    //Set robot goal state
    ob::ScopedState<> goal(r_space) ;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0 ;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.0 ;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = 1.0 ;

    //Create instance of problem definition
    ob::ProblemDefinitionPtr pd(new ob::ProblemDefinition(si_ptr)) ;

    //Set the start and goal states
    pd->setStartAndGoalStates(start,goal) ;

    //Using the RRT* planner
    ob::PlannerPtr my_planner(new og::RRTstar(si_ptr)) ;

    //Set instance of problem to solve
    my_planner->setProblemDefinition(pd) ;
    my_planner->setup() ;

    //Attempting to solve the problem within 5 second time
    ob::PlannerStatus solved = my_planner->solve(5.0) ;

    if(solved)
    {
        //Output the length of the path
        std::cout
                << "Found solution of Length "
                << pd->getSolutionPath()->length() << std::endl ;

        //If a filename is specified output path as matrix to that file
        if(argc > 1)
        {
            std::ofstream outfile(argv[1]) ;
            boost::static_pointer_cast<og::PathGeometric>(pd->getSolutionPath())->printAsMatrix(outfile) ;
            outfile.close() ;
        }
    }
    else
        std::cout << "No Solution Found!!" <<std::endl ;

}

int main(int argc, char** argv)
{
    plan(argc,argv) ;
    return 0 ;
}
