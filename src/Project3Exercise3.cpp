///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

// Your random tree planner
#include "RTP.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/prm/PRM.h"

#include "ompl/tools/benchmark/Benchmark.h"
#include <omplapp/config.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/geometric/SimpleSetup.h>



void benchmarkCubicles()
{
    // plan in SE3
    ompl::app::SE3RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // define start state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(-4.96);
    start->setY(-40.62);
    start->setZ(70.57);
    start->rotation().setIdentity();

    // define goal state
    ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
    goal->setX(200.49);
    goal->setY(-40.62);
    goal->setZ(70.57);
    goal->rotation().setIdentity();

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // we call setup just so print() can show more information
    setup.setup();
    setup.print();

    ompl::tools::Benchmark b(setup, "Cubicles Benchmarking");
    b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RTP(setup.getSpaceInformation())));
    b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRT(setup.getSpaceInformation())));
    b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::PRM(setup.getSpaceInformation())));
    b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::EST(setup.getSpaceInformation())));

    ompl::tools::Benchmark::Request req;
    req.maxTime = 30;
    req.maxMem  = 1000.0;
    req.runCount = 50;
    req.displayProgress = true;
    b.benchmark(req);

    b.saveResultsToFile();
}

void benchmarkTwistycool()
{
  // plan in SE3
  ompl::app::SE3RigidBodyPlanning setup;

  // load the robot and the environment
  std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_robot.dae";
  std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/Twistycool_env.dae";
  setup.setRobotMesh(robot_fname);
  setup.setEnvironmentMesh(env_fname);

  // define start state
  ompl::base::ScopedState<ompl::base::SE3StateSpace> start(setup.getSpaceInformation());
  start->setX(270);
  start->setY(160);
  start->setZ(-200);
  start->rotation().setIdentity();

  // define goal state
  ompl::base::ScopedState<ompl::base::SE3StateSpace> goal(start);
  goal->setX(270);
  goal->setY(160);
  goal->setZ(-400);
  goal->rotation().setIdentity();

  // set the start & goal states
  setup.setStartAndGoalStates(start, goal);

  // setting collision checking resolution to 1% of the space extent
  setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

  // we call setup just so print() can show more information
  setup.setup();
  setup.print();

  ompl::tools::Benchmark b(setup, "my experiment");
  b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RTP(setup.getSpaceInformation())));
  b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::RRT(setup.getSpaceInformation())));
  b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::PRM(setup.getSpaceInformation())));
  b.addPlanner(ompl::base::PlannerPtr(new ompl::geometric::EST(setup.getSpaceInformation())));

  ompl::tools::Benchmark::Request req;
  req.maxTime = 30.0;
  req.maxMem  = 1000.0;
  req.runCount = 50;
  req.displayProgress = true;
  b.benchmark(req);

  b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Cubicles" << std::endl;
        std::cout << " (2) Twistycool" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkCubicles();
            break;
        case 2:
            benchmarkTwistycool();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
