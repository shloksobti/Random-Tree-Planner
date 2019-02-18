///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>
#include <fstream>
#include <cmath>
#include <functional>
#include <memory>

// The collision checker produced in project 2
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"
//#include "ompl/geometric/planners/rrt/RRT.h"

// Including SimpleSetup will pull in MOST of what you need to plan
#include <ompl/geometric/SimpleSetup.h>

// Except for the state space definitions...
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

using namespace std::placeholders;

bool isValidStatePoint(const ompl::base::State *state, const std::vector<Rectangle> &obstacles)
{
    // Cast the state to a real vector state
    auto r2state = state->as<ompl::base::RealVectorStateSpace::StateType>();

    // Extract x, y
    double x = r2state->values[0];
    double y = r2state->values[1];

    return isValidPoint(x, y, obstacles);
}

bool isValidStateSquare(const ompl::base::State *state, double sideLength, const std::vector<Rectangle> &obstacles)
{
    // Cast the state to a compound state
    auto cstate = state->as<ompl::base::CompoundState>();

    // Extract the real vector component (x, y)
    auto r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    double x = r2state->values[0];
    double y = r2state->values[1];

    // Extract theta (SO(2))
    auto so2State = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
    double theta = so2State->value;

    return isValidSquare(x, y, theta, sideLength, obstacles);

}

void planPoint(const std::vector<Rectangle> & obstacles)
{
    auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // We need to set bounds on R^2
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-3);  // x and y have a minimum of -2
    bounds.setHigh(3);  // x and y have a maximum of 2

    // Set the bounds on R2
    r2->setBounds(bounds);

    // Create a SimpleSetup container
    ompl::geometric::SimpleSetup ss(r2);

    ss.setStateValidityChecker(std::bind(isValidStatePoint, _1, obstacles));

    // Step 4 Specify the start and goal states
    // ScopedState creates the correct state instance for the state space
    ompl::base::ScopedState<> start(r2);
    start[0] = -1.0;//-1.0
    start[1] = 1.0; //1

    ompl::base::ScopedState<> goal(r2);
    goal[0] = 2.0;
    goal[1] = 1.5;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // set Planner
    auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Step 6) Attempt to solve the problem within the given time (seconds)
    ompl::base::PlannerStatus solved = ss.solve(2.0);
    if (solved)
    {
        // Apply some heuristics to simplify (and prettify) the solution
        ss.simplifySolution();

        // print the path to screen
        std::cout << "Found solution:" << std::endl;
        ompl::geometric::PathGeometric &path = ss.getSolutionPath();
        path.interpolate(50);
        path.printAsMatrix(std::cout);

        // print path to file
        std::ofstream fout("path.txt");
        fout << "R2" << std::endl;
        path.printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;

}

void planBox(const std::vector<Rectangle> &obstacles)
{
      ompl::base::StateSpacePtr se2;

      auto r2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);

      ompl::base::RealVectorBounds bounds(2);
      bounds.setLow(-3);  // x and y have a minimum of -2
      bounds.setHigh(3);  // x and y have a maximum of 2
      r2->setBounds(bounds);

      auto so2 = std::make_shared<ompl::base::SO2StateSpace>();
      se2 = r2 + so2;

      ompl::geometric::SimpleSetup ss(se2);

      ss.setStateValidityChecker(std::bind(isValidStateSquare, _1, 0.3, obstacles));

      ompl::base::ScopedState<> start(se2);
      start[0] = 0.0; //0.0
      start[1] = 0.0;
      start[2] = 0.0;

      ompl::base::ScopedState<> goal(se2);
      goal[0] = 2.0;
      goal[1] = 1.5;
      goal[2] = 0.0;



      ss.setStartAndGoalStates(start, goal);
      auto planner = std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation());
      ss.setPlanner(planner);



      // Step 6) Attempt to solve the problem within the given time (seconds)
      ompl::base::PlannerStatus solved = ss.solve(1.0);

      if (solved)
      {
          // Apply some heuristics to simplify (and prettify) the solution
          ss.simplifySolution();

          // print the path to screen
          std::cout << "Found solution:" << std::endl;
          ompl::geometric::PathGeometric &path = ss.getSolutionPath();
          path.interpolate(50);
          path.printAsMatrix(std::cout);

          // print path to file
          std::ofstream fout("path.txt");
          fout << "SE2" << std::endl;
          path.printAsMatrix(fout);
          fout.close();
      }
      else
          std::cout << "No solution found" << std::endl;

}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    Rectangle rect;

    rect.x = 0.5;
    rect.y = 0;
    rect.width = 2;
    rect.height = 1.0;
    obstacles.push_back(rect);

    rect.x = 0.5;
    rect.y = 1.0;
    rect.width = 1.0;
    rect.height = 1.0;
    obstacles.push_back(rect);
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    Rectangle rect;

    rect.x = 0.5;
    rect.y = 0.0;
    rect.width = 2;
    rect.height = 0.2;
    obstacles.push_back(rect);

    rect.x = 0.5;
    rect.y = 0.0;
    rect.width = 0.2;
    rect.height = 2.0;
    obstacles.push_back(rect);

    rect.x = 0.5;
    rect.y = 2.0;
    rect.width = 2.0;
    rect.height = 0.2;
    obstacles.push_back(rect);
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);


    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Environment 1" << std::endl;
        std::cout << " (2) Environment 2" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

	    switch (choice)
	    {
		case 1:
		    makeEnvironment1(obstacles);
		    break;
		case 2:
		    makeEnvironment2(obstacles);
		    break;
		default:
		    std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
