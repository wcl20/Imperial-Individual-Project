#ifndef __SIMULATION_HPP__
#define __SIMULATION_HPP__

// Robot DART
#include <robot_dart/robot_dart_simu.hpp>
#include "include/simulation/policyControl.hpp"
#include "include/simulation/descriptor.hpp"
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
// If we want graphics, include appropriate files
#ifdef GRAPHIC
  #include <robot_dart/gui/magnum/graphics.hpp>
#endif

// Define Global hexapod
namespace global
{
        std::shared_ptr<robot_dart::Robot> hexapod;
}

namespace simulation
{
/*************************************************************************************************************
* Simulation
*************************************************************************************************************/
robot_dart::RobotDARTSimu initialize()
{
        using namespace robot_dart;

        // Simulation
        RobotDARTSimu simu(0.001);

        // Make a clone of the hexapod
        auto robot = global::hexapod->clone();
        robot->skeleton()->setPosition(5, 0.15);

        // Create simulation
        simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());

        // Add robot to world
        simu.add_robot(robot);

        // Graphics
#ifdef GRAPHIC
        auto graphics = std::make_shared<gui::magnum::Graphics<> >(simu.world());
        simu.set_graphics(graphics);
#endif

        return simu;
}

void simulate(robot_dart::RobotDARTSimu &simu, int leg, std::vector<double> ctrl)
{
        using namespace robot_dart;
        auto robot = simu.robots().back();

        // Set controller moves leg to initial position
        double dt = 0.001;
        hexapod_controller::HexaController setController(leg, true);
        auto controller = std::make_shared<control::HexaControl>(dt, setController, ctrl);
        controller->set_h_params(std::vector<double>(1, dt));
        robot->add_controller(controller);
        simu.run(2);

        // Descriptor tracks position of leg
        simu.add_descriptor(std::make_shared<descriptor::HexaDescriptor>(descriptor::HexaDescriptor(simu, leg)));

        // Move controller performs actual action
        robot->remove_controller(controller);
        hexapod_controller::HexaController moveController(leg, false);
        controller = std::make_shared<control::HexaControl>(dt, moveController, ctrl);
        controller->set_h_params(std::vector<double>(1, dt));
        robot->add_controller(controller);

        // Run simulation for 0.5 seconds
        simu.world()->setTime(0);
        simu.run(0.5);
}
}

#endif
