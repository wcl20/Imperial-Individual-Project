#ifndef __SIMULATION_HPP__
#define __SIMULATION_HPP__

// Robot DART
#include <robot_dart/robot_dart_simu.hpp>
// Controller for hexapod
#include "include/simulation/policy.hpp"
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
// If we want graphics, include appropriate files
#ifdef GRAPHIC
  #include <robot_dart/gui/magnum/graphics.hpp>
#endif
// Descriptor
#include "include/simulation/descriptor.hpp"
// Obstacles
#include "include/simulation/obstacle.hpp"
#include "include/simulation/wall.hpp"

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

        // Set robot initial position
        robot->skeleton()->setPosition(3, 1);
        robot->skeleton()->setPosition(4, 1);
        robot->skeleton()->setPosition(5, 0.15);

        // Create simulation
        simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());

        // Add obstacles
        for (obstacle::Obstacle obstacle : obstacle::obstacles) {
                simu.add_robot(obstacle.robot());
        }

        // Add walls
        for (wall::Wall wall : wall::walls) {
                simu.add_robot(wall.robot());
        }

        // Add robot to world
        simu.add_robot(robot);

        // Create descriptors
        simu.add_descriptor(std::make_shared<descriptor::HexaDescriptor>(descriptor::HexaDescriptor(simu)));

        // Add floor to world
        simu.add_floor();

        // Graphics
#ifdef GRAPHIC
        auto graphics = std::make_shared<gui::magnum::Graphics<> >(simu.world(), 1024, 768);
        simu.set_graphics(graphics);
        graphics->enable_shadows(false);
        graphics->look_at({-2, -4, 5}, {2.5, 2.5, 1});
#endif

        return simu;
}

void simulate(robot_dart::RobotDARTSimu &simu, std::vector<double> ctrl, double x)
{
        using namespace robot_dart;
        auto robot = simu.robots().back();
        // Define number of steps
        int steps = 10;
        // Create controller
        double dt = 0.001;
        std::shared_ptr<control::HexaControl> controller;
        for (int i = 0; i < steps; ++i) {
                // Create controller for one time step
                std::vector<double> combined_ctrl(NS::Params::gen_size * 6, 0);
                // For each leg
                for (int leg = 0; leg < 6; ++leg) {
                        // Get current time step
                        double t = i % 2;
                        // Get current position of the leg
                        double current_position = ctrl[6 * t + leg];
                        // Get next position of the leg
                        double next_position = ctrl[6 * (1 - t) + leg];
                        // Get height of swing (Tripod motion)
                        double h = std::abs((i + leg) % 2 - x);
                        // Get controller for leg
                        Eigen::Vector3d descriptor (current_position, next_position, h);
                        std::vector<double> leg_ctrl = archive::get_leg_ctrl(descriptor, archive::L0_archives[leg]);
                        // Combine leg controller
                        for (int i = 0; i < NS::Params::gen_size; ++i) {
                                combined_ctrl[NS::Params::gen_size * leg + i] = leg_ctrl[i];
                        }
                }

                // Create new controller
                controller = std::make_shared<control::HexaControl>(dt, combined_ctrl);
                controller->set_h_params(std::vector<double>(1, dt));
                // Assign controller to robot
                robot->add_controller(controller);

                // Run simulation for 5 seconds
                simu.world()->setTime(0);
                simu.run(0.5);

                // Remove previous controller
                robot->remove_controller(controller);
        }
}
}

#endif
