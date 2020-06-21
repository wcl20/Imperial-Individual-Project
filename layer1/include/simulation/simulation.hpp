#ifndef __SIMULATION_HPP__
#define __SIMULATION_HPP__

// Robot DART
#include <robot_dart/robot_dart_simu.hpp>
#include "include/simulation/policy.hpp"
#include "include/simulation/descriptor.hpp"
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
// If we want graphics, include appropriate files
#ifdef GRAPHIC
  #include <robot_dart/gui/magnum/graphics.hpp>
#endif
#include "include/ns/ns.hpp"

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

        // Descriptor tracks path of hexapod
        simu.add_descriptor(std::make_shared<descriptor::HexaDescriptor>(descriptor::HexaDescriptor(simu)));

        // Add floor to world
        simu.add_floor();

        // Graphics
#ifdef GRAPHIC
        auto graphics = std::make_shared<gui::magnum::Graphics<> >(simu.world());
        simu.set_graphics(graphics);
#endif

        return simu;
}

std::pair<std::vector<double>, std::vector<double> > simulate(robot_dart::RobotDARTSimu &simu, std::vector<double> ctrl, bool write=false)
{
        using namespace robot_dart;
        auto robot = simu.robots().back();
        // Store the swing and stance controller used
        std::pair<std::vector<double>, std::vector<double> > descriptors;
        // Define number of steps
        int steps = write ? 2 : 10;
        // Create controller
        double dt = 0.001;
        std::shared_ptr<control::HexaControl> controller;
        for (int i = 0; i < steps; ++i) {
                // Create controller for one time step
                std::vector<double> combined_ctrl(NS::Params::gen_size * 6, 0);
                std::vector<double> combined_desc(NS::Params::qd::behav_dim * 6, 0);
                // For each leg
                for (int leg = 0; leg < 6; ++leg) {
                        // Get current time step
                        double t = i % 2;
                        // Get current position of the leg
                        double current_position = ctrl[6 * t + leg];
                        // Get next position of the leg
                        double next_position = ctrl[6 * (1 - t) + leg];
                        // Get height of swing (Tripod motion)
                        double h = (i + leg) % 2;
                        // Get controller for leg
                        Eigen::Vector3d descriptor (current_position, next_position, h);
                        // Returns descriptor of closest controller
                        std::vector<double> leg_ctrl, desc;
                        std::tie(leg_ctrl, desc) = archive::get_leg_ctrl(descriptor, archive::archives[leg]);
                        // Combine leg controller
                        for (int i = 0; i < NS::Params::gen_size; ++i) {
                                combined_ctrl[NS::Params::gen_size * leg + i] = leg_ctrl[i];
                        }
                        // Combine leg descriptors
                        if (write) {
                                for (int i = 0; i < NS::Params::qd::behav_dim; ++i) {
                                        combined_desc[NS::Params::qd::behav_dim * leg + i] = desc[i];
                                }
                        }
                }

                // Store swing and stance actions
                if (write) {
                        if (i == 0) descriptors.first = combined_desc;
                        if (i == 1) descriptors.second = combined_desc;
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

        return descriptors;
}
}

#endif
