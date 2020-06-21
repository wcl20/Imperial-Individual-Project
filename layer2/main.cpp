//| This file is a part of the sferes2 framework.
//| Copyright 2016, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#include <iostream>
#include <fstream>
// Robot DART
#include <robot_dart/robot_dart_simu.hpp>
#include "include/ns/archive.hpp"
#include "include/qd/archive.hpp"

// Define Global parameters
namespace global
{
std::shared_ptr<robot_dart::Robot> hexapod;
}

#include "include/bo/bo.hpp"
#include "include/simulation/simulation.hpp"
#include "include/visualisation/visualisation.hpp"
#include "include/mcts/mcts.hpp"

int main(int argc, char **argv)
{
        // Restrict number of CPUs to 48
        tbb::task_scheduler_init init(48);

        // Initialize Robot
        using namespace robot_dart;
        global::hexapod = std::make_shared<Robot>("/git/sferes2/exp/layer2/resources/hexapod_v2.urdf");
        global::hexapod->set_position_enforced(true);
        global::hexapod->set_actuator_types(dart::dynamics::Joint::SERVO);
        global::hexapod->skeleton()->enableSelfCollisionCheck();

        // Load archive for layer 0
        for (int leg = 0; leg < 6; ++leg) {
                archive::load_archive("/git/sferes2/exp/layer2/data/layer0/leg" + std::to_string(leg) + ".dat", archive::L0_archives[leg]);
        }
        // // Load archive for layer 1
        archive::load_grid("/git/sferes2/exp/layer2/data/layer1/archive.dat", archive::L1_archive);

        // Create map
        std::string map = "          \n"
                          "          \n"
                          "          \n"
                          "          \n"
                          "          \n"
                          "          \n"
                          "          \n"
                          "          \n"
                          "          \n"
                          "          \n";
        wall::initialize(map);

        // Create obstacles
        obstacle::initialize();

        // Create visualisation
        visualisation::initialize("/git/sferes2/exp/layer2/results/figures/temp.svg");
        visualisation::draw_obstacles(obstacle::obstacles);
        visualisation::draw_walls(wall::walls);

        // Define goals
        std::vector<std::vector<double> > goals = { {4, 4} };

        // Create file
        std::ofstream file;


        while (!goals.empty()) {

                // Create goal at waypoint
                MCTS::goal << goals.back()[0], goals.back()[1];
                goals.pop_back();

                // Simulation
                robot_dart::RobotDARTSimu simulation = simulation::initialize();
                auto robot = simulation.robots().back();

                // Store initial rotation
                Eigen::Vector3d initial_orientation = robot->skeleton()->getPositions().head(3);
                Eigen::Matrix3d initial_rotation = dart::math::expMapRot({ initial_orientation(0), initial_orientation(1), initial_orientation(2) });

                // Get current position of the hexapod
                Eigen::Vector2d position = robot->skeleton()->getPositions().head(6).tail(3).head(2);

                // Get current orientation of hexapod
                Eigen::Vector3d orientation = robot->skeleton()->getPositions().head(3);
                Eigen::Matrix3d rotation = dart::math::expMapRot({ orientation(0), orientation(1), orientation(2) });
                // Convert to euler angles
                Eigen::Vector3d euler = dart::math::matrixToEulerXYZ(initial_rotation.inverse() * rotation);
                // Get theta (yaw) from euler angles
                double theta = std::round(euler(2) * 100.0) / 100.0;

                // Use A* Search to compute best path
                astar::AStar<> astar;
                astar::Node start(std::round(position(0) / MCTS::cell_size), std::round(position(1) / MCTS::cell_size), MCTS::map_size, MCTS::map_size);
                astar::Node end(std::round(MCTS::goal(0) / MCTS::cell_size), std::round(MCTS::goal(1) / MCTS::cell_size), MCTS::map_size, MCTS::map_size);
                auto path = astar.search(start, end, MCTS::collides, MCTS::map_size, MCTS::map_size);

                // visualisation
                visualisation::draw_goal(MCTS::goal);
                visualisation::draw_robot(position, theta, 1.0);
                // Draw Best path
                for (int i = 0; i < path.size() - 1; i++) {
                        Eigen::Vector2d start(path[i]._x * MCTS::cell_size, path[i]._y * MCTS::cell_size);
                        Eigen::Vector2d end(path[i + 1]._x * MCTS::cell_size, path[i + 1]._y * MCTS::cell_size);
                        visualisation::draw_path(start, end, svg::Color::Red);
                }


                // Get distance to goal
                auto t0 = std::chrono::steady_clock::now();
                double distance = (position - MCTS::goal).norm();

                // Save distance
                file.open("/git/sferes2/exp/layer2/results/distances/temp.csv");
                file << time << "," << distance << "," << 0.4 << std::endl;

                while (distance > 0.2) {

                        // Find best controller using MCTS
                        std::vector<double> ctrl = MCTS::best_ctrl(position, theta);

                        // Find best swing height using bayesian optimizer
                        Eigen::VectorXd x = BO::optimizer.sample();
                        double height = x(0) * 0.4;

                        // Run controller
                        simulation::simulate(simulation, ctrl, height);

                        // Find new position of robot
                        Eigen::Vector2d new_position = robot->skeleton()->getPositions().head(6).tail(3).head(2);

                        // Find new theta of robot
                        orientation = robot->skeleton()->getPositions().head(3);
                        rotation = dart::math::expMapRot({ orientation(0), orientation(1), orientation(2) });
                        euler = dart::math::matrixToEulerXYZ(initial_rotation.inverse() * rotation);
                        double new_theta = std::round(euler(2) * 100.0) / 100.0;

                        // Update Gaussian Process model
                        int collisions = std::static_pointer_cast<descriptor::HexaDescriptor>(simulation.descriptor(0))->get();
                        double oberservation = x(0) - collisions / 10;
                        BO::optimizer.update(x, limbo::tools::make_vector(oberservation));

                        // visualisation
                        visualisation::draw_path(position, new_position, svg::Color::Black);
                        visualisation::draw_robot(new_position, new_theta, x(0));

                        // Update
                        position = new_position;
                        theta = new_theta;
                        distance = (position - MCTS::goal).norm();

                        // Get current time
                        auto time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - t0).count();
                        file << time << "," << distance << "," << height << std::endl;
                };

                // Reset robot
                robot.reset();
        }

        file.close();
        global::hexapod.reset();
        return 0;
}
