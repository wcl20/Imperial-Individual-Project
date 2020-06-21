#ifndef __FITNESS_HPP__
#define __FITNESS_HPP__

#include <fstream>
#include "include/simulation/simulation.hpp"

FIT_QD(HexaFitness)
{
public:
        HexaFitness(){}

        template<typename Indiv>
        void eval(const Indiv& ind)
        {
                // Create a controller
                _ctrl.resize(ind.size());
                for (unsigned i = 0; i < ind.size(); ++i) {
                        _ctrl[i] = round(ind.data(i) * 1000.0) / 1000.0;
                }
                // Simulate robot using evolved controller
                std::pair<std::vector<double>, double> data = simulate(_ctrl);
                // Set behavior descriptor of this controller
                this->set_desc(data.first);
                // Set fitness of this controller
                this->_value = data.second;
        }

        // Run simulation using evolved controller
        std::pair<std::vector<double>, double> simulate(std::vector<double> ctrl, bool write=false)
        {
                using namespace robot_dart;

                // Create file
                std::ofstream file;

                // Create Simulation
                RobotDARTSimu simulation = simulation::initialize();
                auto robot = simulation.robots().back();

                // Store initial rotation
                Eigen::Vector3d initial_orientation = robot->skeleton()->getPositions().head(3);

                // Run Simulation
                auto descriptors = simulation::simulate(simulation, ctrl, write);

                // Get hexapod positions
                std::vector<Eigen::Vector3d> positions;
                std::static_pointer_cast<descriptor::HexaDescriptor>(simulation.descriptor(0))->get(positions);

                if (write) {
                        // Write used controllers into temp file
                        file.open("/git/sferes2/exp/layer1/temp.csv", std::ios_base::app);
                        for (int i = 0; i < descriptors.first.size(); i++) file << descriptors.first[i] << ",";
                        file << std::endl;
                        for (int i = 0; i < descriptors.second.size(); i++) file << descriptors.second[i] << ",";
                        file << std::endl;
                        file.close();
                }

                // Compute descriptor
                Eigen::Vector3d final_position = positions.back();
                std::vector<double> descriptor = compute_descriptor(final_position);

                // Compute fitness
                Eigen::Vector3d final_orientation = robot->skeleton()->getPositions().head(3);
                double value = compute_fitness(final_position, final_orientation, initial_orientation);

                // Reset robot
                robot.reset();

                // Return descriptor and fitness
                return { descriptor, value };
        }

        // Returns arrival angle of individual
        double angle()
        {
                return _angle;
        }

private:
        std::vector<double> _ctrl;
        double _angle;

        std::vector<double> compute_descriptor(Eigen::Vector3d &final_position)
        {
                std::vector<double> descriptor;
                // Store final position (Normalized)
                double x = final_position(0);
                descriptor.push_back((x + 1.2) / 2.4);
                double y = final_position(1);
                descriptor.push_back((y + 1.2) / 2.4);
                return descriptor;
        }

        double compute_fitness(Eigen::Vector3d &final_position, Eigen::Vector3d &final_orientation, Eigen::Vector3d &initial_orientation)
        {
                // Calculate arrival angle
                Eigen::Matrix3d initial_rotation =
                        dart::math::expMapRot({ initial_orientation(0), initial_orientation(1), initial_orientation(2) });
                Eigen::Matrix3d final_rotation =
                        dart::math::expMapRot({ final_orientation(0), final_orientation(1), final_orientation(2) });
                Eigen::Vector3d final_rotation_euler =
                        dart::math::matrixToEulerXYZ(initial_rotation.inverse() * final_rotation);
                double arrival_angle = std::round(final_rotation_euler(2) * 100.0) / 100.0;
                this->_angle = arrival_angle;

                // Compute desired arrival angle
                double x = final_position(0);
                double y = final_position(1);
                double B = std::sqrt((x / 2.0) * (x / 2.0) + (y / 2.0) * (y / 2.0));
                double alpha = std::atan2(y, x);
                double A = B / std::cos(alpha);
                double desired_angle = std::atan2(y, x - A);
                if (x < 0) desired_angle -= M_PI;
                while (desired_angle < -M_PI) desired_angle += 2 * M_PI;
                while (desired_angle > M_PI) desired_angle -= 2 * M_PI;

                // Compute fitness score
                double value = desired_angle - arrival_angle;
                while (value < -M_PI) value += 2 * M_PI;
                while (value > M_PI) value -= 2 * M_PI;
                value = -std::abs(value);

                return value;
        }

};

#endif
