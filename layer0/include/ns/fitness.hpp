#ifndef __FITNESS_HPP__
#define __FITNESS_HPP__

#include <fstream>
#include "include/simulation/simulation.hpp"

FIT_QD(HexaFitness)
{
public:
        HexaFitness() {}

        template<typename Indiv>
        void eval(const Indiv& ind)
        {
                // Create a controller
                _ctrl.resize(ind.size());
                _ctrl[0] = round(ind.data(0) * 1000.0) / 1000.0; // Controls starting position
                _ctrl[1] = round(ind.data(1) * 1000.0) / 1000.0; // Controls lateral movement
                _ctrl[2] = round(ind.data(2) * 1000.0) / 1000.0; // Controls vertical movement
                // Kill solution if trajectory is out of bounds
                if (abs(_ctrl[0] - 2 * _ctrl[1]) > 1) {
                        this->_dead = true;
                        return;
                }
                // Simulate robot using evolved controller
                std::pair<std::vector<double>, double> data = simulate(Params::leg, _ctrl);
                // Set behavior descriptor of this controller
                this->set_desc(data.first);
                // Set fitness of this controller
                this->_value = data.second;
        }

        // Run simulation using evolved controller
        std::pair<std::vector<double>, double> simulate(int leg, std::vector<double> ctrl, bool write=false)
        {
                using namespace robot_dart;

                // Create file
                std::ofstream file;
                if (write) file.open("/git/sferes2/exp/layer0/temp.csv", std::ios_base::app);

                // Create Simulation
                RobotDARTSimu simulation = simulation::initialize();
                auto robot = simulation.robots().back();

                // Record origin position
                static Eigen::Vector3d origin_position = robot->body_pos("leg_" + std::to_string(leg) + "_3");
                if (write) file << origin_position(0) << "," << origin_position(1) << "," << origin_position(2) << std::endl;

                // Run Simulation
                simulation::simulate(simulation, leg, ctrl);

                // Get leg positions
                std::vector<Eigen::Vector3d> positions;
                std::static_pointer_cast<descriptor::HexaDescriptor>(simulation.descriptor(0))->get(positions);

                // Write positions into temp file
                if (write) {
                        for(std::vector<Eigen::Vector3d>::iterator it = positions.begin(); it != positions.end(); ++it)
                        {
                                Eigen::Vector3d position = *it;
                                file << position(0) << "," << position(1) << "," << position(2) << std::endl;
                        }
                        file.close();
                }

                // Compute descriptor
                std::vector<double> descriptor = compute_descriptor(origin_position, positions);

                // Compute fitness
                Eigen::Vector3d final_position = positions.back();
                double value = compute_fitness(final_position);

                // Reset robot
                robot.reset();

                // Return descriptor and fitness
                return { descriptor, value };
        }

private:
        std::vector<double> _ctrl;

        std::vector<double> compute_descriptor(Eigen::Vector3d &origin_position, std::vector<Eigen::Vector3d> positions)
        {
                std::vector<double> descriptor;
                // Get distance of initial_position from origin position
                Eigen::Vector2d initial_position = positions.front().head(2);
                double initial_distance = (initial_position - origin_position.head(2)).norm();
                if (initial_position(0) < origin_position(0)) {
                        initial_distance *= -1;
                }
                // Get distance of final_position from origin position
                Eigen::Vector2d final_position = positions.back().head(2);
                double final_distance = (final_position - origin_position.head(2)).norm();
                if (final_position(0) < origin_position(0)) {
                        final_distance *= -1;
                }
                // Calculate height of swing arc
                double h = 0.0;
                for(std::vector<Eigen::Vector3d>::iterator it = positions.begin(); it != positions.end(); ++it)
                {
                        // Get z position
                        Eigen::Vector3d position = *it;
                        double z = position(2);
                        // Calculate difference from start position (z = 0)
                        if(abs(z) > abs(h)) h = z;
                }
                // Store Start position (Normalized)
                descriptor.push_back((initial_distance + 0.06) / 0.12);
                // Store final position (Normalized)
                descriptor.push_back((final_distance + 0.06) / 0.12);
                // Store height of swing arc (Normalized)
                descriptor.push_back((h + 0.06) / 0.12);
                return descriptor;
        }

        double compute_fitness(Eigen::Vector3d &final_position)
        {
                // No fitness, equivalent to novelty search
                return 0;
        }
};

#endif
