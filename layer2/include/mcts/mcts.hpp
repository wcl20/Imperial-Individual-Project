#ifndef __MCTS_HPP__
#define __MCTS_HPP__

#include <mcts/uct.hpp>
#include "include/qd/archive.hpp"
#include "include/simulation/obstacle.hpp"
#include "include/astar/astar.hpp"

namespace MCTS
{

Eigen::Vector2d goal;

/*************************************************************************************************************
 * MCTS Params
 **************************************************************************************************************/
struct Params {
        struct uct {
                MCTS_PARAM(double, c, 50.0);
        };

        struct spw {
                MCTS_PARAM(double, a, 0.5);
        };

        struct cont_outcome {
                MCTS_PARAM(double, b, 0.6);
        };

        struct mcts_node {
                MCTS_PARAM(size_t, parallel_roots, 4);
        };
};

/*************************************************************************************************************
 * MCTS Action
 **************************************************************************************************************/
struct HexaAction
{
        typedef typename QD::Layer1::container_t::indiv_t indiv_t;
        indiv_t ind;

        HexaAction() {
        }

        HexaAction(indiv_t ind) : ind(ind) {
        }

        bool operator==(const HexaAction& other) const
        {
                // Calculate position difference
                double dx = ind->fit().desc()[0] - other.ind->fit().desc()[0];
                double dy = ind->fit().desc()[1] - other.ind->fit().desc()[1];
                return (dx * dx + dy * dy) < 0.01;
        }
};

/*************************************************************************************************************
 * A* Search
 **************************************************************************************************************/
// Define cell size
const double cell_size = 0.5;
// Define map size
const int map_size = 10;
// Define robot radius
const double radius = 0.5;

bool collides(int x, int y, int x_new, int y_new)
{
        Eigen::Vector2d start(x * cell_size, y * cell_size);
        Eigen::Vector2d end(x_new * cell_size, y_new * cell_size);
        return wall::collides(start, end, radius);
}

/*************************************************************************************************************
 * MCTS State
 **************************************************************************************************************/
template <typename T>
inline T gaussian_rand(T m = 0.0, T v = 1.0)
{
        std::random_device rd;
        std::mt19937 gen(rd());

        std::normal_distribution<T> gaussian(m, v);

        return gaussian(gen);
}

struct HexaState
{

        typedef typename QD::Layer1::container_t::indiv_t indiv_t;
        double x, y, theta;

        HexaState() {
        }

        HexaState(double x, double y, double theta) : x(x), y(y), theta(theta) {
        }

        HexaAction next_action() const
        {
                // Find the next best position to go to
                Eigen::Vector2d next;
                // Calculate distance from goal
                double dx = x - goal(0);
                double dy = y - goal(1);
                double distance = dx * dx + dy * dy;
                // If current position is close to goal
                if (distance < 0.25) {
                        // Set next best position to goal
                        next = goal;
                } else {
                        // Otherwise, use A* search to find the next best position
                        astar::AStar<> astar;
                        astar::Node start(std::round(x / cell_size), std::round(y / cell_size), map_size, map_size);
                        astar::Node end(std::round(goal(0) / cell_size), std::round(goal(1) / cell_size), map_size, map_size);
                        // Searches for a path from start to end
                        auto path = astar.search(start, end, collides, map_size, map_size);
                        // If no path is found, return a random action
                        if (path.size() < 2) return random_action();
                        // Set next best position to one step in best path
                        next << path[1]._x * cell_size, path[1]._y * cell_size;
                }
                // Convert next position to behavior descriptor
                Eigen::Vector3d descriptor(next(0), next(1), 1.0);
                // Define transformation matrix (translation + z-rotation)
                Eigen::MatrixXd transformation(3, 3);
                transformation << std::cos(-theta), -std::sin(-theta), -x * std::cos(-theta) + y * std::sin(-theta),
                        std::sin(-theta), std::cos(-theta), -x * std::sin(-theta) - y * std::cos(-theta),
                        0, 0, 1;
                descriptor = transformation * descriptor;
                // Normalise descriptor
                descriptor(0) = (descriptor(0) + 1.2) / 2.4;
                descriptor(1) = (descriptor(1) + 1.2) / 2.4;

                // Add random noise
                descriptor(0) = gaussian_rand(descriptor(0), 0.001);
                descriptor(1) = gaussian_rand(descriptor(1), 0.001);

                // Find controller with behavior
                indiv_t ind = archive::get_ctrl({descriptor(0), descriptor(1)}, archive::L1_archive);
                return HexaAction(ind);
        }

        HexaAction random_action() const
        {
                indiv_t ind = archive::get_random_ctrl(archive::L1_archive);
                return HexaAction(ind);
        }

        HexaState move(const HexaAction& action) const
        {
                // Get behavior descriptor from action
                std::vector<double> desc = action.ind->fit().desc();

                // Denormalise descriptor
                desc[0] = desc[0] * 2.4 - 1.2;
                desc[1] = desc[1] * 2.4 - 1.2;

                // Find final position of robot using current theta
                Eigen::Vector3d final_position;
                final_position << desc[0], desc[1], 1.0;
                // Define transformation matrix (Z-rotation + translation)
                Eigen::MatrixXd transformation(3, 3);
                transformation << std::cos(theta), -std::sin(theta), x,
                        std::sin(theta), std::cos(theta), y,
                        0, 0, 1;
                final_position = transformation * final_position;
                // Compute desired angle
                double B = std::sqrt((desc[0] / 2.0) * (desc[0] / 2.0) + (desc[1] / 2.0) * (desc[1] / 2.0));
                double alpha = std::atan2(desc[1], desc[0]);
                double A = B / std::cos(alpha);
                double desired_angle = std::atan2(desc[1], desc[0] - A);
                if (desc[0] < 0) desired_angle -= M_PI;
                while (desired_angle < -M_PI) desired_angle += 2 * M_PI;
                while (desired_angle > M_PI) desired_angle -= 2 * M_PI;
                // Compute new state with random noise
                double x_new = gaussian_rand(final_position(0), 0.01);
                double y_new = gaussian_rand(final_position(1), 0.01);
                double theta_new = theta + gaussian_rand(desired_angle, 0.1);
                while (theta_new < -M_PI) theta_new += 2 * M_PI;
                while (theta_new > M_PI) theta_new -= 2 * M_PI;

                return HexaState(x_new, y_new, theta_new);
        }

        bool terminal() const
        {
                // Check robot out of bounds
                if (x < 0 || x >= map_size * cell_size ||
                    y < 0 || y >= map_size * cell_size) {
                        return true;
                }
                // Check robot collision
                Eigen::Vector2d point;
                point << x, y;
                if (wall::collides(point, radius)) return true;
                // Check robot reached goal
                double dx = x - goal(0);
                double dy = y - goal(1);
                return (dx * dx + dy * dy) < 0.01;
        }

        bool operator==(const HexaState& other) const
        {
                // Calculate position difference
                double dx = x - other.x;
                double dy = y - other.y;
                // Calculate angle difference
                double dtheta = theta - other.theta;
                while (dtheta < -M_PI) dtheta += 2 * M_PI;
                while (dtheta > M_PI) dtheta -= 2 * M_PI;
                return (dx * dx + dy * dy) < 0.01 && std::abs(dtheta) < 0.3;
        }
};

/*************************************************************************************************************
 * MCTS Reward Function
 **************************************************************************************************************/
struct RewardFunction {
        template <typename State>
        double operator()(std::shared_ptr<State> from_state, HexaAction action, std::shared_ptr<State> to_state)
        {
                // Check robot out of bounds
                if (to_state->x < 0 || to_state->x >= map_size * cell_size ||
                    to_state->y < 0 || to_state->y >= map_size * cell_size)
                        return -1.0;
                // Check robot collision
                Eigen::Vector2d point;
                point << to_state->x, to_state->y;
                if (wall::collides(point, radius))
                        return -1.0;
                // Check robot reach goal
                double dx = to_state->x - goal(0);
                double dy = to_state->y - goal(1);
                return (dx * dx + dy * dy) < 0.01 ? 1000.0 : 0.0;
        }
};

/*************************************************************************************************************
 * MCTS Algorithm
 **************************************************************************************************************/
std::vector<double> best_ctrl(Eigen::Vector2d& position, double theta)
{
        mcts::par::init();

        RewardFunction world;
        HexaState init(position(0), position(1), theta);
        auto tree = std::make_shared<mcts::MCTSNode<Params,
                                                    HexaState,
                                                    mcts::SimpleStateInit<HexaState>,
                                                    mcts::SimpleValueInit,
                                                    mcts::UCTValue<Params>,
                                                    mcts::UniformRandomPolicy<HexaState, HexaAction>,
                                                    HexaAction, mcts::SPWSelectPolicy<Params>,
                                                    mcts::ContinuousOutcomeSelect<Params> > >(init, 2000);
        int iterations = 1000;
        tree->compute(world, iterations);
        auto best = tree->best_action();

        // Get controller
        std::vector<float> gen = best->action().ind->gen().data();
        std::vector<double> ctrl (gen.begin(), gen.end());
        return ctrl;
}
}

#endif
