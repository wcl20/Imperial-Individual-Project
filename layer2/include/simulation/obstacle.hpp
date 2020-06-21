#ifndef __OBSTACLE_HPP__
#define __OBSTACLE_HPP__

// Robot DART
#include <robot_dart/robot_dart_simu.hpp>
#include <limbo/tools.hpp>

namespace obstacle
{
/*************************************************************************************************************
 * Obstacles
 **************************************************************************************************************/
struct Obstacle
{
        double x, y, h;
        Obstacle(double x, double y, double h) : x(x), y(y), h(h) {
        }

        std::shared_ptr<robot_dart::Robot> robot() {
                Eigen::Vector6d position;
                position << 0, 0, 0, x, y, 0;
                Eigen::Vector3d size(0.05, 0.05, h);
                std::string name = "obstacle(" + std::to_string(x) + ", " + std::to_string(y) + ")";
                return robot_dart::Robot::create_box(size, position, "fixed", 1, dart::Color::Red(1.0), name);
        }
};

// Obstacles
std::vector<Obstacle> obstacles = {};

void initialize()
{
        using namespace limbo;
        // Create random number generator
        static tools::rgen_double_t x_generator(0, 5);
        static tools::rgen_double_t y_generator(3, 5);
        static tools::rgen_double_t height_generator(0.03, 0.1);
        for(int i = 0; i < 200; i++) {
                double x = x_generator.rand();
                double y = x_generator.rand();
                double h = height_generator.rand();
                obstacles.push_back(Obstacle(x, y, h));
        }
}

}

#endif
