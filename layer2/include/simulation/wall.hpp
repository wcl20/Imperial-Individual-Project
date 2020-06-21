#ifndef __WALL_HPP__
#define __WALL_HPP__

// Robot DART
#include <robot_dart/robot_dart_simu.hpp>
#include <limbo/tools.hpp>

namespace wall
{
/*************************************************************************************************************
 * Walls
 **************************************************************************************************************/
struct Wall
{
        double x, y, r = 0.3;
        Wall(double x, double y) : x(x), y(y) {
        }

        std::shared_ptr<robot_dart::Robot> robot() {
                Eigen::Vector6d position;
                position << 0, 0, 0, x, y, 0;
                Eigen::Vector3d size(0.5, 0.5, 0.5);
                std::string name = "wall(" + std::to_string(x) + ", " + std::to_string(y) + ")";
                return robot_dart::Robot::create_box(size, position, "fixed", 1, dart::Color::Red(1.0), name);
        }
};

// Obstacles
std::vector<Wall> walls = {};

void initialize(std::string &map)
{
        using namespace limbo;
        // Start at top left corner of the map
        double row = 4.75, col = 0.25;
        // Iterate map string
        for (size_t i = 0; i < map.size(); i++) {
                char symbol = map[i];
                // Handle new line character
                if (symbol == '\n') {
                        // Move to next row
                        row -= 0.5;
                        col = -0.25;
                }
                // Handle * character
                if (symbol == '*') {
                        // Create wall
                        walls.push_back(Wall(col, row));
                }
                // Move to next column
                col += 0.5;
        }
}
/*************************************************************************************************************
 * Collisions
 **************************************************************************************************************/

// Checks if point collide with obstacles
bool collides(const Eigen::Vector2d &point, double radius)
{
        for (Wall wall : walls) {
                double dx = point(0) - wall.x;
                double dy = point(1) - wall.y;
                if (std::sqrt(dx * dx + dy * dy) <= wall.r + radius) return true;
        }
        return false;
}

// Checks if a line segment collides with obstacles
bool collides(const Eigen::Vector2d& start, const Eigen::Vector2d& end)
{
        double dx = end(0) - start(0);
        double dy = end(1) - start(1);
        for (Wall wall : walls) {
                // Assume obstacle is located at origin (Translate start position)
                double x = start(0) - wall.x;
                double y = start(1) - wall.y;
                // Coefficients for qudratic formula
                double a = std::pow(dx, 2) + std::pow(dy, 2);
                double b = 2 * (x * dx + y * dy);
                double c = std::pow(x, 2) + std::pow(y, 2) - std::pow(wall.r, 2);
                // Compute descriminant of quadratic equation
                double discriminant = std::pow(b, 2) - 4 * a * c;
                // If discriminant is larger than 0
                if (discriminant > 0) {
                        // Compute to values of t
                        double t1 = (-b - std::sqrt(discriminant)) / (2.0 * a);
                        double t2 = (-b + std::sqrt(discriminant)) / (2.0 * a);
                        // Collision occurs if t is between 0 and 1
                        if (t1 > 0 && t1 < 1) return true;
                        if (t2 > 0 && t2 < 1) return true;
                }
        }
        return false;
}

bool collides(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double radius)
{
        // Check for point collision
        if (collides(start, radius) || collides(end, radius)) return true;

        // Direction of robot
        Eigen::Vector2d direction = end - start;
        // Vector perpendicular to direction
        Eigen::Vector2d perpendicular = Eigen::Vector2d(-direction(1), direction(0));
        perpendicular.normalize();
        Eigen::Vector2d A = start + perpendicular * radius;
        Eigen::Vector2d B = end + perpendicular * radius;
        Eigen::Vector2d C = end - perpendicular * radius;
        Eigen::Vector2d D = start - perpendicular * radius;
        // Check for path collision
        return collides(A, B) || collides(C, D);
}
}

#endif
