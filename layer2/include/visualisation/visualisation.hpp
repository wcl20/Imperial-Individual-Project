#ifndef __VISUALISATION_HPP__
#define __VISUALISATION_HPP__

#include "include/simulation/obstacle.hpp"
#include "include/visualisation/simple_svg.hpp"

namespace visualisation
{

// Create document
svg::Dimensions dimensions = svg::Dimensions(800, 800);
svg::Document document;
const double magnification = 800.0 / 5.0;

void initialize(std::string path)
{
        // Create document
        document = svg::Document(path, svg::Layout(dimensions, svg::Layout::TopLeft));
        // Create white background
        document << svg::Rectangle(svg::Point(0, 0), 800, 800, svg::Fill(svg::Color::White), svg::Stroke(2, svg::Color::Black));
}

void draw_obstacles(std::vector<obstacle::Obstacle> obstacles)
{
        for (obstacle::Obstacle obstacle : obstacles) {
                svg::Point point((obstacle.x - 0.05)* magnification, (5 - obstacle.y - 0.05) * magnification);
                svg::Rectangle rect(point, 0.05 * magnification, 0.05 * magnification, svg::Fill(svg::Color::Gray), svg::Stroke(1, svg::Color::Gray));
                document << rect;
        }
        document.save();
}

void draw_walls(std::vector<wall::Wall> walls)
{
        for (wall::Wall wall : walls) {
                svg::Point point((wall.x - 0.25) * magnification, (5 - wall.y - 0.25) * magnification);
                document << svg::Rectangle(point, 0.5 * magnification, 0.5 * magnification, svg::Fill(svg::Color::Gray), svg::Stroke(1, svg::Color::Gray));
        }
        document.save();
}

void draw_goal(Eigen::Vector2d &goal)
{
        svg::Point center(goal(0) * magnification, (5 - goal(1)) * magnification);
        double diameter = 0.1 * magnification;
        svg::Circle circle(center, diameter, svg::Fill(svg::Color::Green), svg::Stroke(1, svg::Color::Green));
        document << circle;
        document.save();
}

svg::Color interplolate(svg::Color color1, svg::Color color2, double alpha)
{
        int red = int((color2.red - color1.red) * alpha + color1.red);
        int green = int((color2.green - color1.green) * alpha + color1.green);
        int blue = int((color2.blue - color1.blue) * alpha + color1.blue);
        return svg::Color(red, green, blue);
}

void draw_robot(Eigen::Vector2d &position, double theta, double alpha)
{
        // Draw robot
        svg::Point center(position(0) * magnification, (5 - position(1)) * magnification);
        double diameter = 0.05 * 2.0 * magnification;
        svg::Color color = interplolate(svg::Color::Orange, svg::Color::Blue, alpha);
        svg::Circle circle(center, diameter, svg::Fill(color), svg::Stroke(1, color));
        document << circle;

        // Draw a line to indicate robot orientation
        Eigen::Vector2d direction(std::cos(theta), std::sin(theta));
        direction = position + direction.normalized() * 0.15;
        svg::Polyline line(svg::Stroke(3.0, svg::Color::Black));
        line << center;
        line << svg::Point(direction(0) * magnification, (5 - direction(1)) * magnification);
        document << line;

        document.save();
}

void draw_path(Eigen::Vector2d &start, Eigen::Vector2d &end, svg::Color color)
{
        svg::Polyline line(svg::Stroke(1.0, color));
        line << svg::Point(start(0) * magnification, (5 - start(1)) * magnification);
        line << svg::Point(end(0) * magnification, (5 - end(1)) * magnification);
        document << line;
        document.save();
}


}

#endif
