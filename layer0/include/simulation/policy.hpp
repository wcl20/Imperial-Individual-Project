#ifndef __POLICY_HPP__
#define __POLICY_HPP__

#include "include/simulation/controller.hpp"

namespace robot_dart
{
  namespace control
  {
    struct HexaPolicy
    {
    public:
            // Function use by PolicyControl. Set parameters of controller
            void set_params(const std::vector<double>& ctrl)
            {
                    _controller.set_parameters(ctrl);
            }

            // Function use by PolicyControl. Returns size of controller.
            size_t output_size() const {
                    return 18;
            }

            // Function use by PolicyControl. Returns velocity of each motor.
            Eigen::VectorXd query(const std::shared_ptr<robot_dart::Robot>& robot, double t)
            {
                    auto angles = _controller.pos(t);

                    Eigen::VectorXd target_positions = Eigen::VectorXd::Zero(18);
                    for (size_t i = 0; i < angles.size(); i++)
                            target_positions(i) = ((i % 3 == 1) ? 1.0 : -1.0) * angles[i];

                    Eigen::VectorXd q = robot->skeleton()->getPositions();
                    Eigen::VectorXd q_err = target_positions - q;

                    double gain = 1.0 / (dart::math::constants<double>::pi() * _dt);
                    Eigen::VectorXd vel = q_err * gain;

                    return vel.tail(18);
            }

            // Function use by PolicyControl. Set Hyperparameters
            void set_h_params(const std::vector<double>& h_params)
            {
                    _dt = h_params[0];
            }

            // Function us by PolicyControl. Returns Hyperparameters.
            std::vector<double> h_params() const
            {
                    return std::vector<double>(1, _dt);
            }

            void set_controller(const hexapod_controller::HexaController controller)
            {
                    _controller = controller;
            }

    protected:
            hexapod_controller::HexaController _controller;
            double _dt;
    };
  }
}

#endif
