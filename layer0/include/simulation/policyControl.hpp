#ifndef __POLICY_CONTROL_HPP__
#define __POLICY_CONTROL_HPP__

#include <robot_dart/control/robot_control.hpp>
#include <robot_dart/robot.hpp>
#include <robot_dart/utils.hpp>
// HexaPolicy
#include "include/simulation/policy.hpp"
// HexaController
#include "include/simulation/controller.hpp"

namespace robot_dart
{
  namespace control
  {
    template <typename Policy>
    class PolicyControl : public RobotControl
    {
    public:
            PolicyControl(double dt,
              const hexapod_controller::HexaController& controller, const std::vector<double>& ctrl) :
                    RobotControl(ctrl, false), _dt(dt)
            {
                    _policy.set_controller(controller);
            }

            void configure() override
            {
                    _policy.set_params(_ctrl);
                    if (_policy.output_size() == _control_dof)
                            _active = true;
                    auto robot = _robot.lock();
                    if (_full_dt)
                            _dt = robot->skeleton()->getTimeStep();
                    _first = true;
                    _i = 0;
                    _threshold = -robot->skeleton()->getTimeStep() * 0.5;
            }

            void set_h_params(const std::vector<double>& h_params)
            {
                    _policy.set_h_params(h_params);
            }

            std::vector<double> h_params() const
            {
                    return _policy.h_params();
            }

            Eigen::VectorXd calculate(double t) override
            {
                    ROBOT_DART_ASSERT(_control_dof == _policy.output_size(),
                      "PolicyControl: Policy output size is not the same as DOFs of the robot",
                      Eigen::VectorXd::Zero(_control_dof));
                    if (_first || _full_dt || (t - _prev_time - _dt) >= _threshold) {
                            _prev_commands = _policy.query(_robot.lock(), t);

                            _first = false;
                            _prev_time = t;
                            _i++;
                    }

                    return _prev_commands;
            }

            std::shared_ptr<RobotControl> clone() const override
            {
                    return std::make_shared<PolicyControl>(*this);
            }

    protected:
            int _i;
            Policy _policy;
            double _dt, _prev_time, _threshold;
            Eigen::VectorXd _prev_commands;
            bool _first = false;
            bool _full_dt = false;
    };

    typedef PolicyControl<HexaPolicy> HexaControl;
  }
}

#endif
