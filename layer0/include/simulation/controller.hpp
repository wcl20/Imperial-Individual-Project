#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

// For M_PI constant
#define _USE_MATH_DEFINES

#include <array>
#include <cassert>
#include <cmath>
#include <vector>

#define ARRAY_DIM 100

namespace hexapod_controller {

  class HexaController
  {
  public:
          typedef std::array<double, ARRAY_DIM> array_t;

          HexaController() {}

          HexaController(int leg, bool fixed) : _leg(leg), _fixed(fixed) {}

          /**
              All parameters should have a value between 0 and 1.
           **/
          void set_parameters(const std::vector<double>& ctrl)
          {
                  assert(ctrl.size() == 3);

                  _controller = ctrl;

                  // Set random movement for all the legs to prevent segmentation fault
                  array_t random_signal = _sine_signal(0.1, 0.1);
                  _legs0commands.clear();
                  _legs0commands.push_back(random_signal);
                  _legs0commands.push_back(random_signal);
                  _legs1commands.clear();
                  _legs1commands.push_back(random_signal);
                  _legs1commands.push_back(random_signal);
                  _legs2commands.clear();
                  _legs2commands.push_back(random_signal);
                  _legs2commands.push_back(random_signal);
                  _legs3commands.clear();
                  _legs3commands.push_back(random_signal);
                  _legs3commands.push_back(random_signal);
                  _legs4commands.clear();
                  _legs4commands.push_back(random_signal);
                  _legs4commands.push_back(random_signal);
                  _legs5commands.clear();
                  _legs5commands.push_back(random_signal);
                  _legs5commands.push_back(random_signal);

                  // Set controller for specified leg
                  switch (_leg) {
                  case 0:
                          _legs0commands.clear();
                          if (_fixed) {
                                  _legs0commands.push_back(_fixed_signal(ctrl[0]));
                                  _legs0commands.push_back(_sine_signal(0.0, 0.0));
                          } else {
                                  _legs0commands.push_back(_cosine_signal(ctrl[0], ctrl[1]));
                                  _legs0commands.push_back(_sine_signal(ctrl[2], 0.0));
                          }
                          break;
                  case 1:
                          _legs1commands.clear();
                          if (_fixed) {
                                  _legs1commands.push_back(_fixed_signal(ctrl[0]));
                                  _legs1commands.push_back(_sine_signal(0.0, 0.0));
                          } else {
                                  _legs1commands.push_back(_cosine_signal(ctrl[0], ctrl[1]));
                                  _legs1commands.push_back(_sine_signal(ctrl[2], 0.0));
                          }
                          break;
                  case 2:
                          _legs2commands.clear();
                          if (_fixed) {
                                  _legs2commands.push_back(_fixed_signal(ctrl[0]));
                                  _legs2commands.push_back(_sine_signal(0.0, 0.0));
                          } else {
                                  _legs2commands.push_back(_cosine_signal(ctrl[0], ctrl[1]));
                                  _legs2commands.push_back(_sine_signal(ctrl[2], 0.0));
                          }
                          break;
                  case 3:
                          _legs3commands.clear();
                          if (_fixed) {
                                  _legs3commands.push_back(_fixed_signal(ctrl[0]));
                                  _legs3commands.push_back(_sine_signal(0.0, 0.0));
                          } else {
                                  _legs3commands.push_back(_cosine_signal(ctrl[0], ctrl[1]));
                                  _legs3commands.push_back(_sine_signal(ctrl[2], 0.0));
                          }
                          break;
                  case 4:
                          _legs4commands.clear();
                          if (_fixed) {
                                  _legs4commands.push_back(_fixed_signal(ctrl[0]));
                                  _legs4commands.push_back(_sine_signal(0.0, 0.0));
                          } else {
                                  _legs4commands.push_back(_cosine_signal(ctrl[0], ctrl[1]));
                                  _legs4commands.push_back(_sine_signal(ctrl[2], 0.0));
                          }
                          break;
                  case 5:
                          _legs5commands.clear();
                          if (_fixed) {
                                  _legs5commands.push_back(_fixed_signal(ctrl[0]));
                                  _legs5commands.push_back(_sine_signal(0.0, 0.0));
                          } else {
                                  _legs5commands.push_back(_cosine_signal(ctrl[0], ctrl[1]));
                                  _legs5commands.push_back(_sine_signal(ctrl[2], 0.0));
                          }
                          break;
                  }
          }

          const std::vector<double>& parameters() const
          {
                  return _controller;
          }

          std::vector<double> pos(double t) const
          {
                  assert(_controller.size() == 3);

                  std::vector<double> angles;
                  for (int leg = 0; leg < 6; leg++) {

                          switch (leg) {
                          case 0:
                                  angles.push_back(M_PI_4 / 2 * _legs0commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(M_PI_4 * _legs0commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(-M_PI_4 * _legs0commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  break;

                          case 1:
                                  angles.push_back(M_PI_4 / 2 * _legs1commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(M_PI_4 * _legs1commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(-M_PI_4 * _legs1commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  break;

                          case 2:
                                  angles.push_back(M_PI_4 / 2 * _legs2commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(M_PI_4 * _legs2commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(-M_PI_4 * _legs2commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  break;

                          case 3:
                                  angles.push_back(M_PI_4 / 2 * _legs3commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(M_PI_4 * _legs3commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(-M_PI_4 * _legs3commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  break;

                          case 4:
                                  angles.push_back(M_PI_4 / 2 * _legs4commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(M_PI_4 * _legs4commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(-M_PI_4 * _legs4commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  break;

                          case 5:
                                  angles.push_back(M_PI_4 / 2 * _legs5commands[0][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(M_PI_4 * _legs5commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  angles.push_back(-M_PI_4 * _legs5commands[1][(int(std::floor(t * ARRAY_DIM))) % ARRAY_DIM]);
                                  break;
                          }
                  }
                  return angles;

          }

  protected:
          // Defines which leg to control
          int _leg;
          // If true, fix leg to _start
          bool _fixed;
          std::vector<array_t> _legs0commands;
          std::vector<array_t> _legs1commands;
          std::vector<array_t> _legs2commands;
          std::vector<array_t> _legs3commands;
          std::vector<array_t> _legs4commands;
          std::vector<array_t> _legs5commands;
          std::vector<double> _controller;

          array_t _sine_signal(double amplitude, double phase) const
          {
                  // Calculate phase shift
                  double shift = M_PI * phase;
                  array_t command;
                  for (unsigned i = 0; i < ARRAY_DIM; ++i) {
                          // Calculate time step
                          double t = 2 * M_PI * i / ARRAY_DIM;
                          // Follow Sine wave: a * sin(x - b)
                          command[i] = amplitude * sin(t - shift);
                  }
                  return command;
          }

          array_t _cosine_signal(double start, double amplitude) const
          {
                  array_t command;
                  for (unsigned i = 0; i < ARRAY_DIM; ++i) {
                          double t = 2 * M_PI * i / ARRAY_DIM;
                          command[i] = amplitude * (cos(t) - 1) + start;
                  }
                  return command;
          }

          array_t _fixed_signal(double position) const
          {
                  array_t command;
                  for (unsigned i = 0; i < ARRAY_DIM; ++i) {
                          command[i] = position;
                  }
                  return command;
          }
  };
}

#endif
