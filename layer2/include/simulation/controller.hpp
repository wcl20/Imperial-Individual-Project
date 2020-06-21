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

          void set_parameters(const std::vector<double>& ctrl)
          {
                  assert(ctrl.size() == 18);

                  _legs0commands.clear();
                  _legs1commands.clear();
                  _legs2commands.clear();
                  _legs3commands.clear();
                  _legs4commands.clear();
                  _legs5commands.clear();

                  _controller = ctrl;

                  _legs0commands.push_back(_cosine_signal(ctrl[0], ctrl[1]));
                  _legs0commands.push_back(_sine_signal(ctrl[2]));

                  _legs1commands.push_back(_cosine_signal(ctrl[3], ctrl[4]));
                  _legs1commands.push_back(_sine_signal(ctrl[5]));

                  _legs2commands.push_back(_cosine_signal(ctrl[6], ctrl[7]));
                  _legs2commands.push_back(_sine_signal(ctrl[8]));

                  _legs3commands.push_back(_cosine_signal(ctrl[9], ctrl[10]));
                  _legs3commands.push_back(_sine_signal(ctrl[11]));

                  _legs4commands.push_back(_cosine_signal(ctrl[12], ctrl[13]));
                  _legs4commands.push_back(_sine_signal(ctrl[14]));

                  _legs5commands.push_back(_cosine_signal(ctrl[15], ctrl[16]));
                  _legs5commands.push_back(_sine_signal(ctrl[17]));
          }

          const std::vector<double>& parameters() const
          {
                  return _controller;
          }

          std::vector<double> pos(double t) const
          {
                  assert(_controller.size() == 18);

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
          std::vector<array_t> _legs0commands;
          std::vector<array_t> _legs1commands;
          std::vector<array_t> _legs2commands;
          std::vector<array_t> _legs3commands;
          std::vector<array_t> _legs4commands;
          std::vector<array_t> _legs5commands;
          std::vector<double> _controller;

          array_t _sine_signal(double amplitude) const
          {
                  array_t command;
                  for (unsigned i = 0; i < ARRAY_DIM; ++i) {
                          // Calculate time step
                          double t = 2 * M_PI * i / ARRAY_DIM;
                          // Follow Sine wave: a * sin(x - b)
                          command[i] = amplitude * sin(t);
                  }
                  return command;
          }

          array_t _cosine_signal(double start, double amplitude) const
          {
                  array_t command;
                  for (unsigned i = 0; i < ARRAY_DIM; ++i) {
                          double t = 2 * M_PI * i / ARRAY_DIM;
                          command[i] = _clip(amplitude * (cos(t) - 1) + start);
                  }
                  return command;
          }

          // Clip values between -1 and 1
          double _clip(double value) const
          {
                  return std::max(-1.0, std::min(value, 1.0));
          }
  };
} // namespace hexapod_controller

#endif
