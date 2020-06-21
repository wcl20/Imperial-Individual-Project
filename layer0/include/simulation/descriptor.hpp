#ifndef __DESCRIPTOR_HPP__
#define __DESCRIPTOR_HPP__


namespace robot_dart
{
  namespace descriptor
  {
    struct HexaDescriptor : BaseDescriptor
    {
    public:
            // Constructor
            HexaDescriptor(RobotDARTSimu &simu, int leg, size_t desc_dump = 1)
                    : BaseDescriptor(simu, desc_dump), _leg(leg) {
            }

            // This function is called every desc_dump steps of the simulation
            void operator()()
            {
                    // Get current position of the leg
                    Eigen::Vector3d position = _simu.robots().back()->body_pos("leg_" + std::to_string(_leg) + "_3");
                    // Store this position in vector of positions
                    _positions.push_back(position);
            }

            // Get list of positions
            void get(std::vector<Eigen::Vector3d> &positions)
            {
                    positions = _positions;
            }

    private:
            std::vector<Eigen::Vector3d> _positions;
            int _leg;
    };
  }
}

#endif
