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
            HexaDescriptor(RobotDARTSimu &simu, size_t desc_dump = 100)
                    : BaseDescriptor(simu, desc_dump) {
            }

            // This function is called every desc_dump steps of the simulation
            void operator()()
            {
                    // Get current position of the hexapod
                    Eigen::Vector3d position = _simu.robots().back()->skeleton()->getPositions().head(6).tail(3);
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
    };
  }
}

#endif
