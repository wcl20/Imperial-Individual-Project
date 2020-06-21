#ifndef __DESCRIPTOR_HPP__
#define __DESCRIPTOR_HPP__

#include <dart/collision/CollisionFilter.hpp>
#include <dart/collision/CollisionObject.hpp>

namespace robot_dart
{
namespace descriptor
{
struct HexaDescriptor : BaseDescriptor
{
public:
        // Constructor
        HexaDescriptor(RobotDARTSimu &simu, size_t desc_dump = 50)
                : BaseDescriptor(simu, desc_dump) {}

        // This function is called every desc_dump steps of the simulation
        void operator()()
        {
                // Get collision results
                const dart::collision::CollisionResult& collison_result = _simu.world()->getLastCollisionResult();
                auto contacts = collison_result.getContacts();
                // Iterate all collsions
                for (auto it = contacts.begin(); it != contacts.end(); ++it) {
                    std::string object1 = it->collisionObject1->getShapeFrame()->getName();
                    std::string object2 = it->collisionObject2->getShapeFrame()->getName();
                    // Check one of the object is a leg
                    if (object1.find("leg") != std::string::npos || object2.find("leg") != std::string::npos) {
                        // Check one of the object is an obstacle
                        if (object1.find("obstacle") != std::string::npos || object2.find("obstacle") != std::string::npos) {
                            _collisions += 1;
                        }
                    }
                }
        }

        // Get number of collisions
        int get()
        {       int collisions = _collisions;
                // Reset collisions
                _collisions = 0;
                return collisions;
        }

private:
        int _collisions = 0;

};
}
}

#endif
