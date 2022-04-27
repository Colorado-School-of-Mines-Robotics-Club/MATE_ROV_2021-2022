#include "flight_controller/Thruster.hpp"

Thruster::Thruster() : position(0,0,0), maximum_thrust(0,0,0) {}

Thruster::Thruster(Eigen::Vector3d position, Eigen::Vector3d maximum_thrust) {
    this->position = position;
    this->maximum_thrust = maximum_thrust;
}