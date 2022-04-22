#include "Thruster.hpp"

Thruster::Thruster(Eigen::Vector3d position, Eigen::Vector3d thrust) {
    this->position = position;
    this->maximum_thrust = thrust;
}