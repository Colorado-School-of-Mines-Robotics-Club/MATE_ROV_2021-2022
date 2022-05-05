#include "flight_controller/Thruster.hpp"

Thruster::Thruster() : position(0,0,0), thrust(0,0,0) {}

Thruster::Thruster(Eigen::Vector3d position, Eigen::Vector3d thrust) {
    this->position = position;
    this->thrust = thrust;
}