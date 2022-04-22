#ifndef THRUSTER_HPP
#define THRUSER_HPP

#include "eigen3/Eigen/Dense"

struct Thruster {
    Thruster();
    Thruster(Eigen::Vector3d position, Eigen::Vector3d maximum_thrust);
    Eigen::Vector3d position;
    Eigen::Vector3d maximum_thrust;
};

#endif //THRUSTER_HPP