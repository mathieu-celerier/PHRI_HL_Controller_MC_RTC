#pragma once

#include <stdlib.h>
#include <iostream>
#include <SpaceVecAlg/SpaceVecAlg>
#include <algorithm>

#include <hl_controller/utils/eigen_utils.h>

class PPCTask
{
public:
    // Constructor
    PPCTask(
        double timestep,
        Eigen::Vector3d _initPos,
        Eigen::Quaterniond _initOrientation,
        Eigen::Vector3d _targetPos,
        Eigen::Quaterniond _targetOrientation,
        double reachingTime,
        Eigen::Vector6d rho_inf,
        Eigen::Vector6d kp,
        Eigen::Vector6d M = Eigen::Vector6d::Ones(),
        Eigen::Vector6d targetVelocity = Eigen::Vector6d::Zero(),
        double modulated_error_limit = 1-1e-9
    );

    // Getter/Setter
    Eigen::Vector6d getError(void);
    Eigen::Vector6d getUpBound(void);
    Eigen::Vector6d getLowBound(void);
    Eigen::Vector6d getCommand(void);
    Eigen::Vector3d getLinearVelocityCommand(void);
    Eigen::Vector3d getAngularVelocityCommand(void);

    bool eval(Eigen::Vector3d currentPose, Eigen::Quaterniond currentOrientation);
    bool eval(Eigen::Vector3d currentPose, Eigen::Quaterniond currentOrientation, Eigen::Vector6d currentWrench);

private:
    // PPC methods
    void compute_performance_function(void);
    void compute_performance_function_derivative(void);
    void compute_transformed_error(void);
    void compute_a(void);
    void compute_nuT(void);

private:
    // Time properties
    double t;
    double dt;

    // position/velocity properties
    Eigen::Vector3d initPos;
    Eigen::Quaterniond initOrientation;
    Eigen::Vector3d targetPos;
    Eigen::Quaterniond targetOrientation;
    Eigen::Vector6d targetVelocity;

    // PPC config properties
    double Td; // Desired reaching time
    Eigen::Vector6d error_zero;
    Eigen::Vector6d rho_zero; // Performance bound starting value
    Eigen::Vector6d rho_inf; // Performance bound target value
    Eigen::Vector6d M; // Lower bound factor 0 < M <= 1
    Eigen::Vector6d kp; // Control gain for boundaries repulsive component
    double modulated_error_limit; // Prevent values >= 1 and so log compute error
    Eigen::Vector3d Md; // Impedance model acceleration parameter
    Eigen::Vector3d Bd; // Impedance model velocity parameter
    Eigen::Vector3d Kd; // Impedance model position parameter
    Eigen::Vector6d maxVel;

    // PPC properties
    Eigen::Vector6d error; // Current Task error
    Eigen::Vector6d rho; // Current bound
    Eigen::Vector6d dRho; // Current bound derivative
    Eigen::Vector6d modulated_error; // Current error modulated by the current bound
    Eigen::Vector6d epsilon; // Transformed current error
    Eigen::Matrix6d a;
    Eigen::Matrix6d nuT;
    Eigen::Vector6d command; // Current desired velocity
};