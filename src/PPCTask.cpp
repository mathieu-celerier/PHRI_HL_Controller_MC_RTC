#include <hl_controller/PPCTask.h>

PPCTask::PPCTask(double timestep,Eigen::Vector3d _initPos,Eigen::Quaterniond _initOrientation,Eigen::Vector3d _targetPos,Eigen::Quaterniond _targetOrientation,double reachingTime,
    Eigen::Vector6d _rho_inf,double _kp,Eigen::Vector6d _M,Eigen::Vector6d _targetVelocity,double _modulated_error_limit
) : t(0),dt(timestep),initPos(_initPos),initOrientation(_initOrientation),targetPos(_targetPos),targetOrientation(_targetOrientation),Td(reachingTime),rho_inf(_rho_inf),kp(_kp),M(_M),targetVelocity(_targetVelocity),modulated_error_limit(_modulated_error_limit)
{
    maxVel << 0.25,0.25,0.25,0.5,0.5,0.5;
    command = Eigen::Vector6d::Zero();
    ae = Eigen::Vector6d::Zero();
    kpnueps = Eigen::Vector6d::Zero();

    error_zero.head<3>() = targetPos - initPos;
    error_zero.tail<3>() = initOrientation.w()*targetOrientation.vec() - targetOrientation.w()*initOrientation.vec() - skew(targetOrientation.vec())*initOrientation.vec();
    error = error_zero;

    rho_zero = 2*(error_zero.cwiseAbs() + rho_inf);
    modulated_error = error_zero.array()/rho_zero.array();
    compute_performance_function();
    compute_performance_function_derivative();
}

void PPCTask::compute_performance_function(void)
{
    double time_ratio = t/Td;
    if (t <= Td)
    {
        rho = rho_zero + (rho_inf - rho_zero)*(10*pow(time_ratio,3) - 15*pow(time_ratio,4) + 6*pow(time_ratio,5));
    }
    else
    {
        rho = rho_inf;
    }
}

void PPCTask::compute_performance_function_derivative(void)
{
    if (t <= Td)
    {
        dRho = (30*pow(t,2)*(rho_inf-rho_zero)*pow(Td - t,2))/pow(Td,5);
    }
    else
    {
        dRho = Eigen::Vector6d::Zero();
    }
}

void PPCTask::compute_transformed_error(void)
{
    for (int i=0; i < 6; i++)
    {
        double mod_err = std::max(-modulated_error_limit,std::min(modulated_error[i],modulated_error_limit)); // Saturate modulated error to prevent having values >= 1
        if (error[i] >=0)
        {
            epsilon[i] = log((M[i] + mod_err)/(M[i]*(1-mod_err)));
        }
        else
        {
            epsilon[i] = log((M[i]*(1+mod_err))/(M[i] - mod_err));
        }
    }
}

void PPCTask::compute_a(void)
{
    a = ((Eigen::Vector6d)(-dRho.array()/rho.array())).asDiagonal();
}

void PPCTask::compute_nuT(void)
{
    Eigen::Vector6d vec_nuT;
    for (int i=0; i < 6; i++)
    {
        double mod_err = std::max(-modulated_error_limit,std::min(modulated_error[i],modulated_error_limit)); // Saturate modulated error to prevent having values >= 1
        if (error[i] >=0)
        {
            vec_nuT[i] = -((M[i]+1)/((mod_err-1)*(M[i]+mod_err)))*(1.0/rho[i]);
        }
        else
        {
            vec_nuT[i] = ((M[i]+1)/((mod_err+1)*(M[i]-mod_err)))*(1.0/rho[i]);
        }
    }
    nuT = vec_nuT.asDiagonal();
}

Eigen::Vector6d PPCTask::getError(void)
{
    return error;
}

Eigen::Vector6d PPCTask::getUpBound(void)
{
    return rho;
}

Eigen::Vector6d PPCTask::getLowBound(void)
{
    return -rho;
}

Eigen::Vector6d PPCTask::getCommand(void)
{
    return command;
}

Eigen::Vector6d PPCTask::getModErr(void)
{
    return modulated_error;
}

Eigen::Vector6d PPCTask::getAE(void)
{
    return ae;
}

Eigen::Vector6d PPCTask::getKpNu(void)
{
    return kpnueps;
}

Eigen::Vector3d PPCTask::getLinearVelocityCommand(void)
{
    return command.head<3>();
}

Eigen::Vector3d PPCTask::getAngularVelocityCommand(void)
{
    return command.tail<3>();
}

bool PPCTask::eval(Eigen::Vector3d currentPose, Eigen::Quaterniond currentOrientation)
{
    // Compute current pose error
    error.head<3>() = currentPose - targetPos;
    error.tail<3>() = sva::rotationError(targetOrientation.toRotationMatrix(),currentOrientation.toRotationMatrix());

    // Compute PPC components
    compute_performance_function();
    compute_performance_function_derivative();
    modulated_error = error.array()/rho.array(); // Compute e/rho
    compute_transformed_error(); // Compute eps
    compute_a();
    compute_nuT();

    double k = (t < Td) ? kp : 1*kp; // Variable kp if needed

    t += dt;

    ae = a*error;
    kpnueps = k*nuT*epsilon;

    command = -(ae + kpnueps - targetVelocity);
    command = command.cwiseMin(maxVel).cwiseMax(-maxVel);
    // std::cout << "===================================" << std::endl;
    // std::cout << error_zero.transpose() << std::endl;
    // std::cout << error.transpose() << std::endl;
    // std::cout << rho.transpose() << std::endl;
    // std::cout << dRho.transpose() << std::endl;
    // std::cout << modulated_error.transpose() << std::endl;
    // std::cout << a << std::endl;
    // std::cout << k << std::endl;
    // std::cout << nuT << std::endl;
    // std::cout << epsilon.transpose() << std::endl;
    // std::cout << command.transpose() << std::endl;

    return true;
}