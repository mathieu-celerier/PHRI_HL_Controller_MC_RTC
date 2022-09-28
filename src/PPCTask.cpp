#include <hl_controller/PPCTask.h>

PPCTask::PPCTask(double timestep,Eigen::Vector3d _initPos,Eigen::Quaterniond _initOrientation,Eigen::Vector3d _targetPos,Eigen::Quaterniond _targetOrientation,double reachingTime,
    Eigen::Vector6d _rho_inf,double _kp, double _decay, Eigen::Vector6d _M,Eigen::Vector6d _targetVelocity,double _modulated_error_limit
) : t(0),dt(timestep),initPos(_initPos),initOrientation(_initOrientation),targetPos(_targetPos),targetOrientation(_targetOrientation),Td(reachingTime),rho_inf(_rho_inf),kp(_kp),decay(_decay),M(_M),targetVelocity(_targetVelocity),modulated_error_limit(_modulated_error_limit)
{
    Md << 1, 1, 1;
    Bd << 60, 60, 60;
    Kd << 200, 200, 200;

    maxVel << 0.1,0.1,0.1,M_PI/4,M_PI/4,M_PI/4;
    command = Eigen::Vector6d::Zero();
    ae = Eigen::Vector6d::Zero();
    kpnueps = Eigen::Vector6d::Zero();
    filtered_kpnueps = Eigen::Vector6d::Zero();
    d = Eigen::Vector3d::Zero();
    dp = Eigen::Vector3d::Zero();

    F = Eigen::Vector3d::Zero();

    error_zero.head<3>() = initPos - targetPos;
    error_zero.tail<3>() = sva::rotationError(targetOrientation.toRotationMatrix(),initOrientation.toRotationMatrix());
    error = error_zero;

    rho_zero = 2*(error_zero.cwiseAbs() + rho_inf);
    modulated_error = error_zero.array()/rho_zero.array();
    compute_performance_function();
    compute_performance_function_derivative();
    compute_transformed_error(); // Compute eps
    compute_a();
    compute_nuT();
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
        if (error_zero[i] >=0)
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
        if (error_zero[i] >=0)
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

void PPCTask::compute_displacement(void)
{
    Eigen::Vector6d k1 = impedance_model_derivative(d,dp);
    Eigen::Vector6d k2 = impedance_model_derivative(d+dt*k1.head<3>()/2,dp+dt*k1.tail<3>()/2);
    Eigen::Vector6d k3 = impedance_model_derivative(d+dt*k2.head<3>()/2,dp+dt*k2.tail<3>()/2);
    Eigen::Vector6d k4 = impedance_model_derivative(d+dt*k3.head<3>(),dp+dt*k3.tail<3>());
    
    d += (dt/6)*(k1.head<3>() + 2*k2.head<3>() + 2*k3.head<3>() + k4.head<3>());
    dp += (dt/6)*(k1.tail<3>() + 2*k2.tail<3>() + 2*k3.tail<3>() + k4.tail<3>());
}

Eigen::Vector6d PPCTask::impedance_model_derivative(Eigen::Vector3d x, Eigen::Vector3d v)
{
    Eigen::Vector6d va;
    va.head<3>() = v;
    va.tail<3>() = (F.array() - Bd.array()*v.array() - Kd.array()*x.array())/Md.array();
    return va;
}

double PPCTask::getTime(void)
{
    return t;
}

double PPCTask::getTd(void)
{
    return Td;
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
    return saturated_command;
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

Eigen::Vector6d PPCTask::getKpNuSat(void)
{
    return saturated_kpnueps;
}

Eigen::Vector6d PPCTask::getKpNuFilt(void)
{
    return filtered_kpnueps;
}

Eigen::Vector3d PPCTask::getF(void)
{
    return F;
}

Eigen::Vector3d PPCTask::getDisplacement(void)
{
    return d;
}

Eigen::Vector3d PPCTask::getTarget(void)
{
    return targetPos + d;
}

Eigen::Vector3d PPCTask::getLinearVelocityCommand(void)
{
    return saturated_command.head<3>();
}

Eigen::Vector3d PPCTask::getAngularVelocityCommand(void)
{
    return saturated_command.tail<3>();
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
    kpnueps = (k*nuT*epsilon);
    saturated_kpnueps = kpnueps.cwiseMin(maxVel).cwiseMax(-maxVel);
    filtered_kpnueps += decay*(saturated_kpnueps - filtered_kpnueps);

    command = -(ae + filtered_kpnueps - targetVelocity);
    saturated_command = command;
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

bool PPCTask::eval(Eigen::Vector3d currentPose, Eigen::Quaterniond currentOrientation, sva::ForceVecd currentWrench)
{
    F += decay*(-currentWrench.force() - F);
    // if(t > Td/2 and t < 3*Td/2)
    // {
    //     Eigen::Vector3d f;
    //     f << 10, 0, 0;
    //     F = f;
    // }
    // else
    // {
    //     F = Eigen::Vector3d::Zero();
    // }
    compute_displacement();

    // Compute current pose error
    error.head<3>() = currentPose - (targetPos + d);
    error.tail<3>() = Eigen::Vector3d::Zero(); // sva::rotationError(targetOrientation.toRotationMatrix(),currentOrientation.toRotationMatrix());

    // Compute PPC components
    compute_performance_function();
    compute_performance_function_derivative();
    modulated_error = error.array()/rho.array(); // Compute e/rho
    compute_transformed_error(); // Compute eps
    compute_a();
    compute_nuT();

    double k = (t < Td) ? kp : 0*kp; // Variable kp if needed

    t += dt;

    ae = a*error;
    kpnueps = (k*nuT*epsilon);
    saturated_kpnueps = kpnueps.cwiseMin(maxVel).cwiseMax(-maxVel);
    filtered_kpnueps += decay*(saturated_kpnueps - filtered_kpnueps);

    command = -(ae + filtered_kpnueps - targetVelocity);
    saturated_command = command;
    saturated_command[2] = 0.0;
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