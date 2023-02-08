#include "humanLikeController_basicMotions.h"

#include <hl_controller/PHRI_HLController.h>

void humanLikeController_basicMotions::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_basicMotions::start(mc_control::fsm::Controller & ctl_)
{
    auto & ctl = static_cast<PHRI_HLController &>(ctl_);

    sva::PTransformd initPose;

    ctl.solver().addTask(ctl.eeTask);
    ctl.eeTask->reset();
    initPose = ctl.eeTask->get_ef_pose();
    stiffness << 0, 0, 0;
    damping << 20, 20, 20;
    ctl.eeTask->positionTask->stiffness(stiffness);
    ctl.eeTask->positionTask->damping(damping);
    ctl.eeTask->positionTask->refVel(Eigen::Vector3d::Zero());

    duration = 3;
    speed = 0.2;
    timeElapsed = 0.0;
    state = BasicMotions::pX;

    std::cout << "[MC RTC pHRIController] Basic motion testing.\n";
    // ctl.reset({ctl.realRobots().robot().mbc().q});
    ctl.getPostureTask(ctl.robot().name())->weight(100);
    // ctl.getPostureTask(ctl.robot().name())->posture(ctl.posture_target);
    }

bool humanLikeController_basicMotions::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  timeElapsed += ctl.timeStep;
  switch(state)
  {
    case BasicMotions::pX:
        ctl.eeTask->positionTask->refVel(Eigen::Vector3d(speed,0,0));
        if (timeElapsed >= duration)
        {
            state = BasicMotions::nX;
            timeElapsed = 0.0;
        }
        break;
    case BasicMotions::nX:
        ctl.eeTask->positionTask->refVel(Eigen::Vector3d(-speed,0,0));
        if (timeElapsed >= duration)
        {
            state = BasicMotions::pY;
            timeElapsed = 0.0;
        }
        break;
    case BasicMotions::pY:
        ctl.eeTask->positionTask->refVel(Eigen::Vector3d(0,speed,0));
        if (timeElapsed >= duration)
        {
            state = BasicMotions::nY;
            timeElapsed = 0.0;
        }
        break;
    case BasicMotions::nY:
        ctl.eeTask->positionTask->refVel(Eigen::Vector3d(0,-speed,0));
        if (timeElapsed >= duration)
        {
            state = BasicMotions::pZ;
            timeElapsed = 0.0;
        }
        break;
    case BasicMotions::pZ:
        ctl.eeTask->positionTask->refVel(Eigen::Vector3d(0,0,speed));
        if (timeElapsed >= duration)
        {
            state = BasicMotions::nZ;
            timeElapsed = 0.0;
        }
        break;
    case BasicMotions::nZ:
        ctl.eeTask->positionTask->refVel(Eigen::Vector3d(0,0,-speed));
        if (timeElapsed >= duration)
        {
            state = BasicMotions::stop;
            timeElapsed = 0.0;
        }
        break;
    case BasicMotions::stop:
        ctl.eeTask->positionTask->refVel(Eigen::Vector3d::Zero());
        break;
  }
  
  return false;
}

void humanLikeController_basicMotions::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);  
}

EXPORT_SINGLE_STATE("BasicMotion", humanLikeController_basicMotions)