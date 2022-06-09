#include "humanLikeController_Interaction.h"

#include <hl_controller/PHRI_HLController.h>

void humanLikeController_Interaction::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Interaction::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  std::cout << "[MC RTC pHRIController] Admittance control.\n";

  Eigen::Vector3d targetPos = ctl.config()("PosTask")("targetPose");
  Eigen::Quaterniond targetOri = ctl.config()("PosTask")("targetOrientation");
  double reachingTime = ctl.config()("PosTask")("duration");
  Eigen::Vector6d rhoInf = ctl.config()("PosTask")("max_error");
  Eigen::Vector6d k = ctl.config()("PosTask")("kp");

  ctl.solver().addTask(ctl.eePosTask);
  ctl.solver().addTask(ctl.eeOriTask);
  ctl.eePosTask->reset();
  ctl.eeOriTask->reset();
  ctl.eePosTask->refVel(Eigen::Vector3d::Zero());
  ctl.eeOriTask->refVel(Eigen::Vector3d::Zero());
  ctl.eePosTask->position(targetPos);
  ctl.eeOriTask->orientation(targetOri.toRotationMatrix());

  p_PPCTask = new PPCTask(ctl.timeStep, ctl.eePosTask->position(), Eigen::Quaterniond(ctl.eeOriTask->orientation()), targetPos, targetOri, reachingTime,rhoInf,k);

  ctl.reset({ctl.realRobots().robot().mbc().q});
}

bool humanLikeController_Interaction::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  p_PPCTask->eval(ctl.eePosTask->position(),Eigen::Quaterniond(ctl.eeOriTask->orientation()));

  ctl.eePosTask->refVel(p_PPCTask->getLinearVelocityCommand());
  ctl.eeOriTask->refVel(p_PPCTask->getAngularVelocityCommand());

  if(ctl.config().has("switch"))
  {
    std::string out = ctl.config()("switch");
    ctl.config().remove("switch");
    output(out);
    
    return true;
  }

  return false;
}

void humanLikeController_Interaction::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  ctl.solver().removeTask(ctl.eePosTask);
  ctl.solver().removeTask(ctl.eeOriTask);
}

EXPORT_SINGLE_STATE("Interaction", humanLikeController_Interaction)