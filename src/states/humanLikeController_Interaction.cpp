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

  Eigen::Vector3d initPose;
  Eigen::Quaterniond initOri;

  ctl.solver().addTask(ctl.eePosTask);
  ctl.solver().addTask(ctl.eeOriTask);
  ctl.eePosTask->reset();
  ctl.eeOriTask->reset();
  initPose = ctl.eePosTask->position();
  initOri = Eigen::Quaterniond(ctl.eeOriTask->orientation());
  ctl.eePosTask->refVel(Eigen::Vector3d::Zero());
  ctl.eeOriTask->refVel(Eigen::Vector3d::Zero());
  ctl.eePosTask->position(targetPos);
  ctl.eeOriTask->orientation(targetOri.toRotationMatrix());

  p_PPCTask = new PPCTask(ctl.timeStep, initPose, initOri, targetPos, targetOri, reachingTime,rhoInf,k);

  ctl.reset({ctl.realRobots().robot().mbc().q});

  ctl.logger().addLogEntry("PPC_error", [this]() {return p_PPCTask->getError();});
  ctl.logger().addLogEntry("PPC_lower bound", [this]() {return p_PPCTask->getLowBound();});
  ctl.logger().addLogEntry("PPC_upper bound", [this]() {return p_PPCTask->getUpBound();});
  ctl.logger().addLogEntry("PPC_command", [this]() {return p_PPCTask->getCommand();});
}

bool humanLikeController_Interaction::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  sva::PTransformd currentPose =  ctl.robot().surfacePose("Arm");
  p_PPCTask->eval(currentPose.translation(),Eigen::Quaterniond(currentPose.rotation()));

  linearVel = p_PPCTask->getLinearVelocityCommand();
  angularVel = p_PPCTask->getAngularVelocityCommand();

  ctl.eePosTask->refVel(linearVel);
  ctl.eeOriTask->refVel(angularVel);

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

  delete p_PPCTask;

  ctl.logger().removeLogEntry("PPC_error");
  ctl.logger().removeLogEntry("PPC_lower bound");
  ctl.logger().removeLogEntry("PPC_upper bound");
  ctl.logger().removeLogEntry("PPC_command");

  ctl.solver().removeTask(ctl.eePosTask);
  ctl.solver().removeTask(ctl.eeOriTask);
}

EXPORT_SINGLE_STATE("Interaction", humanLikeController_Interaction)