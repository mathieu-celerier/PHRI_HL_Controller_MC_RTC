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
  double k = ctl.config()("PosTask")("kp");

  sva::PTransformd initPose;

  ctl.solver().addTask(ctl.eeTask);
  ctl.eeTask->reset();
  initPose = ctl.eeTask->get_ef_pose();
  // initOri = Eigen::Quaterniond(ctl.eeOriTask->orientation());
  ctl.eeTask->positionTask->refVel(Eigen::Vector3d::Zero());
  ctl.eeTask->orientationTask->refVel(Eigen::Vector3d::Zero());
  ctl.eeTask->positionTask->position(targetPos);
  ctl.eeTask->orientationTask->orientation(targetOri.toRotationMatrix());

  p_PPCTask = new PPCTask(ctl.timeStep, initPose.translation(), Eigen::Quaterniond(initPose.rotation()), targetPos, targetOri, reachingTime,rhoInf,k);

  ctl.reset({ctl.realRobots().robot().mbc().q});
  ctl.getPostureTask(ctl.robot().name())->weight(1000);

  ctl.logger().addLogEntry("PPC_error", [this]() {return p_PPCTask->getError();});
  ctl.logger().addLogEntry("PPC_lower bound", [this]() {return p_PPCTask->getLowBound();});
  ctl.logger().addLogEntry("PPC_upper bound", [this]() {return p_PPCTask->getUpBound();});
  ctl.logger().addLogEntry("PPC_command", [this]() {return p_PPCTask->getCommand();});
  ctl.logger().addLogEntry("PPC_mod err", [this]() {return p_PPCTask->getModErr();});
  ctl.logger().addLogEntry("PPC_ae", [this]() {return p_PPCTask->getAE();});
  ctl.logger().addLogEntry("PPC_kpnueps", [this]() {return p_PPCTask->getKpNu();});
}

bool humanLikeController_Interaction::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  sva::PTransformd currentPose =  ctl.robot().surfacePose("Arm");
  p_PPCTask->eval(currentPose.translation(),Eigen::Quaterniond(currentPose.rotation()));

  linearVel = p_PPCTask->getLinearVelocityCommand();
  angularVel = p_PPCTask->getAngularVelocityCommand();

  ctl.eeTask->positionTask->refVel(linearVel);
  ctl.eeTask->orientationTask->refVel(angularVel);

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
  ctl.logger().removeLogEntry("PPC_mod err");
  ctl.logger().removeLogEntry("PPC_ae");
  ctl.logger().removeLogEntry("PPC_kpnueps");

  ctl.solver().removeTask(ctl.eeTask);
}

EXPORT_SINGLE_STATE("Interaction", humanLikeController_Interaction)