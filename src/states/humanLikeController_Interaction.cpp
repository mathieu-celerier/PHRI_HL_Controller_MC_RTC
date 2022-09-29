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
  double decay = ctl.config()("PosTask")("filter_decay");

  sva::PTransformd initPose;

  ctl.solver().addTask(ctl.eeTask);
  ctl.eeTask->reset();
  initPose = ctl.eeTask->get_ef_pose();
  stiffness << 0, 0, 0;
  damping << 20, 20, 20;
  ctl.eeTask->positionTask->stiffness(stiffness);
  ctl.eeTask->positionTask->damping(damping);
  ctl.eeTask->positionTask->refVel(Eigen::Vector3d::Zero());
  ctl.eeTask->orientationTask->refVel(Eigen::Vector3d::Zero());
  ctl.eeTask->positionTask->position(targetPos);
  ctl.eeTask->orientationTask->orientation(targetOri.toRotationMatrix());

  p_PPCTask = new PPCTask(ctl.timeStep, initPose.translation(), Eigen::Quaterniond(initPose.rotation()), targetPos, targetOri, reachingTime,rhoInf,k,decay);

  ctl.reset({ctl.realRobots().robot().mbc().q});
  ctl.getPostureTask(ctl.robot().name())->weight(100);
  ctl.getPostureTask(ctl.robot().name())->target(ctl.posture_target);

  ctl.logger().addLogEntry("PPC_error", [this]() {return p_PPCTask->getError();});
  ctl.logger().addLogEntry("PPC_lower bound", [this]() {return p_PPCTask->getLowBound();});
  ctl.logger().addLogEntry("PPC_upper bound", [this]() {return p_PPCTask->getUpBound();});
  ctl.logger().addLogEntry("PPC_command", [this]() {return p_PPCTask->getCommand();});
  ctl.logger().addLogEntry("PPC_ae", [this]() {return p_PPCTask->getAE();});
  ctl.logger().addLogEntry("PPC_kpnueps", [this]() {return p_PPCTask->getKpNu();});
  ctl.logger().addLogEntry("PPC_kpnueps_sat", [this]() {return p_PPCTask->getKpNuSat();});
  ctl.logger().addLogEntry("PPC_kpnueps_filt", [this]() {return p_PPCTask->getKpNuFilt();});
  ctl.logger().addLogEntry("PPC_F", [this]() {return p_PPCTask->getF();});
  ctl.logger().addLogEntry("PPC_d", [this]() {return p_PPCTask->getDisplacement();});
  ctl.logger().addLogEntry("EE_Pos_Task_eval", [&ctl]() {return ctl.eeTask->positionTask->eval();});
  ctl.logger().addLogEntry("EE_Pos_Task_speed", [&ctl]() {return ctl.eeTask->positionTask->speed();});
  ctl.logger().addLogEntry("EE_Ori_Task_eval", [&ctl]() {return ctl.eeTask->orientationTask->eval();});
  ctl.logger().addLogEntry("EE_Ori_Task_speed", [&ctl]() {return ctl.eeTask->orientationTask->speed();});
}

bool humanLikeController_Interaction::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  sva::PTransformd currentPose =  ctl.robot().frame("tool_frame").position();
  // std::cout << ctl.robot().forceSensors()[0].worldWrenchWithoutGravity(ctl.robot()) << std::endl;
  p_PPCTask->eval(currentPose.translation(),Eigen::Quaterniond(currentPose.rotation()),ctl.robot().forceSensors()[0].worldWrenchWithoutGravity(ctl.robot()));
  ctl.eeTask->positionTask->position(p_PPCTask->getTarget());
  // p_PPCTask->eval(currentPose.translation(),Eigen::Quaterniond(currentPose.rotation()));

  linearVel = p_PPCTask->getLinearVelocityCommand();
  angularVel = p_PPCTask->getAngularVelocityCommand();

  ctl.eeTask->positionTask->refVel(linearVel);
  ctl.eeTask->orientationTask->refVel(angularVel);

  double Td = p_PPCTask->getTd();
  double t = p_PPCTask->getTime();
  double stiff = std::min(std::max((200.0/1.0)*(t-Td),0.0),200.0);
  double damp = std::min(std::max(((2*sqrt(200.0) - 20)/1.0)*(t-Td),0.0),(2*sqrt(200.0) - 20));

  stiffness << stiff, stiff, 100;
  damping << 20+damp, 20+damp, 20;
  ctl.eeTask->positionTask->stiffness(stiffness);
  ctl.eeTask->positionTask->damping(damping);

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
  ctl.logger().removeLogEntry("PPC_ae");
  ctl.logger().removeLogEntry("PPC_kpnueps");
  ctl.logger().removeLogEntry("PPC_kpnueps_sat");
  ctl.logger().removeLogEntry("PPC_kpnueps_filt");
  ctl.logger().removeLogEntry("PPC_F");
  ctl.logger().removeLogEntry("PPC_d");
  ctl.logger().removeLogEntry("EE_Pos_Task_eval");
  ctl.logger().removeLogEntry("EE_Pos_Task_speed");
  ctl.logger().removeLogEntry("EE_Ori_Task_eval");
  ctl.logger().removeLogEntry("EE_Ori_Task_speed");

  ctl.solver().removeTask(ctl.eeTask);
}

EXPORT_SINGLE_STATE("Interaction", humanLikeController_Interaction)