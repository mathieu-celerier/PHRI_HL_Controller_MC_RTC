#include "PHRI_HLController.h"

PHRI_HLController::PHRI_HLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  solver().addConstraintSet(dynamicsConstraint);

  getPostureTask(robot().name())->stiffness(1);
  getPostureTask(robot().name())->damping(16*sqrt(5));
  
  // Position and Orientation tasks
  posTask = std::make_shared<mc_tasks::PositionTask>("panda_link8", robots(), robots().robotIndex());
  posTask->setGains(100,20);
  posTask->weight(10000.0);
  oriTask = std::make_shared<mc_tasks::OrientationTask>("panda_link8", robots(), robots().robotIndex());
  oriTask->setGains(100,20);
  oriTask->weight(10000.0);

  mc_rtc::log::success("HumanLike_PHRI_Controller init done ");
}

bool PHRI_HLController::run()
{
  bool ret = mc_control::fsm::Controller::run(mc_solver::FeedbackType::Joints);
  computeTorques();
  return ret;
}

void PHRI_HLController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void PHRI_HLController::computeTorques()
{
  if (!robot().encoderVelocities().empty())
  {
    const auto &real = realRobots().robot();
    robot().mbc().q = real.mbc().q;
    const auto &alphaIn = robot().encoderVelocities();
    for (size_t i = 0; i < robot().refJointOrder().size(); ++i)
    {
      const auto &j = robot().refJointOrder()[i];
      auto jIndex = robot().jointIndexByName(j);
      robot()
          .mbc()
          .alpha[jIndex][0] = alphaIn[i];
    }
  }

  robot().forwardKinematics();
  robot().forwardVelocity();
}