#include "PHRI_HLController.h"

PHRI_HLController::PHRI_HLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  solver().addConstraintSet(dynamicsConstraint);

  getPostureTask(robot().name())->stiffness(0.0);
  getPostureTask(robot().name())->damping(2*sqrt(5));

  eePosTask = std::make_shared<mc_tasks::PositionTask>(robot().frame("Arm"));
  eeOriTask = std::make_shared<mc_tasks::OrientationTask>(robot().frame("Arm"));


  // Position task parameters
  eePosTask->setGains(1,100);
  eePosTask->weight(50000);

  // Orientation task parameters
  eeOriTask->setGains(10,0);
  eeOriTask->weight(50000);

  logger().addLogEntry("EE_Pos_Task_eval", [this]() {return eePosTask->eval();});
  logger().addLogEntry("EE_Pos_Task_refVel", [this]() {return eePosTask->refVel();});
  logger().addLogEntry("EE_Ori_Task_eval", [this]() {return eePosTask->eval();});
  logger().addLogEntry("EE_Ori_Task_refVel", [this]() {return eePosTask->refVel();});

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

  logger().removeLogEntry("EE_Pos_Task_eval");
  logger().removeLogEntry("EE_Pos_Task_refVel");
  logger().removeLogEntry("EE_Ori_Task_eval");
  logger().removeLogEntry("EE_Ori_Task_refVel");
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