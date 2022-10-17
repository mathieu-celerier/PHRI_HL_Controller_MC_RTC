#include <hl_controller/PHRI_HLController.h>

PHRI_HLController::PHRI_HLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  solver().addConstraintSet(dynamicsConstraint);
  getPostureTask(robot().name())->stiffness(0.0);
  getPostureTask(robot().name())->damping(35);
  getPostureTask(robot().name())->weight(100);
  eeTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("tool_frame"));
  datastore().make<std::string>("ControlMode","");

  logger().addLogEntry("ControlMode",
    [this]() {
      auto mode = datastore().get<std::string>("ControlMode");
      if (mode.compare("") == 0) return 0;
      if (mode.compare("Position") == 0) return 1;
      if (mode.compare("Velocity") == 0) return 2;
      if (mode.compare("Torque") == 0) return 3;
      return 0;
    }
  );

  mc_rtc::log::success("HumanLike_PHRI_Controller init done ");
}

bool PHRI_HLController::run()
{
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  bool ret;

  if (ctrl_mode.compare("Position") == 0)
  {
    ret = mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    ret = mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
    //computeTorques();
  }

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
