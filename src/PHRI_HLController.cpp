#include <hl_controller/PHRI_HLController.h>

PHRI_HLController::PHRI_HLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(new mc_solver::DynamicsConstraint(robots(), 0, solver().dt(), {0.1, 0.01, 0.5}, 0.9));
  
  // auto constraints = dynamicsConstraint;
  solver().addConstraintSet(dynamicsConstraint);
  auto posture_task = getPostureTask(robot().name());
  posture_task->stiffness(0.0);
  posture_task->damping(4);
  posture_task->weight(100);
  eeTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("tool_frame"));
  datastore().make<std::string>("ControlMode","");
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return getPostureTask(robot().name()); });

  logger().addLogEntry("Posture_task_target",[this]() {
    getPostureTarget();
    return posture_target_log;
  });

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

void PHRI_HLController::getPostureTarget(void)
{
  auto mc_posture = getPostureTask(robot().name())->posture();
  auto rjo = robot().refJointOrder();
  posture_target_log.resize(rjo.size());
  if (mc_posture.size() != 0)
  {
    for(size_t i = 0; i < rjo.size(); i++)
    {
      // std::cout << mc_posture[robot().jointIndexByName(rjo[i])][0] << " | ";
      posture_target_log[i] = mc_posture[robot().jointIndexByName(rjo[i])][0];
    }
    // std::cout << "\n";
  }
}