#include <hl_controller/PHRI_HLController.h>

PHRI_HLController::PHRI_HLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  solver().addConstraintSet(dynamicsConstraint);

  std::vector<double> val(1);
  // val[0] = 0.677978;
  val[0] = -0.05;
  posture_target["right_j0"] = val;
  val[0] = 0.0;
  posture_target["head_pan"] = val;
  // val[0] = -0.0394787;
  val[0] = 0.87;
  posture_target["right_j1"] = val;
  // val[0] = -1.48606;
  val[0] = -0.08;
  posture_target["right_j2"] = val;
  // val[0] = 1.56569;
  val[0] = -1.85;
  posture_target["right_j3"] = val;
  // val[0] = -0.0653148;
  val[0] = 0.02;
  posture_target["right_j4"] = val;
  // val[0] = 0.68418;
  val[0] = 1.6;
  posture_target["right_j5"] = val;
  // val[0] = -1.26174;
  val[0] = -3.0;
  posture_target["right_j6"] = val;
  getPostureTask(robot().name())->target(posture_target);

  getPostureTask(robot().name())->stiffness(0.0);
  getPostureTask(robot().name())->damping(35);
  getPostureTask(robot().name())->weight(100);
  eeTask = std::make_shared<mc_tasks::EndEffectorTask>(robot().frame("right_hand"));


  // Position task parameters
  Eigen::Vector3d stiffness, damping;
  stiffness << 0, 0, 100;
  damping << 20, 20, 20;
  eeTask->positionTask->stiffness(stiffness);
  eeTask->positionTask->damping(damping);
  eeTask->positionTask->weight(10000);

  // Orientation task parameters
  eeTask->orientationTask->setGains(100,20);
  eeTask->orientationTask->weight(0);

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
