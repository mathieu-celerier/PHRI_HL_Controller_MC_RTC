#include "PHRI_HLController.h"

PHRI_HLController::PHRI_HLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{

  mc_rtc::log::success("humanLikeController init done ");
}

bool PHRI_HLController::run()
{
  return mc_control::fsm::Controller::run();
}

void PHRI_HLController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


