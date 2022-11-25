#include "humanLikeController_Initial.h"

#include <hl_controller/PHRI_HLController.h>

void humanLikeController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  std::cout << "[MC RTC pHRIController] Position control.\n";
  // ctl.reset({ctl.realRobots().robot().mbc().q});
  ctl.getPostureTask(ctl.robot().name())->weight(100);
  // ctl.getPostureTask(ctl.robot().name())->posture(ctl.posture_target);

  // Init GUI
  ctl.gui()->addElement(
    {"Controller","Init State"},
    mc_rtc::gui::Button(
      "Set current posture as target",
      [&ctl]() {
        ctl.posture_target = ctl.robot().q();
        ctl.postureTask->posture(ctl.posture_target);
      }
    )
  );
}

bool humanLikeController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  if(ctl.datastore().has("getPolicyState"))
  {
    ctrl_states state = ctl.datastore().call<ctrl_states>("getPolicyState");
    switch(state)
    {
      case Contact:
        output("MadeContact");
        return true;
      default:
        break;
    }
  }

  return false;
}

void humanLikeController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
}

EXPORT_SINGLE_STATE("Released", humanLikeController_Initial)