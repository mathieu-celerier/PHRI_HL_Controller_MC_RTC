#include "humanLikeController_Contact.h"

#include <hl_controller/PHRI_HLController.h>

void humanLikeController_Contact::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Contact::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  
  std::cout << "[MC RTC pHRIController] Torque control.\n";
  // ctl.reset({ctl.realRobots().robot().mbc().q});
  ctl.getPostureTask(ctl.robot().name())->weight(100);
  ctl.getPostureTask(ctl.robot().name())->posture(ctl.posture_target);
}

bool humanLikeController_Contact::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  
  if(ctl.datastore().has("getPolicyState"))
  {
    ctrl_states state = ctl.datastore().call<ctrl_states>("getPolicyState");
    switch(state)
    {
      case Released:
        output("Release");
        return true;
      case Active:
        output("Interact");
        return true;
      default:
        break;
    }
  }
  
  return false;
}

void humanLikeController_Contact::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);  
}

EXPORT_SINGLE_STATE("InContact", humanLikeController_Contact)