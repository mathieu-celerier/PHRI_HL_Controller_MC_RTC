#include "humanLikeController_Contact.h"

#include "../PHRI_HLController.h"

void humanLikeController_Contact::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Contact::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  
  std::cout << "[MC RTC pHRIController] Torque control.\n";
  ctl.reset({ctl.realRobots().robot().mbc().q});
}

bool humanLikeController_Contact::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  
  if(ctl.config().has("switch"))
  {
    std::string out = ctl.config()("switch");
    ctl.config().remove("switch");
    output(out);
    
    return true;
  }
  
  return false;
}

void humanLikeController_Contact::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);  
}

EXPORT_SINGLE_STATE("InContact", humanLikeController_Contact)