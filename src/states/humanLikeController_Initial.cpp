#include "humanLikeController_Initial.h"

#include <hl_controller/PHRI_HLController.h>

void humanLikeController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  std::cout << "[MC RTC pHRIController] Position control.\n";
  ctl.reset({ctl.realRobots().robot().mbc().q});
}

bool humanLikeController_Initial::run(mc_control::fsm::Controller & ctl_)
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

void humanLikeController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
}

EXPORT_SINGLE_STATE("Released", humanLikeController_Initial)