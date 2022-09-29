#include "humanLikeController_Policy.h"

#include <hl_controller/PHRI_HLController.h>

void humanLikeController_Policy::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Policy::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
}

bool humanLikeController_Policy::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

//   if(ctl.config().has("switch"))
//   {
//     std::string out = ctl.config()("switch");
//     ctl.config().remove("switch");
//     output(out);
    
//     return true;
//   }

  return false;
}

void humanLikeController_Policy::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
}

EXPORT_SINGLE_STATE("Policy", humanLikeController_Policy)