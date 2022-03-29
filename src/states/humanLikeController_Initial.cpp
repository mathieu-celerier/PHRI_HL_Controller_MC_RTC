#include "humanLikeController_Initial.h"

#include "../PHRI_HLController.h"

void humanLikeController_Initial::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
}

bool humanLikeController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  output("OK");
  return true;
}

void humanLikeController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
}

EXPORT_SINGLE_STATE("humanLikeController_Initial", humanLikeController_Initial)
