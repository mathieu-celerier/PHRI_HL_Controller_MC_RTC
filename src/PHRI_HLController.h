#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>

#include "api.h"

struct PHRI_HLController_DLLAPI PHRI_HLController : public mc_control::fsm::Controller
{
    PHRI_HLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;
private:
    mc_rtc::Configuration config_;
};