#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/PositionTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_rtc/ros.h>

#include "api.h"

#define POS_CTRL 0
#define TRQ_CTRL 1

struct PHRI_HLController_DLLAPI PHRI_HLController : public mc_control::fsm::Controller
{
    PHRI_HLController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

    bool run() override;

    void reset(const mc_control::ControllerResetData & reset_data) override;

    void computeTorques();
private:
    mc_rtc::Configuration config_;
};