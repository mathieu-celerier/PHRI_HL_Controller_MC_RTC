#pragma once

#include <mc_control/fsm/State.h>

#include <hl_controller/PPCTask.h>
#include "humanLikeController_Policy.h"

#define MAX_STIFF 100.0

struct humanLikeController_Interaction : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
    PPCTask* p_PPCTask;
    Eigen::Vector3d linearVel;
    Eigen::Vector3d angularVel;
    Eigen::Vector3d stiffness;
    Eigen::Vector3d damping;
};