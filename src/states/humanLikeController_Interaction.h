#pragma once

#include <thread>
#include <mc_control/fsm/State.h>

#include <hl_controller/PPCTask.h>

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
};