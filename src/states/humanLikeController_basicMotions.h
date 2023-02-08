#pragma once

#include <mc_control/fsm/State.h>
#include "humanLikeController_Policy.h"

enum BasicMotions { pX, nX, pY, nY, pZ, nZ, stop };

struct humanLikeController_basicMotions : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
    Eigen::Vector3d stiffness;
    Eigen::Vector3d damping;
    
    BasicMotions state;
    double duration;
    double timeElapsed;
    double speed;
};