#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_solver/DynamicsConstraint.h>
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

    std::shared_ptr<mc_tasks::EndEffectorTask> eeTask;
    std::vector<double> posture_target_log;
    std::vector<std::vector<double>> posture_target;
private:
    std::mutex mutex_posture;
    void getPostureTarget(void);
    mc_rtc::Configuration config_;
};