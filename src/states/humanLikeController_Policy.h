#pragma once

#include <mc_control/fsm/State.h>
#include <hl_controller/utils/ROSSubscriber.h>

enum ctrl_states { Released, Contact, Active };

struct humanLikeController_Policy : mc_control::fsm::State
{

    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;

private:
    void rosSpinner();
    void policy(mc_control::fsm::Controller & ctl_);
    bool generateTask(mc_control::fsm::Controller & ctl_);

private:
    double controller_dt_;
    mc_rtc::Configuration config_;
    std::shared_ptr<ros::NodeHandle> nh_;
    std::thread spinThread_;
    double maxTime_ = 0.05;
    double freq_ = 100;
    bool interact_btn = false;
    bool start_btn = false;
    double taskSpeed;
    double minTaskTime;
    Eigen::Vector6d rho_inf;
    double kp;
    double filter_decay;
    sva::PTransformd initPose;

    MCGamePiece tracked_piece;
    ctrl_states currentState = Released;
    std::string game_topic_ = "/catching_game/state";
    std::string pose_pub_topic = "robot_ee_pose";
    std::string btn_start_pub_topic = "robot_start_btn";
    std::string btn_interact_pub_topic = "robot_interact_btn";
    ROSGameStateSubscriber game_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher start_btn_pub_;
    ros::Publisher interact_btn_pub_;
};
