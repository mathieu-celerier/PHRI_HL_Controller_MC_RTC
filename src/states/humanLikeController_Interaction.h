#pragma once

#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mc_control/fsm/State.h>

struct humanLikeController_Interaction : mc_control::fsm::State
{
    void configure(const mc_rtc::Configuration & config) override;

    void start(mc_control::fsm::Controller & ctl) override;

    bool run(mc_control::fsm::Controller & ctl) override;

    void teardown(mc_control::fsm::Controller & ctl) override;
private:
    void updateVelTargetCallback(const geometry_msgs::Twist& twist_msg);
    void get_ee_velocity_target(double dt);

    std::thread* thread;

    std::mutex target_mutex;
    Eigen::Vector3d linearVel;
    Eigen::Vector3d angularVel;
    bool runThread;
};