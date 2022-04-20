#include "humanLikeController_Interaction.h"

#include "../PHRI_HLController.h"

void humanLikeController_Interaction::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Interaction::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  std::cout << "[MC RTC pHRIController] Admittance control.\n";

  target_lin_vel = Eigen::Vector3d::Zero();
  target_ang_vel = Eigen::Vector3d::Zero();

  ctl.reset({ctl.realRobots().robot().mbc().q});

  ctl.solver().addTask(ctl.posTask);
  ctl.solver().addTask(ctl.oriTask);

  runThread = true;
  thread = new std::thread(&humanLikeController_Interaction::get_ee_velocity_target,this,ctl.timeStep);
}

bool humanLikeController_Interaction::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  if (target_mutex.try_lock()){
    ctl.posTask->refVel(target_lin_vel);
    ctl.oriTask->refVel(target_ang_vel);
    target_mutex.unlock();
  }

  if(ctl.config().has("switch"))
  {
    std::string out = ctl.config()("switch");
    ctl.config().remove("switch");
    output(out);
    
    return true;
  }

  return false;
}

void humanLikeController_Interaction::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  runThread = false;
  thread->join();
  ctl.solver().removeTask(ctl.posTask);
  ctl.solver().removeTask(ctl.oriTask);
}

void humanLikeController_Interaction::updateVelTargetCallback(const geometry_msgs::Twist::ConstPtr& twist) {
  if (target_mutex.try_lock()){
    // std::cout << "==========\n" << twist->linear << std::endl << "-----\n" << twist->angular << std::endl;
    target_lin_vel << twist->linear.x,twist->linear.y,twist->linear.z;
    target_ang_vel << twist->angular.x,twist->angular.y,twist->angular.z;
    target_mutex.unlock();
  }
}

void humanLikeController_Interaction::get_ee_velocity_target(double dt) {
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("target_twist",1000,&humanLikeController_Interaction::updateVelTargetCallback,this);

  ros::Rate rate(1.0/dt);
  while(runThread){
    rate.sleep();
    ros::spinOnce();
  }
}

EXPORT_SINGLE_STATE("Interaction", humanLikeController_Interaction)