#include "humanLikeController_Interaction.h"

#include "../PHRI_HLController.h"

void humanLikeController_Interaction::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Interaction::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  std::cout << "[MC RTC pHRIController] Admittance control.\n";

  joint_vel = Eigen::VectorXd::Zero(7);

  ctl.reset({ctl.realRobots().robot().mbc().q});

  runThread = true;
  thread = new std::thread(&humanLikeController_Interaction::get_ee_velocity_target,this,ctl.timeStep);
}

bool humanLikeController_Interaction::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  if (target_mutex.try_lock()){
    ctl_.getPostureTask(ctl_.robot().name())->refVel(joint_vel);
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
}

void humanLikeController_Interaction::updateVelTargetCallback(const std_msgs::Float32MultiArray::ConstPtr& joint_vel_msg) {
  if (target_mutex.try_lock()){
    std::vector<double> data(joint_vel_msg->data.begin(),joint_vel_msg->data.end());
    joint_vel = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(data.data(),data.size());
    target_mutex.unlock();
  }
}

void humanLikeController_Interaction::get_ee_velocity_target(double dt) {
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("target_joint_vel",1000,&humanLikeController_Interaction::updateVelTargetCallback,this);

  ros::Rate rate(1.0/dt);
  while(runThread){
    rate.sleep();
    ros::spinOnce();
  }
}

EXPORT_SINGLE_STATE("Interaction", humanLikeController_Interaction)