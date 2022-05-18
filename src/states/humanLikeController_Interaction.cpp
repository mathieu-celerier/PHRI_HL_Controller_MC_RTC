#include "humanLikeController_Interaction.h"

#include "../PHRI_HLController.h"

void humanLikeController_Interaction::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Interaction::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  std::cout << "[MC RTC pHRIController] Admittance control.\n";

  linearVel = Eigen::Vector3d::Zero();
  angularVel = Eigen::Vector3d::Zero();

  ctl.solver().addTask(ctl.eePosTask);
  ctl.solver().addTask(ctl.eeOriTask);

  ctl.reset({ctl.realRobots().robot().mbc().q});

  runThread = true;
  thread = new std::thread(&humanLikeController_Interaction::get_ee_velocity_target,this,ctl.timeStep);
}

bool humanLikeController_Interaction::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  if (target_mutex.try_lock()){
    ctl.eePosTask->reset();
    ctl.eeOriTask->reset();
    ctl.eePosTask->refVel(linearVel);
    ctl.eeOriTask->refVel(angularVel);
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

  ctl.solver().removeTask(ctl.eePosTask);
  ctl.solver().removeTask(ctl.eeOriTask);

  runThread = false;
  thread->join();
}

void humanLikeController_Interaction::updateVelTargetCallback(const geometry_msgs::Twist& twist_msg) {
  if (target_mutex.try_lock()){
    linearVel[0] = twist_msg.linear.x;
    linearVel[1] = twist_msg.linear.y;
    linearVel[2] = twist_msg.linear.z;
    angularVel[0] = twist_msg.angular.x;
    angularVel[1] = twist_msg.angular.y;
    angularVel[2] = twist_msg.angular.z;
    target_mutex.unlock();
  }
}

void humanLikeController_Interaction::get_ee_velocity_target(double dt) {
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("target_twist",10,&humanLikeController_Interaction::updateVelTargetCallback,this);

  ros::Rate rate(1.0/dt);
  while(runThread){
    rate.sleep();
    ros::spinOnce();
  }
}

EXPORT_SINGLE_STATE("Interaction", humanLikeController_Interaction)