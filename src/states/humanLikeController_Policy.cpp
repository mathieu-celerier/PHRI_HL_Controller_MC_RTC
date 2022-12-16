#include "humanLikeController_Policy.h"

#include <hl_controller/PHRI_HLController.h>

void humanLikeController_Policy::configure(const mc_rtc::Configuration & config)
{
}

void humanLikeController_Policy::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  nh_ = mc_rtc::ROSBridge::get_node_handle();
  spinThread_ = std::thread(std::bind(&humanLikeController_Policy::rosSpinner, this));

  mc_rtc::log::info("Subscribing to {}", game_topic_);

  game_sub_.subscribe(*nh_, game_topic_);
  game_sub_.maxTime(maxTime_);
  pose_pub_ = nh_->advertise<geometry_msgs::Pose>(pose_pub_topic,1);
  start_btn_pub_ = nh_->advertise<std_msgs::Bool>(btn_start_pub_topic,1);
  interact_btn_pub_ = nh_->advertise<std_msgs::Bool>(btn_interact_pub_topic,1);
  controller_dt_ = ctl.timeStep;

  initPose = ctl.robot().frame("tool_frame").position();

  // Load task specific config
  taskSpeed = ctl.config()("PPCTask")("speed");
  minTaskTime = ctl.config()("PPCTask")("minDuration");
  rho_inf = ctl.config()("PPCTask")("rhoInf");
  kp = ctl.config()("PPCTask")("kp");
  filter_decay = ctl.config()("PPCTask")("filterDecay");

  // Manage datastore
  ctl.datastore().make<sva::PTransformd>("ppcTargetPose",sva::PTransformd());
  ctl.datastore().make<double>("ppcTd",0.0);
  ctl.datastore().make<Eigen::Vector6d>("ppcRhoInf", Eigen::Vector6d::Zero());
  ctl.datastore().make<double>("ppcKp",0.0);
  ctl.datastore().make<double>("ppcFilterDecay",0.0);

  ctl.datastore().make_call("getPolicyState", [this]() -> ctrl_states { return this->currentState; });

  // Init GUI
  ctl.gui()->addElement({"Controller","Policy"},
    mc_rtc::gui::Checkbox(
      "Start button pressed",
      [this](){ return start_btn; },
      [this]() {
        start_btn = !start_btn;
        std_msgs::Bool pub_msg;
        pub_msg.data = start_btn;
        start_btn_pub_.publish(pub_msg);
      }
    ),
    mc_rtc::gui::Checkbox(
      "Interact button pressed",
      [this](){ return interact_btn; },
      [this]() {
        interact_btn = !interact_btn;
        std_msgs::Bool pub_msg;
        pub_msg.data = interact_btn;
        interact_btn_pub_.publish(pub_msg);
      }
    )
  );

  // Init GUI
  ctl.gui()->addElement(
    {"Controller","Init State"},
    mc_rtc::gui::Button(
      "Set current posture as target",
      [&ctl]() {
        ctl.posture_target = ctl.robot().q();
        ctl.postureTask->posture(ctl.posture_target);
      }
    )
  );

  std::cout << "[MC RTC pHRIController] Policy initialized.\n";
}

bool humanLikeController_Policy::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);
  game_sub_.tick(controller_dt_);

  Eigen::Quaterniond quat = Eigen::Quaterniond(ctl.robot().frame("tool_frame").position().rotation());
  Eigen::Vector3d pose = ctl.robot().frame("tool_frame").position().translation();

  geometry_msgs::Pose pub_msg;
  pub_msg.position.x = pose.x();
  pub_msg.position.y = pose.y();
  pub_msg.position.z = pose.z();
  pub_msg.orientation.w = quat.w();
  pub_msg.orientation.x = quat.x();
  pub_msg.orientation.y = quat.y();
  pub_msg.orientation.z = quat.z();
  pose_pub_.publish(pub_msg);

  policy(ctl_);

  return false;
}

void humanLikeController_Policy::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  // Remove GUI elements
  ctl.gui()->removeCategory({"Controller","Policy"});
}

void humanLikeController_Policy::rosSpinner()
{
  mc_rtc::log::info("ROS spinner thread created");
  ros::Rate r(freq_);
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  mc_rtc::log::info("ROS spinner destroyed");
}

void humanLikeController_Policy::policy(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  MCGameState data = game_sub_.data().value();

  switch (currentState)
  {
  case Released:
    ctl.datastore().assign<std::string>("ControlMode","Position");
    // Begin interaction
    if(interact_btn){
      currentState = Contact;
    }
    break;
  
  case Contact:
    ctl.datastore().assign<std::string>("ControlMode","Torque");
    // Interact button released back to safety
    if(!interact_btn) {
      currentState = Released;
      break;
    }

    // Game not started yes
    if (data.status != "s") break;

    // If new task generated successfuly, change state to active interaction
    if(generateTask(ctl_)) currentState = Active;
    break;
  
  case Active:
    ctl.datastore().assign<std::string>("ControlMode","Torque");
    // Interact button released back to safety
    if(!interact_btn) {
      currentState = Released;
      break;
    }

    // If game stoped back to free motion
    if(data.status != "s") {
      currentState = Contact;
      break;
    }
    
    if (std::find_if(data.pieces.begin(),data.pieces.end(),[this](MCGamePiece & p){ return p.ID == tracked_piece.ID; }) == data.pieces.end())
    {
      currentState = Contact;
    }

    break;
  
  default:
    break;
  }
}

bool humanLikeController_Policy::generateTask(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PHRI_HLController &>(ctl_);

  MCGameState data = game_sub_.data().value();
  // If game not started, do nothing
  if(data.status == "") return false;
  // If no pieces available, do nothing
  if(data.pieces.size() == 0)
  {
    // mc_rtc::log::warning("No pieces available");
    return false;
  }
  // Target only pieces equal to 20 or 10
  std::vector<MCGamePiece> valid_pieces;
  std::copy_if(data.pieces.begin(), data.pieces.end(), std::back_inserter(valid_pieces),
    [this](MCGamePiece piece){
      double minTaskSpeed = abs(piece.distToCatcher)/piece.timeToCatcher;
      double Td = abs(piece.distToCatcher)/taskSpeed;
      // if(piece.value >= 10) std::cout << "Value = " << piece.value << " TD = " << Td << " minTaskSpeed = " << minTaskSpeed << std::endl;
      return piece.value >= 10 and minTaskSpeed < taskSpeed and Td > minTaskTime; }
  );
  // If no valid pieces, do nothing
  if(valid_pieces.size() == 0)
  {
    // mc_rtc::log::warning("No valid pieces available");
    return false;
  }
  std::sort(valid_pieces.begin(),valid_pieces.end(),[](MCGamePiece p1, MCGamePiece p2){ return p1.timeToCatcher < p2.timeToCatcher; });
  
  // Target soonest to fall
  tracked_piece = valid_pieces[0];

  // Set PPCTask parameters for usage in "Interaction" state
  sva::PTransformd targetPose = initPose;
  targetPose.translation() += Eigen::Vector3d(0,tracked_piece.distToCatcher,0);

  ctl.datastore().assign<sva::PTransformd>("ppcTargetPose",targetPose);
  ctl.datastore().assign<double>("ppcTd",abs(tracked_piece.distToCatcher/taskSpeed));
  ctl.datastore().assign<Eigen::Vector6d>("ppcRhoInf",rho_inf);
  ctl.datastore().assign<double>("ppcKp",kp);
  ctl.datastore().assign<double>("ppcFilterDecay",filter_decay);

  mc_rtc::log::info("Generated PPC task, reaching time {}[s]", abs(tracked_piece.distToCatcher/taskSpeed));

  return true;
}

EXPORT_SINGLE_STATE("Policy", humanLikeController_Policy)