/* includes //{ */

#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mutex>

#include <mavros_msgs/State.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <mrs_msgs/Vec4.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/MpcTrackerDiagnostics.h>
#include <mrs_msgs/TrajectoryReference.h>
#include <mrs_msgs/TrajectoryReferenceSrv.h>
#include <mrs_msgs/FutureTrajectory.h>
#include <mrs_msgs/ValidateReference.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/param_loader.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//}

/* defines //{ */

#define TARGET_NUM 1000
#define COLLISION_PENALTY 10.0
#define MISSED_TARGET_PENALTY 5.0

//}

namespace mrim_state_machine
{

/* class MrimStateMachine //{ */

// state machine
typedef enum
{

  IDLE_STATE,
  PLANNING_STATE,
  LOADING_STATE,
  VALIDATING_STATE,
  TAKEOFF_STATE,
  FLY_TO_START_STATE,
  FOLLOWING_STATE,
  LANDING_STATE

} State_t;

const char* state_names[8] = {

    "IDLING", "PLANNING TRAJECTORIES", "LOADING TRAJECTORIES", "VALIDATING CURRENT POSITION", "TAKING OFF", "FLYING TO START", "FOLLOWING", "LANDING"};

class MrimStateMachine : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized = false;

  int  service_call_repeat_;
  bool land_ = false;

  bool flying_finished_ = false;

  std::string master_uav_name_;

  bool start_position_1_valid = false;
  bool start_position_2_valid = false;

  // | --------------------- service clients -------------------- |
  ros::ServiceClient service_client_land_1;
  ros::ServiceClient service_client_land_2;

  ros::ServiceClient service_client_fly_to_start_1;
  ros::ServiceClient service_client_fly_to_start_2;

  ros::ServiceClient service_client_start_following_1;
  ros::ServiceClient service_client_start_following_2;

  ros::ServiceClient service_client_load_trajectory_1;
  ros::ServiceClient service_client_load_trajectory_2;

  ros::ServiceClient service_client_validate_start_position_1;
  ros::ServiceClient service_client_validate_start_position_2;
  ros::ServiceClient service_client_start_timer;
  // | --------------------- service servers -------------------- |
  ros::ServiceServer service_server_start;

  // | ----------------------- subscribers ---------------------- |
  ros::Subscriber subscriber_mavros_state;

  ros::Subscriber subscriber_control_manager_diagnostics_1;
  ros::Subscriber subscriber_control_manager_diagnostics_2;

  ros::Subscriber subscriber_mpc_diagnostics_1;
  ros::Subscriber subscriber_mpc_diagnostics_2;

  ros::Subscriber subscriber_trajectory_1;
  ros::Subscriber subscriber_trajectory_2;

  ros::Subscriber subscriber_odom_1;
  ros::Subscriber subscriber_odom_2;

  ros::Subscriber subscriber_trajectories_valid;

  // | ----------------------- publishers ----------------------- |
  ros::Publisher publisher_ready_to_takeoff_1;
  ros::Publisher publisher_ready_to_takeoff_2;

  // | ----------------------- main timer ----------------------- |
  ros::Timer main_timer;
  void       mainTimer(const ros::TimerEvent& event);
  double     main_timer_rate_;

  // | --------------------- Oneshot timers --------------------- |
  ros::Timer takeoff_timer_1;
  ros::Timer takeoff_timer_2;

  ros::Timer land_timer_1;
  ros::Timer land_timer_2;

  ros::Timer fly_to_start_timer_1;
  ros::Timer fly_to_start_timer_2;

  ros::Timer start_following_timer_1;
  ros::Timer start_following_timer_2;

  ros::Timer load_trajectory_timer_1;
  ros::Timer load_trajectory_timer_2;

  ros::Timer validate_start_position_timer_1;
  ros::Timer validate_start_position_timer_2;

  // | -------------------- MPC diagnostics 1 ------------------- |
  void                            callbackMpcDiagnostics1(const mrs_msgs::MpcTrackerDiagnosticsPtr& msg);
  std::mutex                      mutex_mpc_diagnostics_1;
  mrs_msgs::MpcTrackerDiagnostics mpc_diagnostics_1;
  bool                            got_mpc_diagnostics_1 = false;

  // | -------------------- MPC diagnostics 2 ------------------- |
  void                            callbackMpcDiagnostics2(const mrs_msgs::MpcTrackerDiagnosticsPtr& msg);
  std::mutex                      mutex_mpc_diagnostics_2;
  mrs_msgs::MpcTrackerDiagnostics mpc_diagnostics_2;
  bool                            got_mpc_diagnostics_2 = false;

  // | -------------------- Tracker status 1 ------------------- |
  void                                callbackControlManagerDiagnostics1(const mrs_msgs::ControlManagerDiagnosticsPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_1;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_1;
  bool                                got_control_manager_diagnostics_1 = false;

  // | -------------------- Tracker status 2 ------------------- |
  void                                callbackControlManagerDiagnostics2(const mrs_msgs::ControlManagerDiagnosticsPtr& msg);
  std::mutex                          mutex_control_manager_diagnostics_2;
  mrs_msgs::ControlManagerDiagnostics control_manager_diagnostics_2;
  bool                                got_control_manager_diagnostics_2 = false;

  // | ---------------------- Trajectory 1 ---------------------- |
  void                          callbackTrajectory1(const mrs_msgs::TrajectoryReferenceConstPtr& msg);
  bool                          got_trajectory_1    = false;
  bool                          trajectory_1_loaded = false;
  std::mutex                    mutex_trajectory_1;
  mrs_msgs::TrajectoryReference trajectory_1;

  // | ---------------------- Trajectory 2 ---------------------- |
  void                          callbackTrajectory2(const mrs_msgs::TrajectoryReferenceConstPtr& msg);
  bool                          got_trajectory_2    = false;
  bool                          trajectory_2_loaded = false;
  std::mutex                    mutex_trajectory_2;
  mrs_msgs::TrajectoryReference trajectory_2;

  // | ----------------------- Odometry 1 ----------------------- |
  void               callbackOdometry1(const nav_msgs::OdometryConstPtr& msg);
  bool               got_odometry_1 = false;
  std::mutex         mutex_odometry_1;
  nav_msgs::Odometry odometry_1;

  // | ----------------------- Odometry 2 ----------------------- |
  void               callbackOdometry2(const nav_msgs::OdometryConstPtr& msg);
  bool               got_odometry_2 = false;
  std::mutex         mutex_odometry_2;
  nav_msgs::Odometry odometry_2;

  // | --------------------- other callbacks -------------------- |
  void callbackTrajectoriesValidation(const std_msgs::Bool& msg);
  bool got_trajectory_validation = false;
  bool trajectories_valid        = false;

  // | --------------------- other callbacks -------------------- |
  bool callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  // | ------------------- state machine stuff ------------------ |
  State_t current_state = IDLE_STATE;
  void    switchState(State_t new_state);

  // | --------------------- Oneshot timers --------------------- |
  void validateStartPositionTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void takeoffTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void landTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void flyToStartTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void startFollowingTimerOneshot(const ros::TimerEvent& event, uint8_t id);
  void loadTrajectoryTimerOneshot(const ros::TimerEvent& event, uint8_t id);

  void validateStartPosition(void);
  void takeoff(void);
  void land(void);
  void flyToStart(void);
  void startFollowing(void);
  void loadTrajectories(void);

  bool StartPositionValid(void);
  bool MpcTrackerActive(void);
  bool MpcFlightInProgress(void);

  // | --------------------- score counting --------------------- |
  ros::Time start_time;
};
//}

/* onInit() //{ */

void MrimStateMachine::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh_, "MrimStateMachine");

  param_loader.loadParam("main_timer_rate", main_timer_rate_);
  param_loader.loadParam("service_call_repeat", service_call_repeat_);
  param_loader.loadParam("land", land_);
  param_loader.loadParam("master_uav_name", master_uav_name_);

  // --------------------------------------------------------------
  // |                         subscribers                        |
  // --------------------------------------------------------------

  subscriber_mpc_diagnostics_1 = nh_.subscribe("mpc_diagnostics_1_in", 1, &MrimStateMachine::callbackMpcDiagnostics1, this, ros::TransportHints().tcpNoDelay());
  subscriber_mpc_diagnostics_2 = nh_.subscribe("mpc_diagnostics_2_in", 1, &MrimStateMachine::callbackMpcDiagnostics2, this, ros::TransportHints().tcpNoDelay());

  subscriber_control_manager_diagnostics_1 =
      nh_.subscribe("control_manager_diagnostics_1_in", 1, &MrimStateMachine::callbackControlManagerDiagnostics1, this, ros::TransportHints().tcpNoDelay());
  subscriber_control_manager_diagnostics_2 =
      nh_.subscribe("control_manager_diagnostics_2_in", 1, &MrimStateMachine::callbackControlManagerDiagnostics2, this, ros::TransportHints().tcpNoDelay());

  subscriber_trajectory_1 = nh_.subscribe("trajectory_1_in", 1, &MrimStateMachine::callbackTrajectory1, this, ros::TransportHints().tcpNoDelay());
  subscriber_trajectory_2 = nh_.subscribe("trajectory_2_in", 1, &MrimStateMachine::callbackTrajectory2, this, ros::TransportHints().tcpNoDelay());

  subscriber_odom_1 = nh_.subscribe("odometry_1_in", 1, &MrimStateMachine::callbackOdometry1, this, ros::TransportHints().tcpNoDelay());
  subscriber_odom_2 = nh_.subscribe("odometry_2_in", 1, &MrimStateMachine::callbackOdometry2, this, ros::TransportHints().tcpNoDelay());

  subscriber_trajectories_valid =
      nh_.subscribe("trajectories_valid_in", 1, &MrimStateMachine::callbackTrajectoriesValidation, this, ros::TransportHints().tcpNoDelay());

  // --------------------------------------------------------------
  // |                         publishers                         |
  // --------------------------------------------------------------

  publisher_ready_to_takeoff_1 = nh_.advertise<std_msgs::UInt8>("takeoff_1_out", 1);
  publisher_ready_to_takeoff_2 = nh_.advertise<std_msgs::UInt8>("takeoff_2_out", 1);

  // --------------------------------------------------------------
  // |                       service clients                      |
  // --------------------------------------------------------------

  service_client_land_1 = nh_.serviceClient<std_srvs::Trigger>("land_1_out");
  service_client_land_2 = nh_.serviceClient<std_srvs::Trigger>("land_2_out");

  service_client_fly_to_start_1 = nh_.serviceClient<std_srvs::Trigger>("fly_to_start_1_out");
  service_client_fly_to_start_2 = nh_.serviceClient<std_srvs::Trigger>("fly_to_start_2_out");

  service_client_start_following_1 = nh_.serviceClient<std_srvs::Trigger>("start_following_1_out");
  service_client_start_following_2 = nh_.serviceClient<std_srvs::Trigger>("start_following_2_out");

  service_client_load_trajectory_1 = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("load_trajectory_1_out");
  service_client_load_trajectory_2 = nh_.serviceClient<mrs_msgs::TrajectoryReferenceSrv>("load_trajectory_2_out");

  service_client_validate_start_position_1 = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_start_position_1_out");
  service_client_validate_start_position_2 = nh_.serviceClient<mrs_msgs::ValidateReference>("validate_start_position_2_out");

  service_client_start_timer = nh_.serviceClient<std_srvs::SetBool>("start_out");

  // --------------------------------------------------------------
  // |                       service servers                      |
  // --------------------------------------------------------------

  service_server_start = nh_.advertiseService("start_in", &MrimStateMachine::callbackStart, this);

  // --------------------------------------------------------------
  // |                           timers                           |
  // --------------------------------------------------------------

  main_timer = nh_.createTimer(ros::Rate(main_timer_rate_), &MrimStateMachine::mainTimer, this);

  load_trajectory_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::loadTrajectoryTimerOneshot, this, _1, 1), true, false);
  load_trajectory_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::loadTrajectoryTimerOneshot, this, _1, 2), true, false);

  start_following_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::startFollowingTimerOneshot, this, _1, 1), true, false);
  start_following_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::startFollowingTimerOneshot, this, _1, 2), true, false);

  fly_to_start_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::flyToStartTimerOneshot, this, _1, 1), true, false);
  fly_to_start_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::flyToStartTimerOneshot, this, _1, 2), true, false);

  land_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::landTimerOneshot, this, _1, 1), true, false);
  land_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::landTimerOneshot, this, _1, 2), true, false);

  takeoff_timer_1 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::takeoffTimerOneshot, this, _1, 1), true, false);
  takeoff_timer_2 = nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::takeoffTimerOneshot, this, _1, 2), true, false);

  validate_start_position_timer_1 =
      nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::validateStartPositionTimerOneshot, this, _1, 1), true, false);
  validate_start_position_timer_2 =
      nh_.createTimer(ros::Duration(0), boost::bind(&MrimStateMachine::validateStartPositionTimerOneshot, this, _1, 2), true, false);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[MavrosInterface]: Could not load all parameters!");
    ros::shutdown();
  }

  is_initialized = true;

  ROS_INFO("[MrimStateMachine]: initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackTrajectoriesValidation() //{ */

void MrimStateMachine::callbackTrajectoriesValidation(const std_msgs::Bool& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MrimStateMachine]: getting trajectories validation info.");

  got_trajectory_validation = true;

  trajectories_valid = msg.data;
}

//}

/* callbackMpcDiagnostics1() //{ */

void MrimStateMachine::callbackMpcDiagnostics1(const mrs_msgs::MpcTrackerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MrimStateMachine]: getting mpc diagnostics (1)");

  std::scoped_lock lock(mutex_mpc_diagnostics_1);

  got_mpc_diagnostics_1 = true;

  mpc_diagnostics_1 = *msg;
}

//}

/* callbackMpcDiagnostics2() //{ */

void MrimStateMachine::callbackMpcDiagnostics2(const mrs_msgs::MpcTrackerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MrimStateMachine]: getting mpc diagnostics (2)");

  std::scoped_lock lock(mutex_mpc_diagnostics_2);

  got_mpc_diagnostics_2 = true;

  mpc_diagnostics_2 = *msg;
}

//}

/* callbackControlManagerDiagnostics1() //{ */

void MrimStateMachine::callbackControlManagerDiagnostics1(const mrs_msgs::ControlManagerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MrimStateMachine]: getting control manager diagnostics (1)");

  std::scoped_lock lock(mutex_control_manager_diagnostics_1);

  got_control_manager_diagnostics_1 = true;

  control_manager_diagnostics_1 = *msg;
}

//}

/* callbackControlManagerDiagnostics2() //{ */

void MrimStateMachine::callbackControlManagerDiagnostics2(const mrs_msgs::ControlManagerDiagnosticsPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MrimStateMachine]: getting control manager diagnostics (2)");

  std::scoped_lock lock(mutex_control_manager_diagnostics_2);

  got_control_manager_diagnostics_2 = true;

  control_manager_diagnostics_2 = *msg;
}

//}

/* callbackTrajectory1() //{ */

void MrimStateMachine::callbackTrajectory1(const mrs_msgs::TrajectoryReferenceConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO("[MrimStateMachine]: received trajectory (1)");

  std::scoped_lock lock(mutex_trajectory_1);

  trajectory_1 = *msg;

  got_trajectory_1 = true;
}

//}

/* callbackTrajectory2() //{ */

void MrimStateMachine::callbackTrajectory2(const mrs_msgs::TrajectoryReferenceConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO("[MrimStateMachine]: received trajectory (2)");

  std::scoped_lock lock(mutex_trajectory_2);

  trajectory_2 = *msg;

  got_trajectory_2 = true;
}

//}

/* callbackOdometry1() //{ */

void MrimStateMachine::callbackOdometry1(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MrimStateMachine]: receiving odometry (1)");

  std::scoped_lock lock(mutex_odometry_2);

  odometry_1 = *msg;

  got_odometry_1 = true;
}

//}

/* callbackOdometry2() //{ */

void MrimStateMachine::callbackOdometry2(const nav_msgs::OdometryConstPtr& msg) {

  if (!is_initialized) {
    return;
  }

  ROS_INFO_ONCE("[MrimStateMachine]: receiving odometry (2)");

  std::scoped_lock lock(mutex_odometry_2);

  odometry_2 = *msg;

  got_odometry_2 = true;
}

//}

/* callbackStart() //{ */

bool MrimStateMachine::callbackStart([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!got_odometry_1 || !got_odometry_2) {

    res.success = false;
    res.message = "missing odometries";

    return true;
  }

  switchState(PLANNING_STATE);

  res.success = true;
  res.message = "started";

  return true;
}

//}

// --------------------------------------------------------------
// |                        main routines                       |
// --------------------------------------------------------------

/* MpcTrackerActive() //{ */

bool MrimStateMachine::MpcTrackerActive(void) {

  std::scoped_lock lock(mutex_mpc_diagnostics_1, mutex_mpc_diagnostics_2);

  if (!got_mpc_diagnostics_1 || !got_mpc_diagnostics_2) {
    return false;
  }

  if (mpc_diagnostics_1.active && mpc_diagnostics_2.active) {
    return true;
  }

  return false;
}

//}

/* MpcFlightInProgress() //{ */

bool MrimStateMachine::MpcFlightInProgress(void) {

  std::scoped_lock lock(mutex_mpc_diagnostics_1, mutex_mpc_diagnostics_2);

  if (control_manager_diagnostics_1.tracker_status.have_goal || control_manager_diagnostics_2.tracker_status.have_goal) {
    return true;
  }

  return false;
}

//}

// | --------------------- oneshot timers --------------------- |

/* validateStartPositionTimerOneshot() //{ */

void MrimStateMachine::validateStartPositionTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {
  mrs_msgs::ValidateReference ref_out;
  ref_out.request.reference.header.frame_id = "fcu";

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_validate_start_position_1.call(ref_out);
    } else {
      service_client_validate_start_position_2.call(ref_out);
    }

    if (!ref_out.response.success) {

      ROS_ERROR("[MrimStateMachine]: current uav position (%d) is not valid: %s", id, ref_out.response.message.c_str());

    } else {

      if (id == 1) {
        start_position_1_valid = true;
      } else {
        start_position_2_valid = true;
      }
      ROS_INFO("[MrimStateMachine]: current uav position (%d) is valid", id);
      break;
    }
  }
}

//}

/* takeoffTimerOneshot() //{ */

void MrimStateMachine::takeoffTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  std_msgs::UInt8 ready_out;
  ready_out.data = id;

  ROS_INFO("[MrimStateMachine]: vehicle (%d) ready for takeoff", id);

  while (ros::ok()) {

    if (id == 1) {
      std::scoped_lock lock(mutex_mpc_diagnostics_1);
      publisher_ready_to_takeoff_1.publish(ready_out);
      if (mpc_diagnostics_1.active) {
        ROS_INFO("[MrimStateMachine]: takeoff (%d) successful", id);
        break;
      }
    } else {
      std::scoped_lock lock(mutex_mpc_diagnostics_2);
      publisher_ready_to_takeoff_2.publish(ready_out);
      if (mpc_diagnostics_2.active) {
        ROS_INFO("[MrimStateMachine]: takeoff (%d) successful", id);
        break;
      }
    }
    ros::Duration(0.2).sleep();
    ros::spinOnce();
  }

  std::scoped_lock lock(mutex_mpc_diagnostics_1, mutex_mpc_diagnostics_2);
  if (id == 1 && !mpc_diagnostics_1.active) {
    ROS_ERROR("[MrimStateMachine]: takeoff (%d) failed", id);
  }
  if (id == 2 && !mpc_diagnostics_2.active) {
    ROS_ERROR("[MrimStateMachine]: takeoff (%d) failed", id);
  }
}

//}

/* landTimerOneshot() //{ */

void MrimStateMachine::landTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  std_srvs::Trigger srv_out;

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_land_1.call(srv_out);
    } else {
      service_client_land_2.call(srv_out);
    }

    if (!srv_out.response.success) {

      ROS_ERROR("[MrimStateMachine]: call for land (%d) failed: %s", id, srv_out.response.message.c_str());
    } else {

      ROS_INFO("[MrimStateMachine]: landing (%d)", id);
      break;
    }
  }
}

//}

/* flyToStartTimerOneshot() //{ */

void MrimStateMachine::flyToStartTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  std_srvs::Trigger srv_out;

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_fly_to_start_1.call(srv_out);
    } else {
      service_client_fly_to_start_2.call(srv_out);
    }

    if (!srv_out.response.success) {

      ROS_ERROR("[MrimStateMachine]: call for flying to start (%d) failed: %s", id, srv_out.response.message.c_str());
    } else {

      ROS_INFO("[MrimStateMachine]: flying to start (%d)", id);
      break;
    }
  }
}

//}

/* startFollowingTimerOneshot() //{ */

void MrimStateMachine::startFollowingTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  std_srvs::Trigger srv_out;

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_start_following_1.call(srv_out);
    } else {
      service_client_start_following_2.call(srv_out);
    }

    if (!srv_out.response.success) {

      ROS_ERROR("[MrimStateMachine]: call for start following (%d) failed: %s", id, srv_out.response.message.c_str());
    } else {

      ROS_INFO("[MrimStateMachine]: started following (%d)", id);
      break;
    }
  }
}

//}

/* loadTrajectoryTimerOneshot() //{ */

void MrimStateMachine::loadTrajectoryTimerOneshot([[maybe_unused]] const ros::TimerEvent& event, uint8_t id) {

  mrs_msgs::TrajectoryReferenceSrv srv_out;
  if (id == 1) {
    srv_out.request.trajectory = trajectory_1;
  } else {
    srv_out.request.trajectory = trajectory_2;
  }

  for (int i = 0; i < service_call_repeat_; i++) {

    if (id == 1) {
      service_client_load_trajectory_1.call(srv_out);
    } else {
      service_client_load_trajectory_2.call(srv_out);
    }

    if (!srv_out.response.success) {
      // returned false because the MPC tracker is not currently active

      for (unsigned int i = 0; i < srv_out.response.tracker_names.size(); i++) {
        if (srv_out.response.tracker_names[i] == "MpcTracker") {
          if (srv_out.response.tracker_successes[i]) {
            // trajectory is feasible for the MPC tracker
            goto loaded;
          }
        }
      }
      ROS_ERROR("[MrimStateMachine]: call for loading trajectory (%d) failed", id);
    } else {
    loaded:

      ROS_INFO("[MrimStateMachine]: trajectory loaded (%d)", id);

      if (id == 1) {
        got_trajectory_1    = false;
        trajectory_1_loaded = true;
      } else {
        got_trajectory_2    = false;
        trajectory_2_loaded = true;
      }

      break;
    }
  }
}

//}

// | -------------------- control services -------------------- |

/* validateStartPosition //{ */

void MrimStateMachine::validateStartPosition(void) {
  // get validation of the trajectory first point from the evaluator
  validate_start_position_timer_1.stop();
  validate_start_position_timer_2.stop();

  validate_start_position_timer_1.start();
  validate_start_position_timer_2.start();
}

//}


/* takeoff() //{ */

void MrimStateMachine::takeoff(void) {

  takeoff_timer_1.stop();
  takeoff_timer_2.stop();

  takeoff_timer_1.start();
  takeoff_timer_2.start();
}

//}

/* land() //{ */

void MrimStateMachine::land(void) {

  land_timer_1.stop();
  land_timer_2.stop();

  land_timer_1.start();
  land_timer_2.start();
}

//}

/* flyToStart() //{ */

void MrimStateMachine::flyToStart(void) {

  fly_to_start_timer_1.stop();
  fly_to_start_timer_2.stop();

  fly_to_start_timer_1.start();
  fly_to_start_timer_2.start();
}

//}

/* startFollowing() //{ */

void MrimStateMachine::startFollowing(void) {

  start_following_timer_1.stop();
  start_following_timer_2.stop();

  start_following_timer_1.start();
  start_following_timer_2.start();
}

//}

/* loadTrajectories() //{ */

void MrimStateMachine::loadTrajectories(void) {

  load_trajectory_timer_1.stop();
  load_trajectory_timer_2.stop();

  load_trajectory_timer_1.start();
  load_trajectory_timer_2.start();
}

//}

// | ---------------------- state machine --------------------- |

/* switchState() //{ */

void MrimStateMachine::switchState(State_t new_state) {

  ROS_WARN("[MrimStateMachine]: switching state from %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

    case IDLE_STATE: {

      got_trajectory_1 = false;
      got_trajectory_2 = false;

      got_mpc_diagnostics_1 = false;
      got_mpc_diagnostics_2 = false;

      break;
    }

    case PLANNING_STATE: {

      break;
    }

    case LOADING_STATE: {

      loadTrajectories();

      break;
    }

    case VALIDATING_STATE: {

      validateStartPosition();

      break;
    }
    case TAKEOFF_STATE: {

      takeoff();

      break;
    }

    case FLY_TO_START_STATE: {

      flyToStart();

      break;
    }

    case FOLLOWING_STATE: {

      startFollowing();

      for (int i = 0; i < service_call_repeat_; i++) {

        std_srvs::SetBool srv;
        srv.request.data = true;
        if (service_client_start_timer.call(srv)) {
          ROS_INFO("[MrimStateMachine]: Mission timer started.");
          break;
        } else {
          ROS_ERROR("[MrimStateMachine]: Call for mission timer start fail.");
        }
      }

      start_time = ros::Time::now();

      break;
    }

    case LANDING_STATE: {

      land();

      break;
    }

    break;
  }

  ros::Duration sleep(1.0);
  sleep.sleep();

  current_state = new_state;
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* mainTimer() //{ */

void MrimStateMachine::mainTimer([[maybe_unused]] const ros::TimerEvent& event) {

  ROS_INFO_THROTTLE(10.0, "[MrimStateMachine]: Current state: %s", state_names[current_state]);
  switch (current_state) {

    case IDLE_STATE: {

      if (!flying_finished_) {
        switchState(PLANNING_STATE);
      }

      break;
    }

    case PLANNING_STATE: {

      if (got_trajectory_1 && got_trajectory_2 && got_trajectory_validation) {

        if (trajectories_valid) {
          switchState(LOADING_STATE);
        } else {
          ROS_ERROR("[MrimStateMachine]: The trajectories are not valid - check output of trajectory checker!");
          switchState(IDLE_STATE);
        }

      } else {
        ROS_WARN_THROTTLE(10.0, "[MrimStateMachine]: Missing data: tajectory 1 obtained = %d, trajectory 2 obtained = %d, trajectory validation = %d",
                          got_trajectory_1, got_trajectory_2, got_trajectory_validation);
      }

      break;
    }

    case LOADING_STATE: {

      if (trajectory_1_loaded && trajectory_2_loaded) {

        if (MpcTrackerActive()) {
          switchState(FLY_TO_START_STATE);
        } else {
          switchState(VALIDATING_STATE);
        }
      }

      break;
    }

    case VALIDATING_STATE: {

      if (start_position_1_valid && start_position_2_valid) {
        switchState(TAKEOFF_STATE);
      } else {
        ROS_ERROR("[MrimStateMachine]: Invalid uav position, cannot takeoff!");
        switchState(IDLE_STATE);
      }

      break;
    }

    case TAKEOFF_STATE: {

      if (MpcTrackerActive() && !MpcFlightInProgress()) {

        switchState(FLY_TO_START_STATE);
      }

      break;
    }

    case FLY_TO_START_STATE: {

      if (!MpcFlightInProgress()) {

        switchState(FOLLOWING_STATE);
      }

      break;
    }

    case FOLLOWING_STATE: {

      if (!MpcFlightInProgress()) {

        flying_finished_ = true;

        for (int i = 0; i < service_call_repeat_; i++) {

          std_srvs::SetBool srv;
          srv.request.data = false;
          if (service_client_start_timer.call(srv)) {
            ROS_INFO("[MrimStateMachine]: Mission timer stopped.");
            break;
          } else {
            ROS_ERROR("[MrimStateMachine]: Call for mission timer stop fail.");
          }
        }

        if (land_) {
          switchState(LANDING_STATE);
        } else {
          switchState(IDLE_STATE);
        }
      }
      {
        if (got_trajectory_1 && got_trajectory_2) {

          switchState(LOADING_STATE);
        }
      }

      break;
    }

    case LANDING_STATE: {

      switchState(IDLE_STATE);

      break;
    }

    break;
  }
}  // namespace mtsp_state_machine

//}

}  // namespace mrim_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrim_state_machine::MrimStateMachine, nodelet::Nodelet)
