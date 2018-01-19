#ifndef PAL_FSP_GUI_H
#define PAL_FSP_GUI_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <pal_footstep_planner_msgs/PlanWalkAction.h>
#include <pal_footstep_planner_msgs/ExecuteWalkAction.h>
#include <pal_ros_utils/reference/pose/pose_interactive_marker_reference.h>

#include "ui_pal_fsp_gui.h"
#include <QWidget>

#include <memory>

namespace pal
{
class PalFSPGui : public QWidget
{
  Q_OBJECT
public:
  PalFSPGui(QWidget* parent = 0);
  virtual ~PalFSPGui()
  {
  }

  void init(ros::NodeHandle& nh, ros::NodeHandle& private_nh, QWidget* container);

private:
  typedef actionlib::SimpleActionClient<pal_footstep_planner_msgs::PlanWalkAction> FSPClient;
  typedef actionlib::SimpleActionClient<pal_footstep_planner_msgs::ExecuteWalkAction> EWClient;

  Ui::FSPWidget ui_;
  std::unique_ptr<FSPClient> fsp_client_;
  std::unique_ptr<EWClient> ew_client_;
  pal_robot_tools::InteractiveMakerReferencePtr marker_;
  ros::Publisher marker_pub_;
  ros::Subscriber hint_sub_;
  std::vector<pal_footstep_planner_msgs::FootstepData> path_;
  std::string frame_id_;

  pal_footstep_planner_msgs::PlanWalkGoal createGoal(bool check_collisions, bool replan);
  bool createGoal(pal_footstep_planner_msgs::ExecuteWalkGoal* goal, bool replan);
  void onGoalSucceeded(const actionlib::SimpleClientGoalState& state,
                       const pal_footstep_planner_msgs::PlanWalkResultConstPtr& result);
  void onGoalExecSucceeded(const actionlib::SimpleClientGoalState& state,
                           const pal_footstep_planner_msgs::ExecuteWalkResultConstPtr& result);
  void changeState(bool active, bool keep_execute = false);
  void hintCb(const geometry_msgs::PoseConstPtr& pose);

private slots:
  void onPlan();
  void onReplan();
  void onExecute();
  void onDimensionChange(bool checked);
};
}

#endif /* PAL_FSP_GUI_H */
