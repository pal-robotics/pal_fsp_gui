#ifndef PAL_FSP_GUI_H
#define PAL_FSP_GUI_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <pal_footstep_planner_msgs/PlanWalkAction.h>
#include <pal_robot_tools/reference/interactive_marker_reference.h>

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

  void init(ros::NodeHandle& nh, QWidget* container);

private:
  typedef actionlib::SimpleActionClient<pal_footstep_planner_msgs::PlanWalkAction> FSPClient;

  Ui::FSPWidget ui_;
  std::unique_ptr<FSPClient> fsp_client_;
  InteractiveMakerReferencePtr marker_;
  ros::Publisher marker_pub_;

  pal_footstep_planner_msgs::PlanWalkGoal createGoal(bool replan);
  void onGoalSucceeded(const actionlib::SimpleClientGoalState& state,
                       const pal_footstep_planner_msgs::PlanWalkResultConstPtr& result);
  void changeState(bool active);

private slots:
  void onPlan();
  void onReplan();
  void onExecute();
};
}

#endif /* PAL_FSP_GUI_H */
