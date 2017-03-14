#include <pal_fsp_gui/pal_fsp_gui.h>

#include <pal_robot_tools/conversions.h>
#include <visualization_msgs/MarkerArray.h>

namespace pal
{
PalFSPGui::PalFSPGui(QWidget *parent) : QWidget(parent)
{
}

void PalFSPGui::init(ros::NodeHandle &nh, QWidget *container)
{
  ui_.setupUi(container);

  connect(ui_.plan_btn_, SIGNAL(pressed()), this, SLOT(onPlan()));
  connect(ui_.replan_btn_, SIGNAL(pressed()), this, SLOT(onReplan()));
  connect(ui_.execute_btn_, SIGNAL(pressed()), this, SLOT(onExecute()));

  eVector3 init_marker(0, 0, 0);
  marker_.reset(new InteractiveMakerReference(nh, "Goal", "world", init_marker,
                                              Eigen::Quaterniond::Identity()));

  fsp_client_.reset(new FSPClient(nh, "plan_walk"));

  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
}

pal_footstep_planner_msgs::PlanWalkGoal PalFSPGui::createGoal(bool replan)
{
  pal_footstep_planner_msgs::PlanWalkGoal goal;
  goal.replan = replan;
  ROS_INFO_STREAM((replan ? "REPLAN" : "PLAN") << " ACTION:");

  // Load interactive marker values
  pal::convert(marker_->getPositionTarget(), goal.target_pose.position);
  ROS_INFO_STREAM("POSITION: \n" << goal.target_pose.position);
  pal::convert(marker_->getOrientationTarget(), goal.target_pose.orientation);
  ROS_INFO_STREAM("ORIENTATION: \n" << goal.target_pose.orientation);

  return goal;
}

void PalFSPGui::onGoalSucceeded(const actionlib::SimpleClientGoalState &state,
                                const pal_footstep_planner_msgs::PlanWalkResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    visualization_msgs::MarkerArray markers;
    // Loop over the results creating markers
    for (unsigned int i = 0; i < result->footsteps.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time::now();
      marker.ns = "walk_path";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      // ORANGE color
      marker.color.r = 0.7f;
      marker.color.g = 0.3f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0;

      // Set Scale
      marker.scale.x = 1.0;
      marker.scale.y = 1.0;
      marker.scale.z = 1.0;
      marker.lifetime = ros::Duration();

      // Retrieve the pose
      marker.pose.position.x = result->footsteps[i].location.x;
      marker.pose.position.y = result->footsteps[i].location.y;
      marker.pose.position.z = result->footsteps[i].location.z;
      marker.pose.orientation = result->footsteps[i].orientation;

      markers.markers.push_back(marker);
    }

    // visualize the results
    marker_pub_.publish(markers);
  }
  else
  {
    ROS_ERROR_STREAM("Action " << state.toString() << ": " << state.getText());
  }

  changeState(true);
}

void PalFSPGui::changeState(bool active)
{
  ui_.plan_btn_->setEnabled(active);
  ui_.replan_btn_->setEnabled(active);
  ui_.execute_btn_->setEnabled(active);
}

void PalFSPGui::onPlan()
{
  pal_footstep_planner_msgs::PlanWalkGoal goal = createGoal(false);
  fsp_client_->sendGoal(goal, boost::bind(&PalFSPGui::onGoalSucceeded, this, _1, _2));
  changeState(false);
}

void PalFSPGui::onReplan()
{
  pal_footstep_planner_msgs::PlanWalkGoal goal = createGoal(true);
  fsp_client_->sendGoal(goal, boost::bind(&PalFSPGui::onGoalSucceeded, this, _1, _2));
  changeState(false);
}

void PalFSPGui::onExecute()
{
  ROS_INFO("Execute!");
}
}
