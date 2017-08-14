#include <pal_fsp_gui/pal_fsp_gui.h>

#include <pal_robot_tools/conversions.h>
#include <visualization_msgs/MarkerArray.h>

namespace pal
{
PalFSPGui::PalFSPGui(QWidget *parent) : QWidget(parent), frame_id_("map")
{
}

void PalFSPGui::init(ros::NodeHandle &nh, ros::NodeHandle &private_nh, QWidget *container)
{
  ui_.setupUi(container);

  connect(ui_.plan_btn_, SIGNAL(pressed()), this, SLOT(onPlan()));
  connect(ui_.replan_btn_, SIGNAL(pressed()), this, SLOT(onReplan()));
  connect(ui_.execute_btn_, SIGNAL(pressed()), this, SLOT(onExecute()));
  connect(ui_.dimension_, SIGNAL(clicked(bool)), this, SLOT(onDimensionChange(bool)));

  // Retrieve base frame
  std::string frame_id_name;
  if (private_nh.searchParam("marker_frame", frame_id_name))
  {
    private_nh.param<std::string>(frame_id_name, frame_id_, "map");
  }
  else
  {
    ROS_WARN("Could not find param marker_frame");
    private_nh.param<std::string>("/marker_frame", frame_id_, "map");
  }
  ROS_INFO_STREAM("Marker will appear on frame: " << frame_id_);

  eVector3 init_marker(0, 0, 0);
  marker_.reset(new pal_robot_tools::InteractiveMakerReference(
      nh, "Goal", frame_id_, init_marker, Eigen::Quaterniond::Identity()));

  fsp_client_.reset(new FSPClient(nh, "plan_walk"));
  ew_client_.reset(new EWClient(nh, "execute_walk"));

  marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("visualize_path", 1);
  hint_sub_ = nh.subscribe("footstep_marker_hint", 1, &PalFSPGui::hintCb, this);
}

pal_footstep_planner_msgs::PlanWalkGoal PalFSPGui::createGoal(bool check_collisions, bool replan)
{
  pal_footstep_planner_msgs::PlanWalkGoal goal;
  goal.replan = replan;
  goal.check_collision = check_collisions;
  ROS_INFO_STREAM((replan ? "REPLAN" : "PLAN") << " ACTION:");

  // Load interactive marker values
  pal::convert(marker_->getPositionTarget(), goal.target_pose.position);
  ROS_INFO_STREAM("POSITION: \n" << goal.target_pose.position);
  pal::convert(marker_->getOrientationTarget(), goal.target_pose.orientation);
  ROS_INFO_STREAM("ORIENTATION: \n" << goal.target_pose.orientation);

  return goal;
}

bool PalFSPGui::createGoal(pal_footstep_planner_msgs::ExecuteWalkGoal *goal)
{
  if (path_.size() != 0)
  {
    goal->path = path_;
    goal->replan = true;
  }
  else
  {
    ROS_ERROR("No path calculated");
    return false;
  }

  return true;
}

void PalFSPGui::onGoalSucceeded(const actionlib::SimpleClientGoalState &state,
                                const pal_footstep_planner_msgs::PlanWalkResultConstPtr &result)
{
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    visualization_msgs::MarkerArray markers;
    // Clear all markers
    visualization_msgs::Marker marker_clear;
    marker_clear.action = 3;  // Delete all
    markers.markers.push_back(marker_clear);

    // Loop over the results creating markers
    for (unsigned int i = 0; i < result->footsteps.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = ros::Time::now();
      marker.ns = "walk_path";
      marker.id = i;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.action = visualization_msgs::Marker::ADD;

      // ORANGE color
      if (result->footsteps[i].robot_side == pal_footstep_planner_msgs::FootstepData::LEFT)
      {
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8;
        marker.ns = "left_foot_moving";
      }
      else
      {
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.8;
        marker.ns = "right_foot_moving";
      }

      // Set Scale
      marker.scale.x = 0.2;
      marker.scale.y = 0.15;
      marker.scale.z = 0.025;
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

    // Store path
    path_ = result->footsteps;
    ui_.display_->setText("Planning Successful");
  }
  else
  {
    ROS_ERROR_STREAM("Action " << state.toString() << ": " << state.getText());
    ui_.display_->setText(QString::fromStdString("Planning Failed: " + state.getText()));
  }

  changeState(true);
}

void PalFSPGui::onGoalExecSucceeded(const actionlib::SimpleClientGoalState &state,
                                    const pal_footstep_planner_msgs::ExecuteWalkResultConstPtr &result)
{
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED || !result->success)
  {
    ROS_ERROR_STREAM("Execute walk " << state.toString() << ": "
                                     << (result->success ? "Success" : "No Success"));
    ui_.display_->setText(QString::fromStdString("Execution Failed: " + state.getText()));
  }
  else
  {
    ui_.display_->setText("Execution Successful");
  }
  ui_.execute_btn_->setText("Execute");
  changeState(true);
}

void PalFSPGui::changeState(bool active, bool keep_execute)
{
  ui_.plan_btn_->setEnabled(active);
  ui_.replan_btn_->setEnabled(active);
  if (!keep_execute)
  {
    ui_.execute_btn_->setEnabled(active);
  }
}

void PalFSPGui::hintCb(const geometry_msgs::PoseConstPtr &pose)
{
  ROS_INFO_STREAM("Got pose hint");
  eVector3 position;
  eQuaternion rotation;
  convert(pose->position, position);
  convert(pose->orientation, rotation);
  marker_->setPoseTarget(createMatrix(rotation, position));
}

void PalFSPGui::onPlan()
{
  pal_footstep_planner_msgs::PlanWalkGoal goal = createGoal(ui_.collision_->isChecked(), false);
  fsp_client_->sendGoal(goal, boost::bind(&PalFSPGui::onGoalSucceeded, this, _1, _2));
  ui_.display_->setText("Planning ...");
  changeState(false);
}

void PalFSPGui::onReplan()
{
  pal_footstep_planner_msgs::PlanWalkGoal goal = createGoal(ui_.collision_->isChecked(), true);
  fsp_client_->sendGoal(goal, boost::bind(&PalFSPGui::onGoalSucceeded, this, _1, _2));
  ui_.display_->setText("Replanning ...");
  changeState(false);
}

void PalFSPGui::onExecute()
{
  if (ui_.execute_btn_->text().toStdString() == "Execute")
  {
    pal_footstep_planner_msgs::ExecuteWalkGoal goal;
    goal.replan = ui_.cont_replan_->isChecked();
    if (createGoal(&goal))
    {
      ew_client_->sendGoal(goal, boost::bind(&PalFSPGui::onGoalExecSucceeded, this, _1, _2));
      changeState(false, true);  // Do not block execute button
      ui_.display_->setText("Executing ...");
      ui_.execute_btn_->setText("Cancel");

      //Remove current path
      path_.clear();
    }
  }
  else
  {
    ew_client_->cancelGoal();
    ui_.display_->setText("Goal canceled by user");
    ui_.execute_btn_->setText("Execute");
    changeState(true);
  }
}

void PalFSPGui::onDimensionChange(bool checked)
{
  if (checked)
  {
    marker_->set2DMode();
  }
  else
  {
    marker_->set3DMode();
  }
}
}
