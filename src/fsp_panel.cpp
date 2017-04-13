#include <pal_fsp_gui/fsp_panel.h>

#include <pluginlib/class_list_macros.h>

namespace pal
{
FspPanel::FspPanel(QWidget *parent) : rviz::Panel(parent), gui_(parent), nh_(), private_nh_("~")
{
  setObjectName("FootstepPanel");
  gui_.init(nh_, private_nh_, this);
}
}

PLUGINLIB_EXPORT_CLASS(pal::FspPanel, rviz::Panel)
