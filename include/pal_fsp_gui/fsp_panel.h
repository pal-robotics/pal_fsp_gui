#ifndef FSP_PANEL_H
#define FSP_PANEL_H

#include <rviz/panel.h>

#include <pal_fsp_gui/pal_fsp_gui.h>

namespace pal {

class FspPanel: public rviz::Panel
{
  Q_OBJECT
public:
  FspPanel(QWidget* parent = 0);

private:
  PalFSPGui gui_;
  ros::NodeHandle nh_;
};

}

#endif /* FSP_PANEL_H */
