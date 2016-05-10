/*  v4l2ucp - A universal control panel for all V4L2 devices
    Copyright (C) 2005,2009 Scott J. Bertin (scottbertin@yahoo.com)
    Copyright (C) 2009-2010 Vasily Khoruzhick (anarsoul@gmail.com)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 */

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <libv4l2.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sys/ioctl.h>

#include "v4l2ucp/mainWindow.h"

MainWindow::MainWindow() :
  fd(-1)
{
  std::string device = "/dev/video0";
  ros::param::get("~device", device);

  fd = v4l2_open(device.c_str(), O_RDWR, 0);
  if (fd < 0)
  {
    ROS_ERROR_STREAM("v4l2ucp: Unable to open file" << device << " "
        << strerror(errno));
    return;
  }

  struct v4l2_capability cap;
  if (v4l2_ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
  {
    ROS_ERROR_STREAM("v4l2ucp: Not a V4L2 device" << device);
    return;
  }

  ROS_INFO_STREAM(cap.driver);
  ROS_INFO_STREAM(cap.card);
  ROS_INFO_STREAM(cap.bus_info);

  ROS_INFO_STREAM((cap.version >> 16) << "." << ((cap.version >> 8) & 0xff) << "."
              << (cap.version & 0xff));

  ROS_INFO_STREAM("0x" << std::hex << cap.capabilities);

  configured_pub_ = nh_.advertise<std_msgs::Empty>("configured", 1, true);

  struct v4l2_queryctrl ctrl;
#ifdef V4L2_CTRL_FLAG_NEXT_CTRL
  /* Try the extended control API first */
  ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  if (0 == v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl))
  {
    do
    {
      add_control(ctrl, fd);
      ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    while (0 == v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl));
  }
  else
#endif
  {
    /* Fall back on the standard API */
    /* Check all the standard controls */
    for (int i = V4L2_CID_BASE; i < V4L2_CID_LASTP1; i++)
    {
      ctrl.id = i;
      if (v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == 0)
      {
        add_control(ctrl, fd);
      }
    }

    /* Check any custom controls */
    for (int i = V4L2_CID_PRIVATE_BASE; ; i++)
    {
      ctrl.id = i;
      if (v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == 0)
      {
        add_control(ctrl, fd);
      }
      else
      {
        break;
      }
    }
  }
  configured_pub_.publish(std_msgs::Empty());
}

MainWindow::~MainWindow()
{
  if (fd >= 0)
    v4l2_close(fd);
}

bool not_alnum(char s)
{
  return not std::isalnum(s);
}

void MainWindow::add_control(const struct v4l2_queryctrl &ctrl, int fd)
{
  std::stringstream name_ss;
  name_ss << ctrl.name;
  std::string name = name_ss.str();
  //std::replace(name.begin(), name.end(), " ", "_");

  // http://stackoverflow.com/questions/6319872/how-to-strip-all-non-alphanumeric-characters-from-a-string-in-c
  // name.erase(std::remove_if(name.begin(), name.end(), std::not1(std::ptr_fun(std::isalnum)), name.end()), name.end());
  name.erase(std::remove_if(name.begin(), name.end(),
      (int(*)(int))not_alnum), name.end());

  // TODO(lucasw) clear out all other params under controls first
  // a previous run would leave leftovers
  ros::param::set("controls/" + name, name_ss.str());
  ros::param::set("controls/" + name + "_min", ctrl.minimum);
  ros::param::set("controls/" + name + "_max", ctrl.maximum);

  if (ctrl.flags & V4L2_CTRL_FLAG_DISABLED)
    return;

  pub_[name] = nh_.advertise<std_msgs::Int32>("feedback/" + name, 1, true);
  switch (ctrl.type)
  {
  case V4L2_CTRL_TYPE_INTEGER:
    ros::param::set("controls/" + name + "_type", "int");
    integer_controls_[name] = new V4L2IntegerControl(fd, ctrl, this, &pub_[name]);
    sub_[name] = nh_.subscribe<std_msgs::Int32>("controls/" + name, 10,
        boost::bind(&MainWindow::integerControlCallback, this, _1, name));
    break;
  case V4L2_CTRL_TYPE_BOOLEAN:
    ros::param::set("controls/" + name + "_type", "bool");
    bool_controls_[name] = new V4L2BooleanControl(fd, ctrl, this, &pub_[name]);
    sub_[name] = nh_.subscribe<std_msgs::Int32>("controls/" + name, 10,
        boost::bind(&MainWindow::boolControlCallback, this, _1, name));
    break;
  case V4L2_CTRL_TYPE_MENU:
    ros::param::set("controls/" + name + "_type", "menu");
    menu_controls_[name] = new V4L2MenuControl(fd, ctrl, this, &pub_[name]);
    sub_[name] = nh_.subscribe<std_msgs::Int32>("controls/" + name, 10,
        boost::bind(&MainWindow::menuControlCallback, this, _1, name));
    break;
  case V4L2_CTRL_TYPE_BUTTON:
    ros::param::set("controls/" + name + "_type", "button");
    button_controls_[name] = new V4L2ButtonControl(fd, ctrl, this, &pub_[name]);
    sub_[name] = nh_.subscribe<std_msgs::Int32>("controls/" + name, 10,
        boost::bind(&MainWindow::buttonControlCallback, this, _1, name));
    break;
  case V4L2_CTRL_TYPE_INTEGER64:
    ros::param::set("controls/" + name + "_type", "int64");
    break;
  case V4L2_CTRL_TYPE_CTRL_CLASS:
    ros::param::set("controls/" + name + "_type", "ctrl");
  default:
    ros::param::set("controls/" + name + "_type", int(ctrl.type));
    break;
  }


  #if 0
  if (!w)
  {
    if (ctrl.type == V4L2_CTRL_TYPE_CTRL_CLASS)
      layout->addWidget(new QLabel(parent));
    else
      layout->addWidget(new QLabel("Unknown control", parent));

    layout->addWidget(new QLabel(parent));
    layout->addWidget(new QLabel(parent));
    return;
  }
  #endif

  if (ctrl.flags & (V4L2_CTRL_FLAG_GRABBED | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE))
  {
    // w->setEnabled(false);
  }


  if (ctrl.type == V4L2_CTRL_TYPE_BUTTON)
  {
  }
  else
  {
  }

  // TODO(lucasw) have a ros timer that updates the values from hardware
}

void MainWindow::integerControlCallback(
    const std_msgs::Int32::ConstPtr& msg, std::string name)
{
  // The ui is overriding this control immediately after it is set
  // need to set the slider to this value.
  // ROS_INFO_STREAM("integer " << name << " " << msg->data);
  integer_controls_[name]->setValue(msg->data);
}

void MainWindow::boolControlCallback(
    const std_msgs::Int32::ConstPtr& msg, std::string name)
{
  bool_controls_[name]->setValue(msg->data);
}

void MainWindow::menuControlCallback(
    const std_msgs::Int32::ConstPtr& msg, std::string name)
{
  menu_controls_[name]->setValue(msg->data);
}

void MainWindow::buttonControlCallback(
    const std_msgs::Int32::ConstPtr& msg, std::string name)
{
  // TODO(lucasw) this doesn't do anything
  button_controls_[name]->setValue(msg->data);
}

void MainWindow::about()
{
  std::string about ="This application is a port of an original v4l2ucp to Qt4 library,\n"
                     "v4l2ucp is a universal control panel for all V4L2 devices. The\n"
                     "controls come directly from the driver. If they cause problems\n"
                     "with your hardware, please contact the maintainer of the driver.\n\n"
                     "Copyright (C) 2005 Scott J. Bertin\n"
                     "Copyright (C) 2009-2010 Vasily Khoruzhick\n\n"
                     "This program is free software; you can redistribute it and/or modify\n"
                     "it under the terms of the GNU General Public License as published by\n"
                     "the Free Software Foundation; either version 2 of the License, or\n"
                     "(at your option) any later version.\n\n"
                     "This program is distributed in the hope that it will be useful,\n"
                     "but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
                     "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
                     "GNU General Public License for more details.\n\n"
                     "You should have received a copy of the GNU General Public License\n"
                     "along with this program; if not, write to the Free Software\n"
                     "Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA\n";

  // ROS_INFO_STREAM("v4l2ucp Version " << V4L2UCP_VERSION << about);
}
