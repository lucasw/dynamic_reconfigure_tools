#ifndef V4L2UCP_MAINWINDOW_H
#define V4L2UCP_MAINWINDOW_H
/*  v4l2ucp - A universal control panel for all V4L2 devices
    Copyright (C) 2005 Scott J. Bertin (scottbertin@yahoo.com)
    Copyright (C) 2009 Vasily Khoruzhick (anarsoul@gmail.com)

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
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "v4l2ucp/v4l2controls.h"

class MainWindow
{
public:
  MainWindow();
  ~MainWindow();

  void about();

private:
  int fd;
  // QTimer timer;
  ros::NodeHandle nh_;
  std::map<std::string, ros::Subscriber> sub_;
  std::map<std::string, ros::Publisher> pub_;

  ros::Publisher configured_pub_;

  std::map<std::string, V4L2IntegerControl*> integer_controls_;
  std::map<std::string, V4L2BooleanControl*> bool_controls_;
  std::map<std::string, V4L2MenuControl*> menu_controls_;
  std::map<std::string, V4L2ButtonControl*> button_controls_;

  void add_control(const struct v4l2_queryctrl &ctrl, int fd);

  void integerControlCallback(const std_msgs::Int32::ConstPtr& msg, std::string name);
  void boolControlCallback(const std_msgs::Int32::ConstPtr& msg, std::string name);
  void menuControlCallback(const std_msgs::Int32::ConstPtr& msg, std::string name);
  void buttonControlCallback(const std_msgs::Int32::ConstPtr& msg, std::string name);
};

#endif  // V4L2UCP_MAINWINDOW_H
