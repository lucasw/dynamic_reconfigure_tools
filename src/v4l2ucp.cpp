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
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include <QApplication>

#include "v4l2ucp/mainWindow.h"

void usage(const char *argv0)
{
  using std::cout;
  using std::endl;

  cout << "Usage: " << argv0 << " [-h | --help] [filename]..." << endl;
  cout << "-h or --help will print this message and exit." << endl;
  cout << "filename is one or more device files for the ";
  cout << "V4L2 devices to control." << endl;
  cout << "If no filenames are given, the filename specified in the" << endl;
  cout << "environment variable V4L2UCP_DEV, or /dev/video0 will be used.";
  cout << endl;
  cout << "Also accepts standard Qt arguments." << endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "v4l2ucp");
  MainWindow *w;
  QApplication a(argc, argv);

  std::string device = "/dev/video0";
  ros::param::get("~device", device);

  w = MainWindow::openFile(device);
  if (w)
  {
    w->show();
  }

  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()));
  return a.exec();
}
