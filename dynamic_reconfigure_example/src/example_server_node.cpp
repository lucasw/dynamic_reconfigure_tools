/*
 * Copyright (c) 2017 Lucas Walter
 * June 2017
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure_example/ExampleConfig.h>
#include <ros/ros.h>

class ExampleServer
{
  ros::NodeHandle nh_;
  dynamic_reconfigure_example::ExampleConfig config_;
  typedef dynamic_reconfigure::Server<dynamic_reconfigure_example::ExampleConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(dynamic_reconfigure_example::ExampleConfig& config,
      uint32_t level);

  boost::recursive_mutex dr_mutex_;

  public:
  ExampleServer();
  ~ExampleServer();
};

ExampleServer::ExampleServer() :
  nh_("~")
{
  ROS_INFO_STREAM(config_.str_param);
  server_.reset(new ReconfigureServer(dr_mutex_, nh_));
  dynamic_reconfigure::Server<dynamic_reconfigure_example::ExampleConfig>::CallbackType cbt =
      boost::bind(&ExampleServer::callback, this, boost::placeholders::_1, boost::placeholders::_2);
  server_->setCallback(cbt);
  ROS_INFO_STREAM(config_.str_param);
}

ExampleServer::~ExampleServer()
{
}

void ExampleServer::callback(dynamic_reconfigure_example::ExampleConfig& config,
      uint32_t level)
{
  ROS_DEBUG_STREAM(config.str_param << " " << config_.str_param);
  config_ = config;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "example_server_node");
  ExampleServer example_server;
  ros::spin();
}
