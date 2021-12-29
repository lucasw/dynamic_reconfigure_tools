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

#ifndef DYNAMIC_RECONFIGURE_EXAMPLE_EXAMPLE_SERVER_H
#define DYNAMIC_RECONFIGURE_EXAMPLE_EXAMPLE_SERVER_H

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure_example/ExampleConfig.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

namespace dynamic_reconfigure_example
{

class ExampleServer : public nodelet::Nodelet
{
  dynamic_reconfigure_example::ExampleConfig config_;
  typedef dynamic_reconfigure::Server<dynamic_reconfigure_example::ExampleConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void callback(dynamic_reconfigure_example::ExampleConfig& config,
      uint32_t level);

  boost::recursive_mutex dr_mutex_;

public:
  virtual void onInit();
  ExampleServer();
  ~ExampleServer();
};

}  // namespace dynamic_reconfigure_example

#endif  // DYNAMIC_RECONFIGURE_EXAMPLE_EXAMPLE_SERVER_H
