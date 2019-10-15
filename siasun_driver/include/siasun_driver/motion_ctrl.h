﻿/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#ifndef MOTOMAN_DRIVER_MOTION_CTRL_H
#define MOTOMAN_DRIVER_MOTION_CTRL_H

#include "simple_message/smpl_msg_connection.h"
#include "siasun_driver/simple_message/siasun_motion_ctrl.h"
#include "siasun_driver/simple_message/siasun_motion_reply.h"

namespace siasun
{
namespace motion_ctrl
{
using industrial::smpl_msg_connection::SmplMsgConnection;
using siasun::simple_message::motion_reply::MotionReply;
typedef siasun::simple_message::motion_ctrl::MotionControlCmd MotionControlCmd;

/**
 * \brief Wrapper class around Siasun-specific motion control commands
 */

class SiasunMotionCtrl
{
public:
  /**
   * \brief Default constructor
   */
  SiasunMotionCtrl() {}

  bool init(SmplMsgConnection* connection, int robot_id);

public:
  bool controllerReady();
  bool setTrajMode(bool enable);
  bool stopTrajectory();
  bool ctrlGripper(bool hand, bool status);

  static std::string getErrorString(const MotionReply &reply);

protected:
  SmplMsgConnection* connection_;
  int robot_id_;

  bool sendAndReceive(MotionControlCmd command, MotionReply &reply);
  bool sendAndReceive(MotionControlCmd command, MotionReply &reply, bool hand, bool status);
};

}  // namespace motion_ctrl
}  // namespace siasun

#endif  // MOTOMAN_DRIVER_MOTION_CTRL_H
