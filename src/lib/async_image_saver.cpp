// -*- Mode: c++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//------------------------------------------------------------------------------
// Copyright (c) 2014, Marcus Geelnard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------

#include "async_image_saver.h"

namespace mageray {

AsyncImageSaver::AsyncImageSaver() : m_terminate(false) {
  // Start the worker thread.
  m_thread = std::thread(&AsyncImageSaver::Worker, this);
}

AsyncImageSaver::~AsyncImageSaver() {
  // Tell the worker to terminate ASAP.
  m_cmds_lock.lock();
  m_terminate = true;
  m_cmds_lock.unlock();
  m_cmds_cond.notify_all();

  // Wait for the worker to terminate.
  m_thread.join();
}

void AsyncImageSaver::SavePNG(
    std::unique_ptr<Image> image, const std::string& file_name) {
  std::unique_lock<std::mutex> guard(m_cmds_lock);

  // BLock until we have trimmed down the command queue, if necessary.
  while (m_cmds.size() >= kMaxPendingCommands) {
    m_cmds_cond.wait(guard);
  }

  // Create a new save command.
  m_cmds.push_back(new SavePNGCommand(std::move(image), file_name));

  // Tell the worker to execute the command ASAP.
  m_cmds_cond.notify_all();
}

void AsyncImageSaver::Worker() {
  std::unique_lock<std::mutex> guard(m_cmds_lock);
  while (m_cmds.size() > 0 || !m_terminate) {
    // Wait for new save commands.
    while (m_cmds.size() == 0 && !m_terminate) {
      m_cmds_cond.wait(guard);
    }

    if (m_cmds.size() > 0) {
      // Pick up the next save command.
      Command* cmd = m_cmds.front();
      m_cmds.pop_front();

      // Do not block the main thread while executing the command.
      guard.unlock();

      // Tell the main thread that the command queue is now shortened.
      m_cmds_cond.notify_all();

      // Execute the command.
      cmd->Execute();

      // Delete the command.
      delete cmd;

      guard.lock();
    }
  }
}

AsyncImageSaver::SavePNGCommand::SavePNGCommand(
    std::unique_ptr<Image> image, const std::string& file_name) {
  m_image = std::move(image);
  m_file_name = file_name;
}

void AsyncImageSaver::SavePNGCommand::Execute() {
  m_image->SavePNG(m_file_name.c_str());
}

} // namespace mageray
