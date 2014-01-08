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

#ifndef MAGERAY_ASYNC_IMAGE_SAVER_H_
#define MAGERAY_ASYNC_IMAGE_SAVER_H_

#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "base/types.h"
#include "image.h"
#include "vec.h"

namespace mageray {

// This is a threaded image saver that will save images to disk using a worker
// thread.
class AsyncImageSaver {
  public:
    AsyncImageSaver();
    ~AsyncImageSaver();

    void SavePNG(std::unique_ptr<Image> image, const std::string& file_name);

  private:
    void Worker();

    class Command {
      public:
        virtual ~Command() {}
        virtual void Execute() = 0;
    };

    class SavePNGCommand : public Command {
      public:
        SavePNGCommand(std::unique_ptr<Image> image, const std::string& file_name);
        void Execute();

      private:
        std::unique_ptr<Image> m_image;
        std::string m_file_name;
    };

    std::thread m_thread;

    std::list<Command*> m_cmds;
    std::mutex m_cmds_lock;
    std::condition_variable m_cmds_cond;
    bool m_terminate;
};

} // namespace mageray

#endif // MAGERAY_ASYNC_IMAGE_SAVER_H_
