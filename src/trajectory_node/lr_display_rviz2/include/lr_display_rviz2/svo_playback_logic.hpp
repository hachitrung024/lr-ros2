// Copyright 2026 Landfill Rover Maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LR_DISPLAY_RVIZ2__SVO_PLAYBACK_LOGIC_HPP_
#define LR_DISPLAY_RVIZ2__SVO_PLAYBACK_LOGIC_HPP_

#include <algorithm>
#include <cstdint>

namespace lr_display_rviz2
{

enum class PlaybackState : uint8_t
{
  Playing = 0,
  Paused = 1,
  End = 2,
};

enum class SeekPolicy
{
  PreservePlayback,
  PauseAtTarget,
  ResumeAtTarget,
};

enum class SeekStage
{
  Idle,
  Pausing,
  SeekPending,
  Seeking,
  Resuming,
  Complete,
  Failed,
};

inline int32_t clamp_frame(int64_t requested, uint32_t total_frames)
{
  if (total_frames == 0U) {
    return 0;
  }
  const auto maximum = static_cast<int64_t>(total_frames) - 1;
  return static_cast<int32_t>(std::clamp<int64_t>(requested, 0, maximum));
}

class SeekTransaction
{
public:
  bool begin(
    int64_t requested_frame,
    uint32_t total_frames,
    PlaybackState playback_state,
    SeekPolicy policy)
  {
    if (active() || total_frames == 0U) {
      return false;
    }
    target_frame_ = clamp_frame(requested_frame, total_frames);
    resume_after_seek_ = policy == SeekPolicy::ResumeAtTarget ||
      (policy == SeekPolicy::PreservePlayback &&
      playback_state == PlaybackState::Playing);
    stage_ = playback_state == PlaybackState::Paused ?
      SeekStage::SeekPending : SeekStage::Pausing;
    return true;
  }

  bool active() const
  {
    return stage_ == SeekStage::Pausing ||
           stage_ == SeekStage::SeekPending ||
           stage_ == SeekStage::Seeking ||
           stage_ == SeekStage::Resuming;
  }

  int32_t target_frame() const {return target_frame_;}
  bool resume_after_seek() const {return resume_after_seek_;}
  SeekStage stage() const {return stage_;}

  bool pause_succeeded()
  {
    if (stage_ != SeekStage::Pausing) {
      return false;
    }
    stage_ = SeekStage::SeekPending;
    return true;
  }

  bool seek_started()
  {
    if (stage_ != SeekStage::SeekPending) {
      return false;
    }
    stage_ = SeekStage::Seeking;
    return true;
  }

  bool seek_succeeded()
  {
    if (stage_ != SeekStage::Seeking) {
      return false;
    }
    stage_ = resume_after_seek_ ? SeekStage::Resuming : SeekStage::Complete;
    return true;
  }

  bool resume_succeeded()
  {
    if (stage_ != SeekStage::Resuming) {
      return false;
    }
    stage_ = SeekStage::Complete;
    return true;
  }

  void fail() {stage_ = SeekStage::Failed;}
  void reset() {stage_ = SeekStage::Idle;}

private:
  int32_t target_frame_{0};
  bool resume_after_seek_{false};
  SeekStage stage_{SeekStage::Idle};
};

}  // namespace lr_display_rviz2

#endif  // LR_DISPLAY_RVIZ2__SVO_PLAYBACK_LOGIC_HPP_
