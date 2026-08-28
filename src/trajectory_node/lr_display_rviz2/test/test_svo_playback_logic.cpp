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

#include <gtest/gtest.h>

#include "lr_display_rviz2/svo_playback_logic.hpp"

namespace lr_display_rviz2
{

TEST(SvoPlaybackLogic, ClampFrameToSvoRange)
{
  EXPECT_EQ(clamp_frame(-10, 100), 0);
  EXPECT_EQ(clamp_frame(42, 100), 42);
  EXPECT_EQ(clamp_frame(1000, 100), 99);
  EXPECT_EQ(clamp_frame(10, 0), 0);
}

TEST(SvoPlaybackLogic, PreservePlayingPausesSeeksAndResumes)
{
  SeekTransaction transaction;
  ASSERT_TRUE(
    transaction.begin(
      25, 100, PlaybackState::Playing, SeekPolicy::PreservePlayback));
  EXPECT_EQ(transaction.target_frame(), 25);
  EXPECT_EQ(transaction.stage(), SeekStage::Pausing);
  EXPECT_TRUE(transaction.resume_after_seek());
  EXPECT_TRUE(transaction.pause_succeeded());
  EXPECT_TRUE(transaction.seek_started());
  EXPECT_TRUE(transaction.seek_succeeded());
  EXPECT_EQ(transaction.stage(), SeekStage::Resuming);
  EXPECT_TRUE(transaction.resume_succeeded());
  EXPECT_EQ(transaction.stage(), SeekStage::Complete);
}

TEST(SvoPlaybackLogic, PreservePausedSeeksWithoutToggle)
{
  SeekTransaction transaction;
  ASSERT_TRUE(
    transaction.begin(
      50, 100, PlaybackState::Paused, SeekPolicy::PreservePlayback));
  EXPECT_EQ(transaction.stage(), SeekStage::SeekPending);
  EXPECT_FALSE(transaction.resume_after_seek());
  EXPECT_TRUE(transaction.seek_started());
  EXPECT_TRUE(transaction.seek_succeeded());
  EXPECT_EQ(transaction.stage(), SeekStage::Complete);
}

TEST(SvoPlaybackLogic, FrameStepStopsAtTarget)
{
  SeekTransaction transaction;
  ASSERT_TRUE(
    transaction.begin(
      101, 100, PlaybackState::Playing, SeekPolicy::PauseAtTarget));
  EXPECT_EQ(transaction.target_frame(), 99);
  EXPECT_EQ(transaction.stage(), SeekStage::Pausing);
  EXPECT_FALSE(transaction.resume_after_seek());
  EXPECT_TRUE(transaction.pause_succeeded());
  EXPECT_TRUE(transaction.seek_started());
  EXPECT_TRUE(transaction.seek_succeeded());
  EXPECT_EQ(transaction.stage(), SeekStage::Complete);
}

TEST(SvoPlaybackLogic, EndStateIsPausedBeforeSeek)
{
  SeekTransaction transaction;
  ASSERT_TRUE(
    transaction.begin(
      0, 100, PlaybackState::End, SeekPolicy::PreservePlayback));
  EXPECT_EQ(transaction.stage(), SeekStage::Pausing);
  EXPECT_FALSE(transaction.resume_after_seek());
}

TEST(SvoPlaybackLogic, RestartResumesAfterSeekingFromEnd)
{
  SeekTransaction transaction;
  ASSERT_TRUE(
    transaction.begin(
      0, 100, PlaybackState::End, SeekPolicy::ResumeAtTarget));
  EXPECT_EQ(transaction.stage(), SeekStage::Pausing);
  EXPECT_TRUE(transaction.resume_after_seek());
  EXPECT_TRUE(transaction.pause_succeeded());
  EXPECT_TRUE(transaction.seek_started());
  EXPECT_TRUE(transaction.seek_succeeded());
  EXPECT_EQ(transaction.stage(), SeekStage::Resuming);
}

TEST(SvoPlaybackLogic, FailureAndTimeoutEndTheTransaction)
{
  SeekTransaction failed;
  ASSERT_TRUE(
    failed.begin(
      10, 100, PlaybackState::Playing, SeekPolicy::PreservePlayback));
  failed.fail();
  EXPECT_EQ(failed.stage(), SeekStage::Failed);
  EXPECT_FALSE(failed.active());

  SeekTransaction timed_out;
  ASSERT_TRUE(
    timed_out.begin(
      10, 100, PlaybackState::Paused, SeekPolicy::PreservePlayback));
  timed_out.fail();
  EXPECT_EQ(timed_out.stage(), SeekStage::Failed);
  EXPECT_FALSE(timed_out.active());
}

TEST(SvoPlaybackLogic, RejectsConcurrentOrEmptySeek)
{
  SeekTransaction transaction;
  EXPECT_FALSE(
    transaction.begin(
      10, 0, PlaybackState::Paused, SeekPolicy::PreservePlayback));
  ASSERT_TRUE(
    transaction.begin(
      10, 100, PlaybackState::Paused, SeekPolicy::PreservePlayback));
  EXPECT_FALSE(
    transaction.begin(
      20, 100, PlaybackState::Paused, SeekPolicy::PreservePlayback));
}

}  // namespace lr_display_rviz2
