// Copyright 2026 LR
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

#include <QApplication>
#include <QEventLoop>
#include <QLabel>
#include <QTimer>
#include <memory>

#include "lr_display_rviz2/svo_control_panel.hpp"

namespace lr_display_rviz2
{

class SvoControlPanelTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    static int argc = 1;
    static char app_name[] = "svo_panel_test";
    static char * argv[] = {app_name, nullptr};
    application_ = std::make_unique<QApplication>(argc, argv);
  }

  static void TearDownTestSuite();

  uint64_t begin(SvoControlPanel & panel, std::function<void()> cancel)
  {
    panel.request_timer_->setInterval(10);
    auto generation = panel.beginRequest();
    panel.cancel_pending_request_ = std::move(cancel);
    return generation;
  }

  bool pending(const SvoControlPanel & panel);

  void complete(SvoControlPanel & panel);

  QString feedback(const SvoControlPanel & panel);

  static void increment(int & count);

  void processEvents()
  {
    QEventLoop loop;
    QTimer::singleShot(30, &loop, &QEventLoop::quit);
    loop.exec();
  }

  static std::unique_ptr<QApplication> application_;
};

std::unique_ptr<QApplication> SvoControlPanelTest::application_;

void SvoControlPanelTest::TearDownTestSuite()
{
  application_.reset();
}

bool SvoControlPanelTest::pending(const SvoControlPanel & panel)
{
  return panel.request_pending_;
}

void SvoControlPanelTest::complete(SvoControlPanel & panel)
{
  panel.finishRequest("Completed", false);
}

QString SvoControlPanelTest::feedback(const SvoControlPanel & panel)
{
  return panel.feedback_label_->text();
}

void SvoControlPanelTest::increment(int & count)
{
  ++count;
}

TEST_F(SvoControlPanelTest, TimeoutCleansRequestAndAllowsRetry)
{
  SvoControlPanel panel;
  int cancellations = 0;
  auto generation = begin(panel, std::bind(increment, std::ref(cancellations)));
  ASSERT_TRUE(pending(panel));
  processEvents();
  EXPECT_FALSE(pending(panel));
  EXPECT_EQ(cancellations, 1);
  EXPECT_TRUE(feedback(panel).contains("timed out"));
  EXPECT_GT(begin(panel, std::bind(increment, std::ref(cancellations))), generation);
  complete(panel);
  processEvents();
  EXPECT_FALSE(pending(panel));
  EXPECT_EQ(cancellations, 1);
  EXPECT_EQ(feedback(panel), "Completed");
}

TEST_F(SvoControlPanelTest, DestructionCleansOutstandingRequest)
{
  int cancellations = 0;
  {
    SvoControlPanel panel;
    begin(panel, std::bind(increment, std::ref(cancellations)));
  }
  EXPECT_EQ(cancellations, 1);
}

}  // namespace lr_display_rviz2
