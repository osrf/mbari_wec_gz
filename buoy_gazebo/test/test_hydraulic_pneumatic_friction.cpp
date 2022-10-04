// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <splinter_ros/splinter1d.hpp>

#include <vector>


// NOLINTNEXTLINE
class HydraulicPneumaticFrictionTest : public ::testing::Test
{
public:
  const std::vector<double> speed;  // m/s
  const std::vector<double> force;  // N
  splinter_ros::Splinter1d friction_model;

  HydraulicPneumaticFrictionTest()
  :  speed{-0.4F, -0.2F, -0.1F, 0.0F, 0.1F, 0.2F, 0.3F, 0.4F},
    force{1200.0F, 700.0F, 500.0F, 0.0F, -1000.0F, -1400.0F, -2100.0F, -2900.0F},
    friction_model(speed, force)
  {
  }
};

TEST_F(HydraulicPneumaticFrictionTest, ModelAccuracy)
{
  EXPECT_NEAR(1000.0, friction_model.eval(-0.3), 100.0);
}
