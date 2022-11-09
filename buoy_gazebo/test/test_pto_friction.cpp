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

#include <simple_interp/interp1d.hpp>

#include <vector>


// NOLINTNEXTLINE
class PTOFrictionTest : public ::testing::Test
{
public:
  const std::vector<double> speed;  // m/s
  const std::vector<double> force;  // N
  simple_interp::Interp1d friction_model;

  PTOFrictionTest()
  :  speed{-5.0, -0.4, -0.1, -0.05, 0.0, 0.05, 0.1, 5.0},
    force{12750.0, 1200.0, 700.0, 400.0, 0.0, -750.0, -1000.0, -32033.0},
    friction_model(speed, force)
  {
  }
};

TEST_F(PTOFrictionTest, ModelAccuracy)
{
  EXPECT_NEAR(0.0, friction_model.eval(0.0), 0.05);
  EXPECT_NEAR(700.0, friction_model.eval(-0.1), 0.05);
  EXPECT_NEAR(-1000.0, friction_model.eval(0.1), 0.05);
  EXPECT_NEAR(-2100.0, friction_model.eval(0.3), 150.0);
  EXPECT_NEAR(1000.0, friction_model.eval(-0.3), 250.0);
  EXPECT_NEAR(-2900.0, friction_model.eval(0.4), 500.0);
}
