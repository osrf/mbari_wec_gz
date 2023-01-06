// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
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

#include "CatenarySoln.hpp"
#include "MooringForce.hpp"

// Only for debug level TODO remove before merge PR
#include <ignition/common/Console.hh>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/World.hh>

#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace buoy_gazebo
{
class MooringForcePrivate
{
public:
  /// \brief Heave cone link entity
  ignition::gazebo::Entity heaveConeLinkEnt{ignition::gazebo::kNullEntity};

  /// \brief Heave cone link to which the virtual mooring is attached
  ignition::gazebo::Link heaveConeLink;

  /// \brief A predefined pose we assume the anchor to be
  ignition::math::Vector3d anchorPos{20, 0, -77};

  /// \brief Model interface
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};

  /// \brief Meters, vertical distance from buoy to anchor
  double V = 82.0;

  /// \brief Meters, total length of mooring chain
  double L = 160.0;

  /// \brief Meters, horizontal distance from buoy to anchor
  double H = 120.0;

  /// \brief N/m, weight of chain per unit length
  double w = 20.0;

  /// \brief Catenary equation to pass to solver
  CatenaryHSoln catenarySoln{V, H, L};

  /// \brief Solution to catenary equation
  Eigen::VectorXd B{};

  /// \brief Look for heave cone link to apply force to
  void FindLink(ignition::gazebo::EntityComponentManager & _ecm);
};

//////////////////////////////////////////////////
MooringForce::MooringForce()
: dataPtr(std::make_unique<MooringForcePrivate>())
{
}

//////////////////////////////////////////////////
void MooringForcePrivate::FindLink(
  ignition::gazebo::EntityComponentManager & _ecm)
{
  this->heaveConeLinkEnt = this->model.LinkByName(_ecm,
    "HeaveCone");
  if (this->heaveConeLinkEnt != ignition::gazebo::kNullEntity) {
    this->heaveConeLink = ignition::gazebo::Link(
      this->heaveConeLinkEnt);
    if (!this->heaveConeLink.Valid(_ecm))
    {
      ignwarn << "Could not find valid heave cone link. Mooring force may "
        << "not be applied correctly." << std::endl;
    }
  }
  else {
    ignwarn << "Could not find valid heave cone link. Mooring force may "
      << "not be applied correctly." << std::endl;
  }
}

//////////////////////////////////////////////////
void MooringForce::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager & /*_eventMgr*/)
{
  // Skip debug messages to run faster TODO change to 3 before merge PR
  ignition::common::Console::SetVerbosity(4);

  this->dataPtr->model = ignition::gazebo::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "MooringForce plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->anchorPos = _sdf->Get<ignition::math::Vector3d>(
    "anchor_position", this->dataPtr->anchorPos).first;
  igndbg << "Anchor position set to " << this->dataPtr->anchorPos
    << std::endl;

  // Find heave cone link
  this->dataPtr->FindLink(_ecm);

  this->dataPtr->B.resize(1);
}

//////////////////////////////////////////////////
void MooringForce::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("MooringForce::PreUpdate");

  // If the link hasn't been identified yet, the plugin is disabled
  if (this->dataPtr->heaveConeLinkEnt == ignition::gazebo::kNullEntity) {
    this->dataPtr->FindLink(_ecm);
    return;
  }

  // TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn << "Detected jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() <<
      "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  Eigen::HybridNonLinearSolver<CatenaryHSoln> catenarySolver(this->dataPtr->catenarySoln);
  // Tolerance for error between two consecutive iterations
  catenarySolver.parameters.xtol = 0.0001;
  // Max number of calls to the function
  catenarySolver.parameters.maxfev = 1000;

  // Initial guess B = L - V - b
  // TODO Is this how to pass in L - V - b as initial guess for B?
  double b = 0.0;
  this->dataPtr->B[0] = this->dataPtr->L - this->dataPtr->V - b;

  // Scaling factor
  double c = CatenaryFunction::CatenaryScalingFactor(
    this->dataPtr->V, this->dataPtr->B[0], this->dataPtr->L);

  int solverInfo;
  // Increment b, to iterate on B
  while ((c <= 1e-5) && (this->dataPtr->L - this->dataPtr->V - b > 0.0)) {
    b += 1.0;
    // Solve for B, pass in initial guess
    solverInfo = catenarySolver.solveNumericalDiff(this->dataPtr->B);

    // Recalculate c with newly solved B
    c = CatenaryFunction::CatenaryScalingFactor(
      this->dataPtr->V, this->dataPtr->B[0], this->dataPtr->L);

    // Found solution
    if (solverInfo == 1)
      break;
  }

  igndbg << "HSolver solverInfo: " << solverInfo << " c: " << c
    << " B: " << this->dataPtr->B[0] << std::endl;

  // Horizontal component of chain tension, in Newtons
  // Force at buoy heave cone is Fx = -Tx
  double Tx = c * this->dataPtr->w;
  // Vertical component of chain tension at buoy heave cone, in Newtons
  double Ty = - this->dataPtr->w * (this->dataPtr->L - this->dataPtr->B[0]);

  // Apply forces to buoy heave cone link, where the mooring would be attached
  //ignition::math::Vector3d force(-Tx, Ty, 0);
  //ignition::math::Vector3d torque(0, 0, 0);
  //buoyLink.AddWorldWrench(_ecm, force, torque);
}
}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::MooringForce,
  ignition::gazebo::System,
  buoy_gazebo::MooringForce::ISystemConfigure,
  buoy_gazebo::MooringForce::ISystemPreUpdate)
