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
#include <gz/common/Console.hh>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>

#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace buoy_gazebo
{
class MooringForcePrivate
{
public:
  /// \brief Buoy link entity
  gz::sim::Entity buoyLinkEnt{gz::sim::kNullEntity};

  /// \brief Buoy link on water surface
  gz::sim::Link buoyLink;

  /// \brief World pose of buoy link
  gz::math::Vector3d buoyPos;

  /// \brief Heave cone link entity
  gz::sim::Entity heaveConeLinkEnt{gz::sim::kNullEntity};

  /// \brief Heave cone link to which the virtual mooring is attached
  gz::sim::Link heaveConeLink;

  /// \brief A predefined pose we assume the anchor to be
  gz::math::Vector3d anchorPos{20, 0, -77};

  /// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Meters, vertical distance from buoy to anchor. Updated per
  /// iteration
  double V = 82.0;

  /// \brief Meters, total length of mooring chain
  double L = 160.0;

  /// \brief Meters, horizontal distance from buoy to anchor. Updated per
  /// iteration
  double H = 120.0;

  /// \brief N/m, weight of chain per unit length
  double w = 20.0;

  /// \brief Catenary equation to pass to solver
  CatenaryHSoln catenarySoln{V, H, L};

  /// \brief Solution to catenary equation
  Eigen::VectorXd B{};

  /// \brief Look for buoy link to find input to catenary equation, and heave
  /// cone link to apply output force to
  bool FindLinks(gz::sim::EntityComponentManager & _ecm);

  /// \brief Update V and H for solver input
  void UpdateVH(gz::sim::EntityComponentManager & _ecm);
};

//////////////////////////////////////////////////
MooringForce::MooringForce()
: dataPtr(std::make_unique<MooringForcePrivate>())
{
}

//////////////////////////////////////////////////
bool MooringForcePrivate::FindLinks(
  gz::sim::EntityComponentManager & _ecm)
{
  // Look for buoy link to get input for catenary equation
  this->buoyLinkEnt = this->model.LinkByName(_ecm,
    "Buoy");
  if (this->buoyLinkEnt != gz::sim::kNullEntity) {
    this->buoyLink = gz::sim::Link(
      this->buoyLinkEnt);
    if (!this->buoyLink.Valid(_ecm))
    {
      ignwarn << "Could not find valid buoy link. Mooring force may "
        << "not be calculated correctly." << std::endl;
      return false;
    }
  }
  else {
    ignwarn << "Could not find valid buoy link. Mooring force may "
      << "not be calculated correctly." << std::endl;
    return false;
  }

  // Look for heave cone link to apply force to
  this->heaveConeLinkEnt = this->model.LinkByName(_ecm,
    "HeaveCone");
  if (this->heaveConeLinkEnt != gz::sim::kNullEntity) {
    this->heaveConeLink = gz::sim::Link(
      this->heaveConeLinkEnt);
    if (!this->heaveConeLink.Valid(_ecm))
    {
      ignwarn << "Could not find valid heave cone link. Mooring force may "
        << "not be applied correctly." << std::endl;
      return false;
    }
  }
  else {
    ignwarn << "Could not find valid heave cone link. Mooring force may "
      << "not be applied correctly." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void MooringForcePrivate::UpdateVH(
  gz::sim::EntityComponentManager & _ecm)
{
  // If necessary links not found yet, nothing to do
  if (this->heaveConeLinkEnt == gz::sim::kNullEntity ||
    this->buoyLinkEnt == gz::sim::kNullEntity) {
    return;
  }

  // Get buoy position in world
  auto buoyPose = this->buoyLink.WorldPose(_ecm);
  this->buoyPos = buoyPose->Pos();

  // Update vertical distance between buoy and anchor
  this->V = fabs(this->buoyPos[2] - this->anchorPos[2]);

  // Update horizontal distance between buoy and anchor
  this->H = sqrt(
    (this->buoyPos[0] - this->anchorPos[0]) *
    (this->buoyPos[0] - this->anchorPos[0]) +
    (this->buoyPos[1] - this->anchorPos[1]) *
    (this->buoyPos[1] - this->anchorPos[1]));
}

//////////////////////////////////////////////////
void MooringForce::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  // Skip debug messages to run faster TODO change to 3 before merge PR
  gz::common::Console::SetVerbosity(4);

  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "MooringForce plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->anchorPos = _sdf->Get<gz::math::Vector3d>(
    "anchor_position", this->dataPtr->anchorPos).first;
  igndbg << "Anchor position set to " << this->dataPtr->anchorPos
    << std::endl;

  // Find necessary model links
  if (this->dataPtr->FindLinks(_ecm)) {
    this->dataPtr->UpdateVH(_ecm);
  }

  this->dataPtr->B.resize(1);
}

//////////////////////////////////////////////////
void MooringForce::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("MooringForce::PreUpdate");

  // If necessary links have not been identified yet, the plugin is disabled
  if (this->dataPtr->heaveConeLinkEnt == gz::sim::kNullEntity ||
    this->dataPtr->buoyLinkEnt == gz::sim::kNullEntity) {
    this->dataPtr->FindLinks(_ecm);
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

  // Update V and H based on latest buoy position
  this->dataPtr->UpdateVH(_ecm);

  Eigen::HybridNonLinearSolver<CatenaryHSoln> catenarySolver(this->dataPtr->catenarySoln);
  // Tolerance for error between two consecutive iterations
  catenarySolver.parameters.xtol = 0.0001;
  // Max number of calls to the function
  catenarySolver.parameters.maxfev = 1000;

  // Initial guess B = L - V - b
  double b = 0.0;
  this->dataPtr->B[0] = this->dataPtr->L - this->dataPtr->V - b;

  // Scaling factor
  double c = CatenaryFunction::CatenaryScalingFactor(
    this->dataPtr->V, this->dataPtr->B[0], this->dataPtr->L);

  int solverInfo;
  // Increment b, to iterate on B
  while ((c <= 1e-5) && (this->dataPtr->L - this->dataPtr->V - b > 0.0)) {
    // Solve for B, pass in initial guess
    solverInfo = catenarySolver.solveNumericalDiff(this->dataPtr->B);

    // Recalculate c with newly solved B
    c = CatenaryFunction::CatenaryScalingFactor(
      this->dataPtr->V, this->dataPtr->B[0], this->dataPtr->L);

    // Found solution
    if (solverInfo == 1)
      break;

    b += 1.0;
    this->dataPtr->B[0] = this->dataPtr->L - this->dataPtr->V - b;
  }

  // Horizontal component of chain tension, in Newtons
  // Force at buoy heave cone is Fx = -Tx
  double Tx = c * this->dataPtr->w;
  // Vertical component of chain tension at buoy heave cone, in Newtons
  double Ty = - this->dataPtr->w * (this->dataPtr->L - this->dataPtr->B[0]);

  igndbg << "HSolver solverInfo: " << solverInfo << " c: " << c
    << " B: " << this->dataPtr->B[0]
    << " Tx: " << Tx
    << " Ty: " << Ty
    << std::endl;

  // Apply forces to buoy heave cone link, where the mooring would be attached
  gz::math::Vector3d force(-Tx, Ty, 0);
  gz::math::Vector3d torque(0, 0, 0);
  this->dataPtr->buoyLink.AddWorldWrench(_ecm, force, torque);
}
}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::MooringForce,
  gz::sim::System,
  buoy_gazebo::MooringForce::ISystemConfigure,
  buoy_gazebo::MooringForce::ISystemPreUpdate)
