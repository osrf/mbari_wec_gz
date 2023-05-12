// Copyright 2022 Open Source Robotics Foundation, Inc.
//                and Monterey Bay Aquarium Research Institute
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
  /// \brief Model interface
  public: gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief The link attached to the mooring chain.
  public: gz::sim::Link link{gz::sim::kNullEntity};

  /// \brief World position of the link
  public: gz::math::Vector3d linkPos;

  /// \brief Name of the link the mooring is attached to.
  public: std::string linkName{"Buoy"};

  /// \brief A predefined pose we assume the anchor to be
  public: gz::math::Vector3d anchorPos{20.0, 0.0, -77.0};

  /// \brief Vertical distance from buoy to anchor. Updated per
  /// iteration (metres).
  public: double V{std::nanf("")};

  /// \brief Total length of mooring chain (metres).
  public: double L{std::nanf("")};

  /// \brief Horizontal distance from buoy to anchor. Updated per
  /// iteration  (metres).
  public: double H{std::nanf("")};

  /// \brief N/m, weight of chain per unit length
  public: double w = 20.0;

  /// \brief radians, atan2 angle of buoy from anchor
  public: double theta = 0.0;

  /// \brief Catenary equation to pass to solver
  public: std::unique_ptr<CatenaryHSoln> catenarySoln;

  /// \brief Solution to catenary equation. Meters, length of chain laying on
  /// the bottom, start of catenary.
  public: Eigen::VectorXd B{};

  /// \brief If we have notified skipping force update
  public: bool notifiedSkipForceUpdate{false};

  /// \brief Debug print period calculated from <debug_print_rate>
  public: std::chrono::steady_clock::duration debugPrintPeriod{0};

  /// \brief Last debug print simulation time
  public: std::chrono::steady_clock::duration lastDebugPrintTime{0};

  /// \brief Constructor
  public: MooringForcePrivate();

  /// \brief Look for buoy link to find input to catenary equation, and heave
  /// cone link to apply output force to
  public: bool FindLinks(gz::sim::EntityComponentManager &_ecm);

  /// \brief Update V and H for solver input
  public: void UpdateVH(gz::sim::EntityComponentManager &_ecm);
};

//////////////////////////////////////////////////
MooringForcePrivate::MooringForcePrivate()
  : catenarySoln{std::make_unique<CatenaryHSoln>(V, H, L)}
{
}

//////////////////////////////////////////////////
bool MooringForcePrivate::FindLinks(
  gz::sim::EntityComponentManager &_ecm)
{
  // Find link
  auto entity = this->model.LinkByName(_ecm, this->linkName);
  this->link = gz::sim::Link(entity);

  if (!this->link.Valid(_ecm)) {
      gzwarn << "Could not find link[" << this->linkName << "]. "
             << "Mooring force will not be calculated."
             << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void MooringForcePrivate::UpdateVH(
  gz::sim::EntityComponentManager &_ecm)
{
  // Skip if buoy link is not valid.
  if (!this->link.Valid(_ecm)) {
    return;
  }

  // Get buoy position in world
  auto linkPose = this->link.WorldPose(_ecm);
  this->linkPos = linkPose->Pos();

  // Update vertical (z) distance between buoy and anchor
  this->V = fabs(this->linkPos[2U] - this->anchorPos[2U]);

  // Update horizontal distance between buoy and anchor
  this->H = sqrt(
    (this->linkPos[0U] - this->anchorPos[0U]) *
    (this->linkPos[0U] - this->anchorPos[0U]) +
    (this->linkPos[1U] - this->anchorPos[1U]) *
    (this->linkPos[1U] - this->anchorPos[1U]));

  // Update angle between buoy and anchor
  this->theta = atan2(this->linkPos[1U] - this->anchorPos[1U],
    this->linkPos[0U] - this->anchorPos[0U]);

  this->catenarySoln.reset(new CatenaryHSoln(this->V, this->H, this->L));
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
MooringForce::~MooringForce() = default;

//////////////////////////////////////////////////
MooringForce::MooringForce()
: System(), dataPtr(std::make_unique<MooringForcePrivate>())
{
}

//////////////////////////////////////////////////
void MooringForce::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  // Skip debug messages to run faster TODO change to 3 before merge PR
  gz::common::Console::SetVerbosity(4);

  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "MooringForce plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->anchorPos = _sdf->Get<gz::math::Vector3d>(
    "anchor_position", this->dataPtr->anchorPos).first;
  gzdbg << "Anchor position set to " << this->dataPtr->anchorPos
    << std::endl;

  if (_sdf->HasElement("chain_length")) {
    this->dataPtr->L = _sdf->Get<double>(
      "chain_length", this->dataPtr->L).first;
    gzdbg << "Mooring chain length set to " << this->dataPtr->L
      << std::endl;
  }

  // Find necessary model links
  if (this->dataPtr->FindLinks(_ecm)) {
    this->dataPtr->UpdateVH(_ecm);
  }

  this->dataPtr->B.resize(1U);

  // debug print throttle, default 1Hz
  {
    double rate(1.0);
    if (_sdf->HasElement("debug_print_rate")) {
      rate = _sdf->Get<double>(
        "debug_print_rate", rate).first;
      gzdbg << "Debug print rate set to " << rate
            << std::endl;
    }
    std::chrono::duration<double> period{rate > 0.0 ? 1.0 / rate : 0.0};
    this->dataPtr->debugPrintPeriod = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period);
  }

}

//////////////////////////////////////////////////
void MooringForce::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("MooringForce::PreUpdate");

  // Skip if buoy link is not valid.
  if (!this->dataPtr->link.Valid(_ecm)) {
    return;
  }

  // \todo(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    gzwarn << "Detected jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() <<
      "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  // Update V and H based on latest buoy position
  this->dataPtr->UpdateVH(_ecm);

  // Skip solver if the chain can drop vertically (within tolerance).
  double toleranceL = 0.1;
  if (this->dataPtr->V + this->dataPtr->H <= this->dataPtr->L + toleranceL) {
    if (!this->dataPtr->notifiedSkipForceUpdate) {
      this->dataPtr->notifiedSkipForceUpdate = true;
      gzmsg << "Buoy chain can drop vertically. Skipping force update."
            << std::endl;
    }

    /* Tz unused at the moment, so no force to apply
    // Assume all force is vertical.
    double Tz = - this->dataPtr->w * this->dataPtr->V;

    gz::math::Vector3d force(0.0, 0.0, Tz);
    gz::math::Vector3d torque = gz::math::Vector3d::Zero;
    if (force.IsFinite() && torque.IsFinite()) {
      // this->dataPtr->link.SetVisualizationLabel("Mooring");
      this->dataPtr->link.AddWorldWrench(_ecm, force, torque);
    }
    else {
      gzerr << "Mooring force and/or torque is not finite.\n";
    }
    */
    return;
  }
  this->dataPtr->notifiedSkipForceUpdate = false;

  Eigen::HybridNonLinearSolver<CatenaryHSoln> catenarySolver(
    *this->dataPtr->catenarySoln);
  // Tolerance for error between two consecutive iterations
  catenarySolver.parameters.xtol = 0.001;
  // Max number of calls to the function
  catenarySolver.parameters.maxfev = 20;
  catenarySolver.diag.setConstant(1, 1.0);
  // Improves solution stability dramatically.
  catenarySolver.useExternalScaling = true;

  // Initial estimate for B (upper bound).
  auto BMax = [](double V, double H, double L) -> double {
    return (L * L - (V * V + H * H)) / (2 * (L - H));
  };

  double bMax = BMax(this->dataPtr->V, this->dataPtr->H, this->dataPtr->L);

  this->dataPtr->B[0] = bMax;
  int solverInfo = catenarySolver.solveNumericalDiff(this->dataPtr->B);

  double c = CatenaryFunction::CatenaryScalingFactor(
    this->dataPtr->V, this->dataPtr->B[0U], this->dataPtr->L);

  // Horizontal component of chain tension, in Newtons
  // Force at buoy heave cone is Fx = -Tx
  double Tr = - c * this->dataPtr->w;
  double Tx = Tr * cos(this->dataPtr->theta);
  double Ty = Tr * sin(this->dataPtr->theta);
  // Vertical component of chain tension at buoy heave cone, in Newtons.
  // Unused at the moment
  double Tz = - this->dataPtr->w * (this->dataPtr->L - this->dataPtr->B[0U]);

  // Throttle update rate
  auto elapsed = _info.simTime - this->dataPtr->lastDebugPrintTime;
  if (elapsed > std::chrono::steady_clock::duration::zero() &&
      elapsed >= this->dataPtr->debugPrintPeriod) {
    this->dataPtr->lastDebugPrintTime = _info.simTime;
    gzdbg << "HSolver solverInfo: " << solverInfo << "\n"
      << " t: " << std::chrono::duration_cast<
          std::chrono::milliseconds>(_info.simTime).count() / 1000.0 << "\n"
      << " Anchor: " << this->dataPtr->anchorPos << "\n"
      << " Buoy:   " << this->dataPtr->linkPos << "\n"
      << " L: " << this->dataPtr->L << "\n"
      << " V: " << this->dataPtr->V << "\n"
      << " H: " << this->dataPtr->H << "\n"
      << " b: " << bMax << "\n"
      << " B: " << this->dataPtr->B[0U] << "\n"
      << " c: " << c << "\n"
      << " theta: " << this->dataPtr->theta << "\n"
      << " Tx: " << Tx << "\n"
      << " Ty: " << Ty << "\n"
      << " Tr: " << Tr << "\n"
      << " Tz: " << Tz << "\n"
      << " nfev: " << catenarySolver.nfev << "\n"
      << " iter: " << catenarySolver.iter << "\n"
      << " fnorm: " << catenarySolver.fnorm << "\n"
      << std::endl;
  }

  // Did not find solution.
  // Maybe shouldn't apply a force that doesn't make sense
  if (solverInfo != 1) {
    gzerr << "HSolver failed to converge, solverInfo: " << solverInfo
          << " No mooring force will be applied." << std::endl;
    return;
  }

  // Apply forces to buoy heave cone link, where the mooring would be attached
  // Tz unused at the moment
  gz::math::Vector3d force(Tx, Ty, 0.0);
  gz::math::Vector3d torque = gz::math::Vector3d::Zero;
  if (force.IsFinite()) {
    // this->dataPtr->link.SetVisualizationLabel("MooringForce");
    this->dataPtr->link.AddWorldWrench(_ecm, force, torque);
  }
  else {
    gzerr << "Mooring force is not finite" << std::endl;
  }
}
}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::MooringForce,
  gz::sim::System,
  buoy_gazebo::MooringForce::ISystemConfigure,
  buoy_gazebo::MooringForce::ISystemPreUpdate)
