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
  gz::math::Vector3d anchorPos{20.0, 0.0, -77.0};

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

  /// \brief Distance of buoy from anchor, beyond which (i.e. H > radius)
  /// mooring force is applied. Within the radius, no or negligible catenary
  /// curve is formed, and no mooring force will be applied.
  double effectiveRadius = 90.0;

  /// \brief N/m, weight of chain per unit length
  double w = 20.0;

  /// \brief radians, atan2 angle of buoy from anchor
  double theta = 0.0;

  /// \brief Catenary equation to pass to solver
  std::unique_ptr<CatenaryHSoln> catenarySoln{new CatenaryHSoln(V, H, L)};

  /// \brief Solution to catenary equation. Meters, length of chain laying on
  /// the bottom, start of catenary.
  Eigen::VectorXd B{};

  /// \brief If we have notified when inside effective radius.
  public: bool notifiedInsideEffectiveRadius{false};

  /// \brief Debug print period calculated from <debug_print_rate>
  public: std::chrono::steady_clock::duration debugPrintPeriod{0};

  /// \brief Last debug print simulation time
  public: std::chrono::steady_clock::duration lastDebugPrintTime{0};

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
      gzwarn << "Could not find valid buoy link. Mooring force may "
        << "not be calculated correctly." << std::endl;
      return false;
    }
  }
  else {
    gzwarn << "Could not find valid buoy link. Mooring force may "
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
      gzwarn << "Could not find valid heave cone link. Mooring force may "
        << "not be applied correctly." << std::endl;
      return false;
    }
  }
  else {
    gzwarn << "Could not find valid heave cone link. Mooring force may "
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

  // Update vertical (z) distance between buoy and anchor
  this->V = fabs(this->buoyPos[2U] - this->anchorPos[2U]);

  // Update horizontal distance between buoy and anchor
  this->H = sqrt(
    (this->buoyPos[0U] - this->anchorPos[0U]) *
    (this->buoyPos[0U] - this->anchorPos[0U]) +
    (this->buoyPos[1U] - this->anchorPos[1U]) *
    (this->buoyPos[1U] - this->anchorPos[1U]));

  // Update angle between buoy and anchor
  this->theta = atan2(this->buoyPos[1U] - this->anchorPos[1U],
    this->buoyPos[0U] - this->anchorPos[0U]);

  this->catenarySoln.reset(new CatenaryHSoln(this->V, this->H, this->L));
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
    gzerr << "MooringForce plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->anchorPos = _sdf->Get<gz::math::Vector3d>(
    "anchor_position", this->dataPtr->anchorPos).first;
  gzdbg << "Anchor position set to " << this->dataPtr->anchorPos
    << std::endl;

  this->dataPtr->effectiveRadius = _sdf->Get<double>(
    "enable_beyond_radius", this->dataPtr->effectiveRadius).first;
  gzdbg << "Effective radius set to beyond " << this->dataPtr->effectiveRadius
    << std::endl;

  if (_sdf->HasElement("chain_length"))
  {
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
    if (_sdf->HasElement("debug_print_rate"))
    {
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
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("MooringForce::PreUpdate");

  // If necessary links have not been identified yet, the plugin is disabled
  if (this->dataPtr->heaveConeLinkEnt == gz::sim::kNullEntity ||
    this->dataPtr->buoyLinkEnt == gz::sim::kNullEntity)
  {
    this->dataPtr->FindLinks(_ecm);
    gzerr << "Could not find heave cone and buoy links in ECM.\n";
    return;
  }

  // TODO(anyone): Support rewind
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
  // If buoy is not far enough from anchor for there to be a need to pull it
  // back, no need to apply mooring force
  if (this->dataPtr->H < this->dataPtr->effectiveRadius)
  {
    if (!this->dataPtr->notifiedInsideEffectiveRadius)
    {
      this->dataPtr->notifiedInsideEffectiveRadius = true;
      gzmsg << "Buoy horizontal radius H [" << this->dataPtr->H << "]"
            << " is inside effective radius R ["
            << this->dataPtr->effectiveRadius << "]"
            << " skipping force update.\n";
    }
    return;
  }
  this->dataPtr->notifiedInsideEffectiveRadius = false;

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
  auto BMax = [](double V, double H, double L) -> double
  {
    return (L*L - (V*V + H*H)) / (2 * (L - H));
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
      elapsed >= this->dataPtr->debugPrintPeriod)
  {
    this->dataPtr->lastDebugPrintTime = _info.simTime;
    gzdbg << "HSolver solverInfo: " << solverInfo << "\n"
      << " t: " << std::chrono::duration_cast<
          std::chrono::milliseconds>(_info.simTime).count()/1000.0 << "\n"
      << " Anchor: " << this->dataPtr->anchorPos << "\n"
      << " Buoy:   " << this->dataPtr->buoyPos << "\n"
      << " R: " << this->dataPtr->effectiveRadius << "\n"
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
      << "\n";
  }

  // Did not find solution. Maybe shouldn't apply a force that doesn't make sense
  if (solverInfo != 1) {
    gzerr << "HSolver failed to converge, solverInfo: " << solverInfo << "\n";
    return;
  }

  // Apply forces to buoy heave cone link, where the mooring would be attached
  gz::math::Vector3d force(Tx, Ty, 0.0);
  gz::math::Vector3d torque = gz::math::Vector3d::Zero;
  if (force.IsFinite())
  {
    // this->dataPtr->buoyLink.SetVisualizationLabel("MooringForce");
    this->dataPtr->buoyLink.AddWorldWrench(_ecm, force, torque);
  }
  else
  {
    gzerr << "Mooring force is not finite\n";
  }
}
}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::MooringForce,
  gz::sim::System,
  buoy_gazebo::MooringForce::ISystemConfigure,
  buoy_gazebo::MooringForce::ISystemPreUpdate)
