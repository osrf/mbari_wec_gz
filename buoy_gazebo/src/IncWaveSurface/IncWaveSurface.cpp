// Copyright 2026 Monterey Bay Aquarium Research Institute
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

#include "IncWaveSurface.hpp"

#include <gz/msgs/double_v.pb.h>

#include <algorithm>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

#include <IncidentWaves/IncWaveState.hpp>

#include "IncWaveComponents.hpp"


namespace buoy_gazebo
{
class IncWaveSurfacePrivate
{
public:
  /// \brief Service callback
  bool OnComponents(gz::msgs::Double_V & _reply);

  gz::sim::Entity IncWaveEntity{gz::sim::kNullEntity};

  gz::transport::Node node;

  /// \brief Empty until Configure succeeds
  std::string serviceName;

  /// \brief Components packed; sim thread only
  bool packed{false};

  std::mutex mutex;

  // Guarded by mutex
  bool ready{false};
  gz::msgs::Double_V components;
};

//////////////////////////////////////////////////
bool IncWaveSurfacePrivate::OnComponents(gz::msgs::Double_V & _reply)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  if (!this->ready) {
    return false;
  }
  _reply = this->components;
  return true;
}

//////////////////////////////////////////////////
IncWaveSurface::IncWaveSurface()
: dataPtr(std::make_unique<IncWaveSurfacePrivate>())
{
}

//////////////////////////////////////////////////
IncWaveSurface::~IncWaveSurface() = default;

//////////////////////////////////////////////////
void IncWaveSurface::Configure(
  const gz::sim::Entity & /*_entity*/,
  const std::shared_ptr<const sdf::Element> & /*_sdf*/,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  auto worldName = _ecm.Component<gz::sim::components::Name>(gz::sim::worldEntity(_ecm));
  if (worldName == nullptr) {
    gzerr << "IncWaveSurface: world has no name. Not serving wave components." << std::endl;
    return;
  }

  const std::string service = IncWaveComponentsService(worldName->Data());
  if (!this->dataPtr->node.Advertise(
      service, &IncWaveSurfacePrivate::OnComponents, this->dataPtr.get()))
  {
    gzerr << "IncWaveSurface: failed to advertise [" << service << "]" << std::endl;
    return;
  }
  this->dataPtr->serviceName = service;
}

//////////////////////////////////////////////////
void IncWaveSurface::PostUpdate(
  const gz::sim::UpdateInfo & /*_info*/,
  const gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("IncWaveSurface::PostUpdate");
  // Not configured, or done: the wave field doesn't change after IncidentWaves
  // configures it
  if (this->dataPtr->serviceName.empty() || this->dataPtr->packed) {
    return;
  }

  if (this->dataPtr->IncWaveEntity == gz::sim::kNullEntity) {
    this->dataPtr->IncWaveEntity =
      _ecm.EntityByComponents(gz::sim::components::Name("IncidentWaves"));
    if (this->dataPtr->IncWaveEntity == gz::sim::kNullEntity) {
      return;
    }
  }

  auto inc_wave_state_comp =
    _ecm.Component<buoy_gazebo::components::IncWaveState>(this->dataPtr->IncWaveEntity);
  if (inc_wave_state_comp == nullptr) {
    return;
  }

  const LinearIncidentWave & inc = inc_wave_state_comp->Data().Inc;
  IncWaveComponents waves;
  waves.maxFreqHz = MAX_FREQ;
  std::size_t numComponents = 0;
  for (int n = 0; n < inc.NumWaveComponents; ++n) {
    IncWaveEntry entry;
    entry.beta = inc.m_beta[n];
    auto copy = [](const Eigen::VectorXd & _from, std::vector<double> & _to)
      {
        _to.assign(_from.data(), _from.data() + _from.size());
      };
    copy(inc.m_A[n], entry.A);
    copy(inc.m_omega[n], entry.omega);
    copy(inc.m_k[n], entry.k);
    copy(inc.m_phases[n], entry.phase);
    numComponents += entry.A.size();
    waves.entries.push_back(std::move(entry));
  }

  gz::msgs::Double_V msg;
  PackIncWaveComponents(waves, msg);
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->components = std::move(msg);
    this->dataPtr->ready = true;
  }
  this->dataPtr->packed = true;

  gzdbg << "IncWaveSurface serving " << waves.entries.size() << " direction(s), "
        << numComponents << " wave components on [" << this->dataPtr->serviceName
        << "]" << std::endl;
}

}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::IncWaveSurface,
  gz::sim::System,
  buoy_gazebo::IncWaveSurface::ISystemConfigure,
  buoy_gazebo::IncWaveSurface::ISystemPostUpdate);
