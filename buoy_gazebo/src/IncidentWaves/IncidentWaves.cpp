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

#include "IncidentWaves.hpp"

#include <gz/msgs/double.pb.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/Model.hh>

#include "LinearIncidentWave.hpp"
#include "IncWaveState.hpp"


namespace buoy_gazebo
{
class IncidentWavesPrivate
{
public:
  /// \brief Piston joint entity
  gz::sim::Entity IncidentWavesEntity{gz::sim::kNullEntity};

  /// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

  std::shared_ptr<LinearIncidentWave> Inc = std::make_shared<LinearIncidentWave>();

  gz::sim::Entity IncWaveEntity;
  buoy_gazebo::IncWaveState inc_wave_state;
};

//////////////////////////////////////////////////
IncidentWaves::IncidentWaves()
: dataPtr(std::make_unique<IncidentWavesPrivate>())
{
}

/////////////////////////////////////////////////
double SdfParamDouble(
  const std::shared_ptr<const sdf::Element> & _sdf,
  const std::string & _field, double _default)
{
  return _sdf->Get<double>(_field, _default).first;
}

//////////////////////////////////////////////////
void IncidentWaves::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "IncidentWaves plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  auto SpectrumType = _sdf->Get<std::string>("IncWaveSpectrumType");

//  double beta = SdfParamDouble(_sdf, "WaveDir", 180.0);  // Not yet implemented
  double beta = 180.0;

  if (!SpectrumType.compare("MonoChromatic")) {
    gzdbg << "SpectrumType " << SpectrumType << std::endl;
    double A = SdfParamDouble(_sdf, "A", 0.0);
    double T = SdfParamDouble(_sdf, "T", 14.0);
    double phase = SdfParamDouble(_sdf, "Phase", 0.0);
    this->dataPtr->Inc->SetToMonoChromatic(A, T, phase, beta);
  }

  if (!SpectrumType.compare("Bretschneider")) {
    gzdbg << "SpectrumType " << SpectrumType << std::endl;
    double Hs = SdfParamDouble(_sdf, "Hs", 0.0);
    double Tp = SdfParamDouble(_sdf, "Tp", 14.0);
    gzdbg << "Hs = " << Hs << "  Tp = " << Tp << std::endl;
    this->dataPtr->Inc->SetToBretschneiderSpectrum(Hs, Tp, beta);
  }

  if (!SpectrumType.compare("Custom")) {
    gzwarn << "SpectrumType " << SpectrumType << "Custom Defined" << std::endl;
    std::vector<double> freq;
    std::vector<double> S;

    for (int i = 0;; i++) {
      std::string f_tag = "f" + std::to_string(i);
      std::string Szz_tag = "Szz" + std::to_string(i);
      if (_sdf->HasElement(f_tag) && _sdf->HasElement(Szz_tag)) {
        freq.push_back(_sdf->Get<double>(f_tag));
        S.push_back(_sdf->Get<double>(Szz_tag));
      } else {
        break;
      }
    }

    if (freq.size() > 2) {  // \TODO(anyone):  Add more checks on validity of spectrum
      this->dataPtr->Inc->SetToCustomSpectrum(freq, S, beta);
    } else {
      gzwarn << "Ill-formed custom wave-spectrum specification, no waves added" << std::endl;
    }
  }

  this->dataPtr->inc_wave_state.Inc = this->dataPtr->Inc;
  this->dataPtr->inc_wave_state.x = 0.0;
  this->dataPtr->inc_wave_state.y = 0.0;

  this->dataPtr->IncWaveEntity = _ecm.CreateEntity();
  _ecm.SetComponentData<buoy_gazebo::components::IncWaveState>(
    this->dataPtr->IncWaveEntity, this->dataPtr->inc_wave_state);
  _ecm.SetComponentData<gz::sim::components::Name>(
    this->dataPtr->IncWaveEntity, "IncidentWaves");
}

//////////////////////////////////////////////////
void IncidentWaves::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("IncidentWaves::PreUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }
  auto SimTime = std::chrono::duration<double>(_info.simTime).count();


//  buoy_gazebo::IncWaveState inc_wave_state;
  if (_ecm.EntityHasComponentType(
      this->dataPtr->IncWaveEntity,
      buoy_gazebo::components::IncWaveState().TypeId()))
  {
    auto inc_wave_state_comp =
      _ecm.Component<buoy_gazebo::components::IncWaveState>(
      this->dataPtr->IncWaveEntity);
    this->dataPtr->inc_wave_state = buoy_gazebo::IncWaveState(inc_wave_state_comp->Data());
  }
  double deta_dx{0.0}, deta_dy{0.0};
  double eta = this->dataPtr->Inc->eta(
    this->dataPtr->inc_wave_state.x,
    this->dataPtr->inc_wave_state.y,
    SimTime, &deta_dx, &deta_dy);

  gz::msgs::Pose req;
  req.set_name("water_plane");
  req.mutable_position()->set_x(this->dataPtr->inc_wave_state.x);
  req.mutable_position()->set_y(this->dataPtr->inc_wave_state.y);
  req.mutable_position()->set_z(eta);

  double roll = atan(deta_dx);
  double pitch = atan(deta_dy);
  double yaw = 0.0;

  gz::math::Quaternion<double> q =
    gz::math::Quaternion<double>::EulerToQuaternion(roll, pitch, yaw);

  req.mutable_orientation()->set_x(q.X());
  req.mutable_orientation()->set_y(q.Y());
  req.mutable_orientation()->set_z(q.Z());
  req.mutable_orientation()->set_w(q.W());

  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
    [](const gz::msgs::Boolean & /*_rep*/, const bool _result)
    {
      if (!_result) {
        gzerr << "Error sending move to request" << std::endl;
      }
    };

  gz::transport::Node node;
  node.Request("/world/mbari_wec_world/set_pose", req, cb);

  return;
}

}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::IncidentWaves,
  gz::sim::System,
  buoy_gazebo::IncidentWaves::ISystemConfigure,
  buoy_gazebo::IncidentWaves::ISystemPreUpdate);
