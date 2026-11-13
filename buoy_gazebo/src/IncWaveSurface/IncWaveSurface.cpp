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

#include <gz/msgs/marker.pb.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector2.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

#include <IncidentWaves/IncWaveState.hpp>


namespace buoy_gazebo
{
/// \brief Service the GUI's MarkerManager plugin serves
static const char kMarkerService[] = "/marker";

class IncWaveSurfacePrivate
{
public:
  /// \brief Number of vertices along x and y
  gz::math::Vector2i gridSize{64, 64};

  /// \brief Distance between vertices along x and y (m)
  gz::math::Vector2d gridSpacing{2.0, 2.0};

  /// \brief Wall-clock time between surface updates
  std::chrono::steady_clock::duration updatePeriod{std::chrono::milliseconds(100)};

  /// \brief Wall-clock time of the last update handed to the worker
  std::chrono::steady_clock::time_point lastUpdateWallTime{};

  gz::sim::Entity IncWaveEntity{gz::sim::kNullEntity};

  gz::transport::Node node;

  /// \brief Reused between updates so the point list is allocated once
  gz::msgs::Marker markerMsg;

  /// \brief eta at each vertex, row-major in y
  std::vector<double> eta;

  std::thread worker;
  std::mutex mutex;
  std::condition_variable cv;

  // Guarded by mutex
  bool stop{false};
  bool busy{false};  // worker has an update pending or in progress
  bool sentOnce{false};
  std::chrono::steady_clock::duration lastSentSimTime{};

  // Written by PostUpdate only while the worker is idle, read by the worker
  // only while busy
  buoy_gazebo::IncWaveState job;
  std::chrono::steady_clock::duration jobSimTime{};

  void WorkerLoop();
  void UpdateMarker();
  bool MarkerServiceAvailable();
};

//////////////////////////////////////////////////
void IncWaveSurfacePrivate::WorkerLoop()
{
  std::unique_lock<std::mutex> lock(this->mutex);
  while (true) {
    this->cv.wait(lock, [this] {return this->stop || this->busy;});
    if (this->stop) {
      return;
    }
    lock.unlock();

    // Skip the work when no GUI is listening; an unanswered request would
    // stay queued in gz-transport
    const bool sent = this->MarkerServiceAvailable();
    if (sent) {
      this->UpdateMarker();
    }

    lock.lock();
    if (sent) {
      this->sentOnce = true;
      this->lastSentSimTime = this->jobSimTime;
    }
    this->busy = false;
  }
}

//////////////////////////////////////////////////
bool IncWaveSurfacePrivate::MarkerServiceAvailable()
{
  std::vector<gz::transport::ServicePublisher> publishers;
  return this->node.ServiceInfo(kMarkerService, publishers) && !publishers.empty();
}

//////////////////////////////////////////////////
void IncWaveSurfacePrivate::UpdateMarker()
{
  GZ_PROFILE("IncWaveSurface::UpdateMarker");
  const int nx = this->gridSize.X();
  const int ny = this->gridSize.Y();
  const double dx = this->gridSpacing.X();
  const double dy = this->gridSpacing.Y();
  const double t = std::chrono::duration<double>(this->jobSimTime).count();

  // Center the grid on the buoy, snapped to whole cells so the vertices stay
  // fixed in the world as the buoy moves
  const double x0 = std::round(this->job.x / dx) * dx - 0.5 * (nx - 1) * dx;
  const double y0 = std::round(this->job.y / dy) * dy - 0.5 * (ny - 1) * dy;

  for (int j = 0; j < ny; ++j) {
    for (int i = 0; i < nx; ++i) {
      this->eta[j * nx + i] = this->job.Inc.eta(x0 + i * dx, y0 + j * dy, t);
    }
  }

  int k = 0;
  auto setPoint = [&](int _i, int _j)
    {
      auto * p = this->markerMsg.mutable_point(k++);
      p->set_x(x0 + _i * dx);
      p->set_y(y0 + _j * dy);
      p->set_z(this->eta[_j * nx + _i]);
    };

  // Two triangles per cell, counter-clockwise seen from above so they face up
  for (int j = 0; j < ny - 1; ++j) {
    for (int i = 0; i < nx - 1; ++i) {
      setPoint(i, j);
      setPoint(i + 1, j);
      setPoint(i + 1, j + 1);
      setPoint(i, j);
      setPoint(i + 1, j + 1);
      setPoint(i, j + 1);
    }
  }

  auto sec_nsec = gz::math::durationToSecNsec(this->jobSimTime);
  this->markerMsg.mutable_header()->mutable_stamp()->set_sec(sec_nsec.first);
  this->markerMsg.mutable_header()->mutable_stamp()->set_nsec(sec_nsec.second);

  this->node.Request(kMarkerService, this->markerMsg);
}

//////////////////////////////////////////////////
IncWaveSurface::IncWaveSurface()
: dataPtr(std::make_unique<IncWaveSurfacePrivate>())
{
}

//////////////////////////////////////////////////
IncWaveSurface::~IncWaveSurface()
{
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    this->dataPtr->stop = true;
  }
  this->dataPtr->cv.notify_one();
  if (this->dataPtr->worker.joinable()) {
    this->dataPtr->worker.join();
  }
}

//////////////////////////////////////////////////
void IncWaveSurface::Configure(
  const gz::sim::Entity & /*_entity*/,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & /*_ecm*/,
  gz::sim::EventManager & /*_eventMgr*/)
{
  auto gridSize = _sdf->Get<gz::math::Vector2i>(
    "grid_size", this->dataPtr->gridSize).first;
  if (gridSize.X() < 2 || gridSize.Y() < 2) {
    gzerr << "IncWaveSurface: grid_size [" << gridSize
          << "] needs at least 2 vertices along x and y. Surface disabled." << std::endl;
    return;
  }

  auto gridSpacing = _sdf->Get<gz::math::Vector2d>(
    "grid_spacing", this->dataPtr->gridSpacing).first;
  if (gridSpacing.X() <= 0.0 || gridSpacing.Y() <= 0.0) {
    gzerr << "IncWaveSurface: grid_spacing [" << gridSpacing
          << "] must be positive. Surface disabled." << std::endl;
    return;
  }

  double publishRate = _sdf->Get<double>("publish_rate", 10.0).first;
  if (publishRate <= 0.0) {
    gzerr << "IncWaveSurface: publish_rate [" << publishRate
          << "] must be positive. Surface disabled." << std::endl;
    return;
  }

  this->dataPtr->gridSize = gridSize;
  this->dataPtr->gridSpacing = gridSpacing;
  this->dataPtr->updatePeriod =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / publishRate));

  this->dataPtr->eta.resize(gridSize.X() * gridSize.Y());

  auto & msg = this->dataPtr->markerMsg;
  msg.set_ns("inc_wave_surface");
  // Must be nonzero: MarkerManager gives id 0 a new random id, which would add
  // a new marker on every update instead of modifying this one
  msg.set_id(1);
  msg.set_action(gz::msgs::Marker::ADD_MODIFY);
  msg.set_type(gz::msgs::Marker::TRIANGLE_LIST);
  msg.set_visibility(gz::msgs::Marker::GUI);

  // Same look as the water_plane model
  auto setColor = [](gz::msgs::Color * _color)
    {
      _color->set_r(0.0);
      _color->set_g(0.0);
      _color->set_b(1.0);
      _color->set_a(0.5);
    };
  setColor(msg.mutable_material()->mutable_ambient());
  setColor(msg.mutable_material()->mutable_diffuse());
  setColor(msg.mutable_material()->mutable_specular());

  const int numPoints = 6 * (gridSize.X() - 1) * (gridSize.Y() - 1);
  msg.mutable_point()->Reserve(numPoints);
  for (int k = 0; k < numPoints; ++k) {
    msg.add_point();
  }

  gzdbg << "IncWaveSurface grid_size=" << gridSize
        << "  grid_spacing=" << gridSpacing
        << "  publish_rate=" << publishRate << std::endl;

  this->dataPtr->worker = std::thread(&IncWaveSurfacePrivate::WorkerLoop, this->dataPtr.get());
}

//////////////////////////////////////////////////
void IncWaveSurface::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  GZ_PROFILE("IncWaveSurface::PostUpdate");
  // Not configured
  if (!this->dataPtr->worker.joinable()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now - this->dataPtr->lastUpdateWallTime < this->dataPtr->updatePeriod) {
    return;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // Worker is still on the previous update; skip this one
  if (this->dataPtr->busy) {
    return;
  }
  // Paused: the GUI already has the surface for this sim time
  if (this->dataPtr->sentOnce && this->dataPtr->lastSentSimTime == _info.simTime) {
    return;
  }
  this->dataPtr->lastUpdateWallTime = now;

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

  this->dataPtr->job = inc_wave_state_comp->Data();
  this->dataPtr->jobSimTime = _info.simTime;
  this->dataPtr->busy = true;
  this->dataPtr->cv.notify_one();
}

}  // namespace buoy_gazebo

GZ_ADD_PLUGIN(
  buoy_gazebo::IncWaveSurface,
  gz::sim::System,
  buoy_gazebo::IncWaveSurface::ISystemConfigure,
  buoy_gazebo::IncWaveSurface::ISystemPostUpdate);
