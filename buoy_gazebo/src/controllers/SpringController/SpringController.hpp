#ifndef CONTROLLERS__SPRINGCONTROLLER__SPRINGCONTROLLER_HPP_
#define CONTROLLERS__SPRINGCONTROLLER__SPRINGCONTROLLER_HPP_

#include <memory>
#include <ignition/gazebo/System.hh>

namespace buoy_gazebo
{
  // Forward declarations.
  struct SpringControllerPrivate;

  /// SDF parameters:
  /// * `<namespace>`: Namespace for ROS node, defaults to sensor scoped name
  /// * `<topic>`: ROS topic to publish to, defaults to "sc_record"
  class SpringController:
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPostUpdate
  {
    /// \brief Constructor
    public: SpringController();

    /// \brief Destructor
    public: ~SpringController() override;

    // Documentation inherited
    public: void Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                const ignition::gazebo::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<SpringControllerPrivate> dataPtr;
  };
}  // buoy_gazebo

#endif  // CONTROLLERS__SPRINGCONTROLLER__SPRINGCONTROLLER_HPP_
