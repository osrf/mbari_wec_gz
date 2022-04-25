#ifndef IGNITION_GAZEBO_SYSTEMS_ELECTROHYDRAULICPTO_HH_
#define IGNITION_GAZEBO_SYSTEMS_ELECTROHYDRAULICPTO_HH_

#include <ignition/gazebo/System.hh>
#include <ignition/transport.hh>
#include <memory>
#include <optional>

namespace buoy_gazebo
{
// enum class SpringType { linear, pneumatic_adiabatic, pneumatic_calibrated};

// Forward declaration
class ElectroHydraulicPTOPrivate;

/// \brief To use, several parameters are required.
/// Two ignition gazebo joints that are either prismatic or continous with 1DOF each, a desciption of
/// the connectoins between actuator (joints), and an oil characteristic specification.
///
/// ## System Parameters
///
/// xml tags in Ignition Gazebo .sdf file define behavior as follows:
///
/// \brief <PrismaticJointName>
///
///  For each actuator of prismatic type, the following nested tags are required:
///     <Area_A>  Piston area on A end of cylinder
///
///     <Area_B>  Piston area on B end of Cylinder
///
///
/// \brief <RevoluteJointName>
///   For each actuator of revolute type, the following nested tags are required:
///
///     <Displacement>  Displacement per revolution of rotary pump/motor.
class ElectroHydraulicPTO
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate,
      public ignition::gazebo::ISystemUpdate,
      public ignition::gazebo::ISystemPostUpdate
{
  /// \brief Constructor
  public: ElectroHydraulicPTO();

  /// \brief Destructor
  public: ~ElectroHydraulicPTO() override = default;

  // Documentation inherited
  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &_eventMgr) override;

  // Documentation inherited
  public: void PreUpdate(
              const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void Update(
              const ignition::gazebo::UpdateInfo &_info,
              ignition::gazebo::EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void PostUpdate(
              const ignition::gazebo::UpdateInfo &_info,
              const ignition::gazebo::EntityComponentManager &_ecm) override;


private:
  ignition::transport::Node node;
  ignition::transport::Node::Publisher pistonvel_pub, rpm_pub, deltaP_pub, targwindcurr_pub, windcurr_pub,
                                       battcurr_pub, loadcurr_pub, scalefactor_pub, retractfactor_pub;

  /// \brief Private data pointer
  private: std::unique_ptr<ElectroHydraulicPTOPrivate> dataPtr;
};
} // namespace buoy_gazebo

#endif
