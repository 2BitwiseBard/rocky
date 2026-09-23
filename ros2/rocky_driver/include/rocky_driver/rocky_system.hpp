// RockySystem — ros2_control SystemInterface for the Feetech bus (hw:=serial).
//
// STATUS: compilable skeleton, wire-protocol TODOs marked. The recommended
// Phase 1-2 path is hw:=topic (Python bridge over the tested driver); this
// C++ plugin exists for when the control loop wants to shed the topic hop.
// Protocol reference: driver/rocky_driver/protocol.py (golden-tested).

#ifndef ROCKY_DRIVER__ROCKY_SYSTEM_HPP_
#define ROCKY_DRIVER__ROCKY_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace rocky_driver
{

class RockySystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RockySystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // one entry per joint, ordered as in the URDF ros2_control block
  std::vector<double> pos_cmd_, pos_state_, vel_state_, eff_state_;
  std::vector<uint8_t> servo_ids_;
  std::vector<int8_t> dir_;          // calibration.yaml
  std::vector<int16_t> offset_counts_;
  std::string port_;
  int baud_{1000000};
  int fd_{-1};                       // serial fd

  // TODO(bench day + one quiet evening):
  //  - open/configure the port (termios, 1 Mbps, raw)
  //  - buildSyncWrite(addr=42, entries...) per family — byte layout is
  //    checksum'd + golden-tested in driver/tests/test_protocol.py
  //  - parse status packets (header hunt + checksum, retry once)
  //  - SYNC_READ 56..70 for STS telemetry, per-ID reads for SCS
};

}  // namespace rocky_driver

#endif  // ROCKY_DRIVER__ROCKY_SYSTEM_HPP_
