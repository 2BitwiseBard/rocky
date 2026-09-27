// RockySystem skeleton implementation — see header for status.
#include "rocky_driver/rocky_system.hpp"

#include <cstdint>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rocky_driver
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
// Servo id from the joint NAME (params.yaml bus:): yaw<i>, hip<i>, knee<i> ->
// 3i+1, 3i+2, 3i+3; claw<i> -> 16+i. The ros2_control block interleaves
// claw<i> after knee<i>, so the joint index must not be used. Returns 0 for a
// name it does not know (0 is never a bus id here).
uint8_t servo_id_for(const std::string & name)
{
  const auto cut = name.find_last_not_of("0123456789");
  if (cut == std::string::npos || cut + 1 >= name.size()) {
    return 0;
  }
  const std::string kind = name.substr(0, cut + 1);
  const int leg = std::stoi(name.substr(cut + 1));
  if (leg < 0 || leg > 4) {
    return 0;
  }
  if (kind == "yaw") {return static_cast<uint8_t>(3 * leg + 1);}
  if (kind == "hip") {return static_cast<uint8_t>(3 * leg + 2);}
  if (kind == "knee") {return static_cast<uint8_t>(3 * leg + 3);}
  if (kind == "claw") {return static_cast<uint8_t>(16 + leg);}
  return 0;
}
}  // namespace

CallbackReturn RockySystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  port_ = info_.hardware_parameters.count("port")
    ? info_.hardware_parameters.at("port") : "/dev/ttyACM0";
  if (info_.hardware_parameters.count("baud")) {
    baud_ = std::stoi(info_.hardware_parameters.at("baud"));
  }
  const auto n = info_.joints.size();
  pos_cmd_.assign(n, 0.0);
  pos_state_.assign(n, 0.0);
  vel_state_.assign(n, 0.0);
  eff_state_.assign(n, 0.0);
  servo_ids_.clear();
  for (size_t k = 0; k < n; ++k) {
    const uint8_t id = servo_id_for(info_.joints[k].name);
    if (id == 0) {
      RCLCPP_ERROR(rclcpp::get_logger("RockySystem"),
                   "joint '%s' is not yaw<i>/hip<i>/knee<i>/claw<i> (i = 0..4)",
                   info_.joints[k].name.c_str());
      return CallbackReturn::ERROR;
    }
    servo_ids_.push_back(id);
  }
  RCLCPP_INFO(rclcpp::get_logger("RockySystem"),
              "init: %zu joints on %s @ %d (SKELETON — serial I/O TODO)",
              n, port_.c_str(), baud_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RockySystem::on_activate(const rclcpp_lifecycle::State &)
{
  // TODO, in the D052 V2 soft-enable order (rocky_driver.soft_enable) — a
  // Feetech servo enables toward its LAST goal, so never torque on first:
  //   open serial -> read present positions into pos_state_ -> TORQUE_LIMIT
  //   400 / ACC 10 -> park GOAL_POSITION at present (slow speed) -> torque on
  //   -> copy pos_state_ into pos_cmd_ (bumpless) -> release the limits once
  //   the first commanded move has landed.
  for (size_t k = 0; k < pos_cmd_.size(); ++k) {pos_cmd_[k] = pos_state_[k];}
  return CallbackReturn::SUCCESS;
}

CallbackReturn RockySystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // TODO: torque off (limp) — never leave servos fighting gravity
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RockySystem::export_state_interfaces()
{
  // export exactly what the ros2_control block declares: position + velocity
  // + effort on the leg joints, position only on the claws
  std::vector<hardware_interface::StateInterface> out;
  for (size_t k = 0; k < info_.joints.size(); ++k) {
    const auto & joint = info_.joints[k];
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        out.emplace_back(joint.name, hardware_interface::HW_IF_POSITION, &pos_state_[k]);
      } else if (si.name == hardware_interface::HW_IF_VELOCITY) {
        out.emplace_back(joint.name, hardware_interface::HW_IF_VELOCITY, &vel_state_[k]);
      } else if (si.name == hardware_interface::HW_IF_EFFORT) {
        out.emplace_back(joint.name, hardware_interface::HW_IF_EFFORT, &eff_state_[k]);
      }
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> RockySystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t k = 0; k < info_.joints.size(); ++k) {
    out.emplace_back(info_.joints[k].name,
                     hardware_interface::HW_IF_POSITION, &pos_cmd_[k]);
  }
  return out;
}

return_type RockySystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: SYNC_READ addr 56 len 15 for STS ids; per-id READ for SCS;
  // decode little/big-endian per family; apply dir/offset calibration.
  // Until then: echo commands (mock-like behavior keeps controllers alive).
  pos_state_ = pos_cmd_;
  return return_type::OK;
}

return_type RockySystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: two SYNC_WRITEs (STS then SCS) at addr 42, layouts + checksums as
  // golden-tested in driver/tests/test_protocol.py.
  return return_type::OK;
}

}  // namespace rocky_driver

PLUGINLIB_EXPORT_CLASS(rocky_driver::RockySystem,
                       hardware_interface::SystemInterface)
