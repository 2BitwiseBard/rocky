// RockySystem skeleton implementation — see header for status.
#include "rocky_driver/rocky_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rocky_driver
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

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
  // ID convention (params.yaml bus:): yaw_i,hip_i,knee_i = 3i+1..3i+3, claws 16+i
  for (size_t k = 0; k < n; ++k) {
    servo_ids_.push_back(static_cast<uint8_t>(k < 15 ? k + 1 : 16 + (k - 15)));
  }
  RCLCPP_INFO(rclcpp::get_logger("RockySystem"),
              "init: %zu joints on %s @ %d (SKELETON — serial I/O TODO)",
              n, port_.c_str(), baud_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn RockySystem::on_activate(const rclcpp_lifecycle::State &)
{
  // TODO: open serial, torque-enable all, read initial positions into
  // pos_state_ and copy into pos_cmd_ (bumpless start)
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
  std::vector<hardware_interface::StateInterface> out;
  for (size_t k = 0; k < info_.joints.size(); ++k) {
    out.emplace_back(info_.joints[k].name,
                     hardware_interface::HW_IF_POSITION, &pos_state_[k]);
    out.emplace_back(info_.joints[k].name,
                     hardware_interface::HW_IF_VELOCITY, &vel_state_[k]);
    out.emplace_back(info_.joints[k].name,
                     hardware_interface::HW_IF_EFFORT, &eff_state_[k]);
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
