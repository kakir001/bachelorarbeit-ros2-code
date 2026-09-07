// myCobot 280 JN ros2_control SystemInterface.
//
// We do NOT embed Python here — the Galactic pybind11 vendor (2.2.7) segfaults
// when worker threads share the GIL with the main thread (see
// feedback_pybind_galactic_threading memory). Instead we fork a Python
// subprocess (mycobot_bridge.py) that owns pymycobot, and talk to it over a
// Unix socket. The C++ side stays pure POSIX.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace mycobot_hardware
{

class MyCobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyCobotHardware)

  ::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  ::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  ::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  ::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  ::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

private:
  static constexpr size_t kArmDof = 6;
  static constexpr size_t kGripperIdx = 6;

  // ros2_control storage (7 joints: 6 arm + 1 gripper)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_position_cmds_;
  std::vector<double> last_sent_cmds_;

  // Parameters
  std::string port_;
  int baud_ = 1000000;
  int command_speed_ = 30;
  double write_period_s_ = 0.05;
  double change_threshold_rad_ = 0.001;
  std::string socket_path_;
  std::string bridge_script_;     // absolute path to mycobot_bridge.py

  // Subprocess + socket
  pid_t bridge_pid_ = -1;
  int sock_fd_ = -1;
  std::atomic<bool> bridge_ready_{false};

  // Read thread (just drains the socket, no Python in this process)
  std::thread read_thread_;
  std::atomic<bool> read_running_{false};
  std::mutex state_mutex_;
  std::vector<double> latest_arm_;
  double latest_gripper_ = 0.0;
  std::atomic<bool> latest_fresh_{false};

  // Timing
  rclcpp::Time last_write_;
  rclcpp::Time last_read_;

  // Helpers
  bool start_bridge();
  bool connect_socket();
  void stop_bridge();
  void read_loop();
  bool send_line(const std::string & line);
};

}  // namespace mycobot_hardware
