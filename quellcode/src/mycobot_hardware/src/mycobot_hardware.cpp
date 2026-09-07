// myCobot 280 JN SystemInterface — talks to mycobot_bridge.py subprocess
// over a Unix socket. All pymycobot calls live in the bridge process.

#include "mycobot_hardware/mycobot_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

extern char ** environ;
using namespace std::chrono_literals;
using hardware_interface::return_type;

namespace mycobot_hardware
{

static rclcpp::Logger logger()
{
  return rclcpp::get_logger("mycobot_hardware");
}

::CallbackReturn MyCobotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != ::CallbackReturn::SUCCESS) {
    return ::CallbackReturn::ERROR;
  }

  auto get_or = [&info](const std::string & key, const std::string & def) {
    auto it = info.hardware_parameters.find(key);
    return (it == info.hardware_parameters.end()) ? def : it->second;
  };

  port_ = get_or("port", "/dev/ttyTHS1");
  baud_ = std::stoi(get_or("baud", "1000000"));
  command_speed_ = std::stoi(get_or("command_speed", "30"));
  write_period_s_ = std::stod(get_or("write_period_s", "0.25"));
  change_threshold_rad_ = std::stod(get_or("change_threshold_rad", "0.001"));
  socket_path_ = get_or("socket_path", "/tmp/mycobot_bridge.sock");

  // Locate bridge script in install/share/...
  try {
    bridge_script_ = ament_index_cpp::get_package_prefix("mycobot_hardware")
      + "/lib/mycobot_hardware/mycobot_bridge.py";
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger(), "cannot locate mycobot_hardware prefix: %s", e.what());
    return ::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  if (n != kArmDof && n != (kArmDof + 1)) {
    RCLCPP_FATAL(logger(),
      "Expected 6 arm joints (optionally +1 gripper). Got %zu.", n);
    return ::CallbackReturn::ERROR;
  }

  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_position_cmds_.assign(n, std::numeric_limits<double>::quiet_NaN());
  last_sent_cmds_.assign(n, std::numeric_limits<double>::quiet_NaN());
  latest_arm_.assign(kArmDof, 0.0);

  RCLCPP_INFO(logger(),
    "init: port=%s baud=%d speed=%d joints=%zu socket=%s",
    port_.c_str(), baud_, command_speed_, n, socket_path_.c_str());
  return ::CallbackReturn::SUCCESS;
}

bool MyCobotHardware::start_bridge()
{
  // Spawn: python3 <bridge_script_> --port <port_> --baud <baud_>
  //                                  --socket <socket_path_> --rate 20
  std::string sport = port_;
  std::string sbaud = std::to_string(baud_);
  std::string srate = "20";
  std::string sspeed = std::to_string(command_speed_);

  std::vector<char *> argv = {
    const_cast<char *>("python3"),
    const_cast<char *>(bridge_script_.c_str()),
    const_cast<char *>("--port"),   const_cast<char *>(sport.c_str()),
    const_cast<char *>("--baud"),   const_cast<char *>(sbaud.c_str()),
    const_cast<char *>("--socket"), const_cast<char *>(socket_path_.c_str()),
    const_cast<char *>("--rate"),   const_cast<char *>(srate.c_str()),
    const_cast<char *>("--speed"),  const_cast<char *>(sspeed.c_str()),
    nullptr,
  };

  pid_t pid = 0;
  int rc = posix_spawnp(&pid, "python3", nullptr, nullptr, argv.data(), environ);
  if (rc != 0) {
    RCLCPP_FATAL(logger(), "posix_spawnp failed: %s", std::strerror(rc));
    return false;
  }
  bridge_pid_ = pid;
  RCLCPP_INFO(logger(), "bridge pid=%d (%s)", pid, bridge_script_.c_str());
  return true;
}

bool MyCobotHardware::connect_socket()
{
  // Wait for bridge to create the socket, then connect.
  for (int attempt = 0; attempt < 60; ++attempt) {  // 60 * 200ms = 12s
    struct stat st;
    if (::stat(socket_path_.c_str(), &st) == 0 && S_ISSOCK(st.st_mode)) {
      int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
      if (fd < 0) {
        RCLCPP_FATAL(logger(), "socket() failed: %s", std::strerror(errno));
        return false;
      }
      sockaddr_un addr{};
      addr.sun_family = AF_UNIX;
      std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
      if (::connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
        sock_fd_ = fd;
        RCLCPP_INFO(logger(), "connected to bridge socket");
        return true;
      }
      ::close(fd);
    }
    std::this_thread::sleep_for(200ms);
  }
  RCLCPP_FATAL(logger(), "bridge socket never appeared at %s", socket_path_.c_str());
  return false;
}

::CallbackReturn MyCobotHardware::on_configure(const rclcpp_lifecycle::State &)
{
  if (!start_bridge()) return ::CallbackReturn::ERROR;
  if (!connect_socket()) {
    stop_bridge();
    return ::CallbackReturn::ERROR;
  }

  // Start read thread now so we can prime hw_positions_ from incoming state.
  read_running_ = true;
  read_thread_ = std::thread(&MyCobotHardware::read_loop, this);

  // Wait up to 2s for first state line.
  for (int i = 0; i < 20 && !latest_fresh_; ++i) {
    std::this_thread::sleep_for(100ms);
  }
  if (latest_fresh_) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    for (size_t i = 0; i < kArmDof; ++i) {
      hw_positions_[i] = latest_arm_[i];
      hw_position_cmds_[i] = latest_arm_[i];
    }
    if (info_.joints.size() > kArmDof) {
      hw_positions_[kGripperIdx] = latest_gripper_;
    }
    RCLCPP_INFO(logger(), "primed positions from bridge");
  } else {
    RCLCPP_WARN(logger(), "no state from bridge yet — continuing");
  }
  return ::CallbackReturn::SUCCESS;
}

::CallbackReturn MyCobotHardware::on_activate(const rclcpp_lifecycle::State &)
{
  send_line("power_on\n");
  const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
  last_write_ = now;
  last_read_ = now;
  RCLCPP_INFO(logger(), "Activated.");
  return ::CallbackReturn::SUCCESS;
}

::CallbackReturn MyCobotHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  send_line("release_all\n");
  RCLCPP_INFO(logger(), "Deactivated — servos released.");
  return ::CallbackReturn::SUCCESS;
}

::CallbackReturn MyCobotHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  read_running_ = false;
  if (sock_fd_ >= 0) {
    send_line("shutdown\n");
    ::shutdown(sock_fd_, SHUT_RDWR);
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  stop_bridge();
  return ::CallbackReturn::SUCCESS;
}

void MyCobotHardware::stop_bridge()
{
  if (bridge_pid_ <= 0) return;
  ::kill(bridge_pid_, SIGTERM);
  // Wait up to 3s
  for (int i = 0; i < 30; ++i) {
    int status = 0;
    pid_t r = ::waitpid(bridge_pid_, &status, WNOHANG);
    if (r == bridge_pid_) {
      bridge_pid_ = -1;
      return;
    }
    std::this_thread::sleep_for(100ms);
  }
  ::kill(bridge_pid_, SIGKILL);
  ::waitpid(bridge_pid_, nullptr, 0);
  bridge_pid_ = -1;
}

bool MyCobotHardware::send_line(const std::string & line)
{
  if (sock_fd_ < 0) return false;
  const char * p = line.data();
  size_t left = line.size();
  while (left > 0) {
    ssize_t n = ::send(sock_fd_, p, left, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR) continue;
      RCLCPP_WARN_THROTTLE(
        logger(), *rclcpp::Clock::make_shared(RCL_STEADY_TIME).get(), 5000,
        "send failed: %s", std::strerror(errno));
      return false;
    }
    p += n;
    left -= n;
  }
  return true;
}

void MyCobotHardware::read_loop()
{
  std::string buf;
  buf.reserve(4096);
  char chunk[1024];
  while (read_running_) {
    ssize_t n = ::recv(sock_fd_, chunk, sizeof(chunk), 0);
    if (n == 0) {
      RCLCPP_WARN(logger(), "bridge closed socket");
      break;
    }
    if (n < 0) {
      if (errno == EINTR) continue;
      if (errno == EBADF) break;  // closed during cleanup
      RCLCPP_WARN_THROTTLE(
        logger(), *rclcpp::Clock::make_shared(RCL_STEADY_TIME).get(), 5000,
        "recv failed: %s", std::strerror(errno));
      std::this_thread::sleep_for(100ms);
      continue;
    }
    buf.append(chunk, static_cast<size_t>(n));

    // Process complete lines.
    size_t pos;
    while ((pos = buf.find('\n')) != std::string::npos) {
      std::string line = buf.substr(0, pos);
      buf.erase(0, pos + 1);
      if (line.empty()) continue;

      // "state <ts> r1..r6 grip"
      std::istringstream iss(line);
      std::string tag;
      iss >> tag;
      if (tag != "state") continue;

      double ts;
      iss >> ts;
      std::vector<double> r(kArmDof);
      bool ok = true;
      for (size_t i = 0; i < kArmDof; ++i) {
        if (!(iss >> r[i])) { ok = false; break; }
      }
      if (!ok) continue;
      int grip = -1;
      iss >> grip;

      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        latest_arm_ = r;
        // pymycobot returns 0-100; values like 255/-1 mean "servo not responding"
        // and must be rejected, otherwise 255 -> 1.785 rad rotates the mimic
        // finger links out of view. Hold the last valid value on a bad read.
        if (grip >= 0 && grip <= 100) {
          latest_gripper_ = (static_cast<double>(grip) / 100.0) * 0.7;
        }
      }
      latest_fresh_ = true;
    }
  }
}

std::vector<hardware_interface::StateInterface>
MyCobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface>
MyCobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    out.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_position_cmds_[i]);
  }
  return out;
}

return_type MyCobotHardware::read()
{
  if (!latest_fresh_) return return_type::OK;
  const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
  const double dt = std::max((now - last_read_).seconds(), 1e-3);
  last_read_ = now;

  std::vector<double> arm(kArmDof);
  double gripper;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    for (size_t i = 0; i < kArmDof; ++i) arm[i] = latest_arm_[i];
    gripper = latest_gripper_;
  }
  for (size_t i = 0; i < kArmDof; ++i) {
    hw_velocities_[i] = (arm[i] - hw_positions_[i]) / dt;
    hw_positions_[i] = arm[i];
  }
  if (info_.joints.size() > kArmDof) {
    hw_velocities_[kGripperIdx] = (gripper - hw_positions_[kGripperIdx]) / dt;
    hw_positions_[kGripperIdx] = gripper;
  }
  return return_type::OK;
}

return_type MyCobotHardware::write()
{
  const auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
  if ((now - last_write_).seconds() < write_period_s_) {
    return return_type::OK;
  }

  std::vector<double> arm_cmd(kArmDof);
  bool any_change = false;
  for (size_t i = 0; i < kArmDof; ++i) {
    const double c = hw_position_cmds_[i];
    if (!std::isfinite(c)) {
      arm_cmd[i] = hw_positions_[i];  // hold
      continue;
    }
    arm_cmd[i] = c;
    if (!std::isfinite(last_sent_cmds_[i]) ||
        std::abs(c - last_sent_cmds_[i]) > change_threshold_rad_)
    {
      any_change = true;
    }
  }
  if (!any_change) return return_type::OK;

  std::ostringstream oss;
  oss.precision(6);
  oss << std::fixed << "send_radians";
  for (size_t i = 0; i < kArmDof; ++i) oss << ' ' << arm_cmd[i];
  oss << ' ' << command_speed_ << '\n';
  send_line(oss.str());

  if (info_.joints.size() > kArmDof) {
    const double c = hw_position_cmds_[kGripperIdx];
    if (std::isfinite(c)) {
      int g = static_cast<int>(std::clamp(c / 0.7, 0.0, 1.0) * 100.0);
      std::ostringstream gg;
      gg << "set_gripper " << g << ' ' << command_speed_ << '\n';
      send_line(gg.str());
    }
  }

  for (size_t i = 0; i < kArmDof; ++i) last_sent_cmds_[i] = arm_cmd[i];
  last_write_ = now;
  return return_type::OK;
}

}  // namespace mycobot_hardware

PLUGINLIB_EXPORT_CLASS(
  mycobot_hardware::MyCobotHardware, hardware_interface::SystemInterface)
