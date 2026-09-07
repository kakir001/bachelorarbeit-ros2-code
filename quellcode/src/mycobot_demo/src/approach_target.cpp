// myCobot 280 JN — Schrauben-/Ziel-ANNAEHERUNG (nur pre-grasp, KEIN descend/greifen).
//
// Benutzerwunsch (2026-06-07):
//   1) Roboter startet IMMER vom Punkt 0 (SRDF "home" = alle Gelenke 0).
//   2) Genau über die vom YOLO-Detector gewählte Schraube/Ziel, Greifer 90° SENKRECHT
//      (top-down), 10 cm über dem Ziel anhalten und den Greifer OEFFNEN.
//   Hier wird angehalten — KEIN Abstieg / Schließen / Place. (pick_place_cartesian macht das.)
//
// RViz: Ziel wird mit /approach/target_marker (grüne Kugel) + dem /vida/target_marker
//   des Detectors angezeigt; die Planungs-Vorschau ist über /display_planned_path sichtbar.
//
// Frame: robot_base (planning), tcp (end-effector). Greifer direkt über Controller.

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using moveit::planning_interface::MoveGroupInterface;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Marker = visualization_msgs::msg::Marker;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperClient = rclcpp_action::Client<FollowJointTrajectory>;

namespace
{
constexpr double EEF_STEP        = 0.005;
constexpr double VEL_SCALE       = 0.20;   // Sicherheit: 20% Geschwindigkeit
constexpr double ACC_SCALE       = 0.20;
constexpr double TARGET_WAIT_SEC = 30.0;   // Warten auf /vida/target
}

// Greifer schaut nach unten: tcp +Y -> Welt -Z. yaw = Azimut, tilt = Abweichung von der Senkrechten.
// tilt=0 -> direkt nach unten (90° senkrecht). (exakt gleiche Konvention wie pick_place_cartesian.)
static geometry_msgs::msg::Quaternion gripperOrientation(double yaw, double tilt)
{
  tf2::Quaternion q;
  q.setRPY(-M_PI_2 + tilt, 0.0, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

// Versucht die Pre-grasp Pose mit einem yaw-Raster (tilt=0, direkt nach unten); wendet den ersten planbaren+
// ausführbaren an. Wenn die Schraube armnah/fern ist, ist die reine senkrechte Einzel-Pose in der IK evtl. nicht lösbar;
// durch yaw-Drehen suchen wir eine erreichbare Konfiguration (KEIN descend — Ausrichtung wird nicht weitergegeben).
static bool approachTopDown(
    MoveGroupInterface& arm, const Pose& pre_pos,
    const rclcpp::Logger& log)
{
  const double yaw_out = std::atan2(pre_pos.position.y, pre_pos.position.x);
  const std::vector<double> yaws{
    yaw_out, yaw_out + M_PI_2, yaw_out - M_PI_2,
    yaw_out + M_PI, yaw_out + M_PI_4, yaw_out - M_PI_4,
  };

  Pose target = pre_pos;
  MoveGroupInterface::Plan plan;
  for (double yaw : yaws) {
    target.orientation = gripperOrientation(yaw, 0.0);
    arm.setPoseTarget(target);
    if (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(log, "[pre-grasp] geloest: yaw=%+.0f° tilt=0° (senkrecht)",
                  yaw * 180.0 / M_PI);
      if (!arm.execute(plan)) {
        RCLCPP_ERROR(log, "[pre-grasp] execute() fehlgeschlagen");
        return false;
      }
      return true;
    }
    RCLCPP_INFO(log, "  [pre-grasp] yaw=%+.0f° nicht planbar, naechste", yaw * 180.0 / M_PI);
  }
  RCLCPP_ERROR(log, "[pre-grasp] kein yaw planbar — Ziel unerreichbar "
               "(in den gruenen Kreis legen)");
  return false;
}

// Greifer DIREKT an den Controller (FollowJointTrajectory) senden (open=0.15).
static bool setGripper(
    const std::shared_ptr<GripperClient>& client,
    const std::string& named_state, const rclcpp::Logger& log)
{
  const double pos = (named_state == "open") ? 0.15 : -0.74;
  if (!client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(log, "kein Greifer-Action-Server");
    return false;
  }
  FollowJointTrajectory::Goal goal;
  goal.trajectory.joint_names = {"gripper_controller"};
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = {pos};
  pt.time_from_start = rclcpp::Duration::from_seconds(1.0);
  goal.trajectory.points = {pt};

  auto goal_fut = client->async_send_goal(goal);
  if (goal_fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(log, "Greifer-Goal konnte nicht gesendet werden"); return false;
  }
  auto handle = goal_fut.get();
  if (!handle) { RCLCPP_ERROR(log, "Greifer-Goal abgelehnt"); return false; }
  auto res_fut = client->async_get_result(handle);
  if (res_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_WARN(log, "Greifer-Ergebnis Zeitueberschreitung (trotzdem weiter)");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  RCLCPP_INFO(log, "Greifer -> %s (pos=%.2f) OK", named_state.c_str(), pos);
  return true;
}

static Marker makeTargetMarker(const geometry_msgs::msg::Point& p)
{
  Marker m;
  m.header.frame_id = "robot_base";
  m.ns = "approach_target";
  m.id = 0;
  m.type = Marker::SPHERE;
  m.action = Marker::ADD;
  m.pose.position = p;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = 0.02;  // 2 cm Kugel
  m.color.g = 1.0; m.color.a = 0.9;          // grün
  return m;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "approach_target",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  auto param_d = [&](const std::string& name, double def) {
    if (!node->has_parameter(name)) node->declare_parameter<double>(name, def);
    return node->get_parameter(name).as_double();
  };
  const double approach_height = param_d("approach_height", 0.10);  // 10 cm darüber

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() { executor.spin(); });

  auto marker_pub = node->create_publisher<Marker>("/approach/target_marker",
                                                   rclcpp::QoS(1).transient_local());

  // --- Zielposition vom Detector holen (/vida/target, robot_base Frame) ---
  geometry_msgs::msg::Point screw;
  std::atomic<bool> have_target{false};
  auto target_sub = node->create_subscription<PoseStamped>(
      "/vida/target", rclcpp::QoS(1),
      [&](PoseStamped::SharedPtr m) {
        screw = m->pose.position; have_target = true;  // immer das AKTUELLSTE Ziel
      });

  MoveGroupInterface arm(node, "arm");
  auto gripper_client = rclcpp_action::create_client<FollowJointTrajectory>(
      node, "/gripper_controller/follow_joint_trajectory");
  arm.setPoseReferenceFrame("robot_base");
  arm.setEndEffectorLink("tcp");
  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(VEL_SCALE);
  arm.setMaxAccelerationScalingFactor(ACC_SCALE);
  (void)EEF_STEP;

  // --- SCHRITT 1: Roboter zum Punkt 0 (SRDF home = alle Gelenke 0) ---
  RCLCPP_INFO(log, ">> SCHRITT 1: zum Punkt 0 fahren (home, alle Gelenke 0)");
  arm.setNamedTarget("home");
  if (!arm.move()) {
    RCLCPP_ERROR(log, "konnte nicht zum Punkt 0 fahren"); rclcpp::shutdown(); spinner.join(); return 1;
  }

  // --- Auf erstes Ziel warten ---
  RCLCPP_INFO(log, "warte auf /vida/target (Detector muss laufen)...");
  {
    auto t0 = node->now();
    while (rclcpp::ok() && !have_target &&
           (node->now() - t0).seconds() < TARGET_WAIT_SEC) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  if (!have_target) {
    RCLCPP_ERROR(log, "innerhalb von %.0f s kein /vida/target gekommen — laeuft Detector/Kamera?",
                 TARGET_WAIT_SEC);
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // --- SCHRITT 2: erneut versuchen BIS ERREICHT (wenn Schraube fern ist, nähert der Benutzer sie an) ---
  // Bei jedem Versuch wird das AKTUELLSTE Ziel gelesen; wenn unerreichbar, wartet der Roboter bei home und fährt
  // automatisch, sobald die Schraube in die grüne Zone gebracht wird. Bei Erfolg öffnet er den Greifer und wartet dort.
  bool reached = false;
  while (rclcpp::ok() && !reached) {
    geometry_msgs::msg::Point t = screw;          // aktuellstes Ziel
    marker_pub->publish(makeTargetMarker(t));     // Ziel in Kamera/RViz anzeigen
    const double r = std::sqrt(t.x * t.x + t.y * t.y) * 1000.0;
    RCLCPP_INFO(log, "Ziel: x=%.3f y=%.3f z=%.3f  (r=%.0fmm)", t.x, t.y, t.z, r);

    Pose pre;
    pre.position.x = t.x;
    pre.position.y = t.y;
    pre.position.z = t.z + approach_height;
    RCLCPP_INFO(log, ">> SCHRITT 2: %.0f cm ueber dem Ziel (z=%.3f), Greifer 90° senkrecht",
                approach_height * 100.0, pre.position.z);

    if (approachTopDown(arm, pre, log)) {
      RCLCPP_INFO(log, ">> Greifer oeffnen");
      setGripper(gripper_client, "open", log);
      RCLCPP_INFO(log, "✅ FERTIG — Roboter %.0fcm ueber dem Ziel, 90° senkrecht, Greifer offen. "
                  "Wartet dort (mit Ctrl+C schliessen).", approach_height * 100.0);
      reached = true;
      break;
    }
    RCLCPP_WARN(log, "⚠ Ziel UNERREICHBAR (r=%.0fmm; ~265mm Senkrecht-Limit). Schraube NAEHER an den Roboter "
                "legen (in den GRUENEN Kreis in der Kamera, r<220mm). In 5s erneut...", r);
    for (int k = 0; k < 50 && rclcpp::ok(); ++k)  // 5 s warten, Marker am Leben halten
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Nach Erfolg den Marker am Leben halten + den Node am Laufen halten. Beenden mit Ctrl+C.
  rclcpp::Rate rate(2.0);
  while (rclcpp::ok()) {
    marker_pub->publish(makeTargetMarker(screw));
    rate.sleep();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
