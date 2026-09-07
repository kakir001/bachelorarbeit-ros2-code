// myCobot 280 JN — Schrauben Pick & Place (Cartesian Annäherung + orientation constraint).
//
// Ablauf (exakt nach Herstellerempfehlung — 3-stufiges Greifen):
//   /vida/target (Detector) → Pick-Position
//   home → joint plan → pick_pre (10cm darüber)   ← PRE-GRASP (senkrechte Ziel-Pose)
//          ↓ cartesian descend                ← LINEAR DESCENT (Ausrichtung konstant)
//        pick → gripper close                 ← CLOSE GRIPPER
//          ↑ cartesian retreat
//        pick_post → joint plan → place_pre (10cm darüber)
//          ↓ cartesian descend
//        place → gripper open
//          ↑ cartesian retreat → home
//
// Greifer senkrecht (90°) halten — Lösung OHNE orientation PATH CONSTRAINT:
//   - die ZIEL-Pose von pre-grasp/place_pre ist bereits mit gripperDown() senkrecht definiert,
//     daher endet der joint plan in einer senkrechten Konfiguration.
//   - Abstieg/Aufstieg werden mit computeCartesianPath gemacht; zwischen zwei Waypoints
//     wird die Ausrichtung ohnehin konstant (senkrecht) gehalten.
//   - Deshalb wird auf die free-space joint plans KEIN orientation constraint gelegt:
//     RRTConnect + rejection-sampling beschränkte Planung (projected state space
//     nicht definiert) lief sekundenlang in timeout/abort (Notizen Sitzung 17b).
//     Ohne Constraint ist die Planung schnell und zuverlässig.
//
// Notizen:
//   - Frame: robot_base (planning), tcp (end-effector).
//   - SRDF named states: arm/home, arm/ready, gripper/open, gripper/closed.
//   - TCP +Y = Annäherungsachse; wenn der Greifer nach unten schaut, +Y → Welt -Z.
//     Yaw (Drehung um die Schraube) frei; seed wird mit atan2 gegeben.

#include <chrono>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using moveit::planning_interface::MoveGroupInterface;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperClient = rclcpp_action::Client<FollowJointTrajectory>;

namespace
{
constexpr double APPROACH_HEIGHT = 0.10;   // 10 cm Annäherungshöhe
constexpr double EEF_STEP        = 0.005;  // 5 mm Interpolationsschritt
constexpr double JUMP_THRESHOLD  = 5.0;    // >0: Sprung im Gelenk-Raum ABLEHNEN
                                           // (war 0 → im descend wurde Handgelenk/Ellbogen
                                           //  teleportiert und machte unsinnige Bewegungen)
constexpr double MIN_FRACTION    = 0.95;   // unter 95% nicht akzeptiert
constexpr double VEL_SCALE       = 0.20;   // Sicherheit: 20% Geschwindigkeit
constexpr double ACC_SCALE       = 0.20;

constexpr double TARGET_WAIT_SEC = 10.0;   // Warten auf /vida/target
}

// Greifer schaut nach unten: tcp +Y -> Welt -Z. yaw = Azimut, tilt = Abweichung von der Senkrechten.
// tilt=0 -> direkt nach unten; tilt>0 -> Greifer neigt sich in yaw-Richtung. (exakt gleiche Konvention wie
// goto_clicked_point.cpp — NICHT literal (1,0,0,0).)
static geometry_msgs::msg::Quaternion gripperOrientation(double yaw, double tilt)
{
  tf2::Quaternion q;
  q.setRPY(-M_PI_2 + tilt, 0.0, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

static Pose lift(const Pose& p, double dz)
{
  Pose out = p;
  out.position.z += dz;
  return out;
}

static bool cartesianMove(
    MoveGroupInterface& arm, const Pose& target,
    const rclcpp::Logger& log, const std::string& label)
{
  std::vector<Pose> waypoints{ target };
  moveit_msgs::msg::RobotTrajectory traj;

  double fraction = arm.computeCartesianPath(
      waypoints, EEF_STEP, JUMP_THRESHOLD, traj);
  RCLCPP_INFO(log, "[%s] cartesian fraction = %.3f", label.c_str(), fraction);

  if (fraction < MIN_FRACTION) {
    RCLCPP_ERROR(log, "[%s] cartesian Pfad unvollstaendig (%.2f < %.2f) — abgebrochen",
                 label.c_str(), fraction, MIN_FRACTION);
    return false;
  }

  if (!arm.execute(traj)) {
    RCLCPP_ERROR(log, "[%s] execute() fehlgeschlagen", label.c_str());
    return false;
  }
  return true;
}

// Versucht die Pre-grasp (Annäherungs-)Pose mit einem yaw+tilt Raster; wendet den ersten planbaren+
// ausführbaren an und gibt die gewählte Ausrichtung über `chosen` zurück. So nutzen descend/retreat
// die GLEICHE Ausrichtung (die orientation des cartesian Abstiegs bleibt konstant).
//
// Warum: wenn die Schraube armnah/niedrig ist, ist die reine senkrecht-nach-unten (tilt=0) Einzel-Pose
// in der IK nicht kollisionsfrei/limit-innerhalb lösbar (OMPL "unable to sample valid goal states").
// Exakt gleicher stufenweiser Ansatz wie goto_clicked_point.cpp: zuerst direkt nach unten,
// sonst Greifer neigen und yaw drehen.
static bool approachWithFallback(
    MoveGroupInterface& arm, const Pose& pre_pos /*nur position wird genutzt*/,
    const rclcpp::Logger& log, const std::string& label,
    geometry_msgs::msg::Quaternion& chosen)
{
  const double yaw_out = std::atan2(pre_pos.position.y, pre_pos.position.x);
  const std::vector<double> yaws{
    yaw_out, yaw_out + M_PI_2, yaw_out - M_PI_2,
    yaw_out + M_PI, yaw_out + M_PI_4, yaw_out - M_PI_4,
  };
  // Für eine flach liegende Schraube NUR direkt-nach-unten (tilt=0) greifen. Schräge Annäherung
  // (alter 20°/35° Fallback) schob die Finger seitlich weg und verdarb den Griff;
  // außerdem fiel sie, als "Reichweiten-Kompromiss" gewählt, 1-2cm neben die Schraube.
  // Jetzt geht sie entweder mit tilt=0 gerade runter-hält, oder sagt sauber "unerreichbar".
  const std::vector<double> tilts{ 0.0 };

  Pose target = pre_pos;
  MoveGroupInterface::Plan plan;
  for (double tilt : tilts) {
    for (double yaw : yaws) {
      target.orientation = gripperOrientation(yaw, tilt);
      arm.setPoseTarget(target);
      if (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(log, "[%s] geloest: yaw=%+.0f° tilt=%.0f°",
                    label.c_str(), yaw * 180.0 / M_PI, tilt * 180.0 / M_PI);
        if (!arm.execute(plan)) {
          RCLCPP_ERROR(log, "[%s] execute() fehlgeschlagen", label.c_str());
          return false;
        }
        chosen = target.orientation;
        return true;
      }
      RCLCPP_INFO(log, "  [%s] yaw=%+.0f° tilt=%.0f° nicht planbar, naechste",
                  label.c_str(), yaw * 180.0 / M_PI, tilt * 180.0 / M_PI);
    }
  }
  RCLCPP_ERROR(log, "[%s] keine yaw/tilt Kombination planbar — Ziel unerreichbar",
               label.c_str());
  return false;
}

// Greifer DIREKT an den Controller (FollowJointTrajectory) senden — MoveIt
// collision-aware Plan NICHT verwenden. Grund: im abgesenkten Pose sind die Greiferfinger
// nahe an platform_top → der MoveGroup Greifer-Plan brach sofort mit "start state in collision"
// ab. Die Schließ-/Öffnungsbewegung braucht keine Planung; eine einzelne Gelenk-
// (gripper_controller) Position wird auf den joint_trajectory_controller geschrieben.
// SRDF: open=0.15, closed=-0.74 (URDF Gelenkbereich [-0.74, 0.15]).
static bool setGripper(
    const std::shared_ptr<GripperClient>& client,
    const std::string& named_state, const rclcpp::Logger& log)
{
  const double pos = (named_state == "open") ? 0.15 : -0.74;

  if (!client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(log, "kein Greifer-Action-Server "
                 "(/gripper_controller/follow_joint_trajectory)");
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
    RCLCPP_ERROR(log, "Greifer-Goal konnte nicht gesendet werden: %s", named_state.c_str());
    return false;
  }
  auto handle = goal_fut.get();
  if (!handle) {
    RCLCPP_ERROR(log, "Greifer-Goal abgelehnt: %s", named_state.c_str());
    return false;
  }
  auto res_fut = client->async_get_result(handle);
  if (res_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_WARN(log, "Greifer-Ergebnis Zeitueberschreitung (trotzdem weiter): %s",
                named_state.c_str());
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // mechanisches Settling
  RCLCPP_INFO(log, "Greifer -> %s (pos=%.2f) OK", named_state.c_str(), pos);
  return true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "pick_place_cartesian",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  // Optionale Parameter (ros2 run ... --ros-args -p grasp_z_offset:=0.005)
  // Da NodeOptions().automatically_declare_parameters_from_overrides(true) aktiv ist,
  // sind die per -p von der Kommandozeile übergebenen Parameter bereits deklariert; ein erneuter
  // declare_parameter Aufruf wirft ParameterAlreadyDeclaredException. Deshalb
  // nur deklarieren, wenn noch nicht deklariert, dann den Wert lesen.
  auto param_d = [&](const std::string& name, double def) {
    if (!node->has_parameter(name)) node->declare_parameter<double>(name, def);
    return node->get_parameter(name).as_double();
  };
  const double grasp_z_offset = param_d("grasp_z_offset", 0.0);
  const double place_x        = param_d("place_x", 0.15);
  const double place_y        = param_d("place_y", 0.15);
  const double place_z        = param_d("place_z", 0.05);

  // Für die async callbacks von MoveGroupInterface ist ein eigener Executor nötig.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() { executor.spin(); });

  // --- Schraubenposition vom Detector holen (/vida/target, robot_base Frame) ---
  // Die orientation des Detectors ist nur Z-yaw; da wir yaw-frei arbeiten, nutzen wir
  // NUR die Position, die Ausrichtung bauen wir selbst mit gripperDown().
  geometry_msgs::msg::Point screw;
  std::atomic<bool> have_target{false};
  auto target_sub = node->create_subscription<PoseStamped>(
      "/vida/target", rclcpp::QoS(1),
      [&](PoseStamped::SharedPtr m) {
        screw = m->pose.position;
        have_target = true;
      });

  RCLCPP_INFO(log, "warte auf /vida/target (Detector muss laufen)...");
  {
    auto t0 = node->now();
    while (rclcpp::ok() && !have_target &&
           (node->now() - t0).seconds() < TARGET_WAIT_SEC) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  if (!have_target) {
    RCLCPP_ERROR(log, "innerhalb von %.0f s kein /vida/target gekommen — laeuft der Detector? Abgebrochen.",
                 TARGET_WAIT_SEC);
    rclcpp::shutdown(); spinner.join(); return 1;
  }
  RCLCPP_INFO(log, "Schraubenziel: x=%.3f y=%.3f z=%.3f (z_offset=%.3f)",
              screw.x, screw.y, screw.z, grasp_z_offset);

  MoveGroupInterface arm(node, "arm");
  // Der Greifer wird jetzt statt über MoveIt direkt über die Controller-Action gesteuert.
  auto gripper_client = rclcpp_action::create_client<FollowJointTrajectory>(
      node, "/gripper_controller/follow_joint_trajectory");

  arm.setPoseReferenceFrame("robot_base");
  arm.setEndEffectorLink("tcp");
  arm.setPlanningTime(5.0);            // constraint-freier joint plan ist schnell; 5s reicht reichlich
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(VEL_SCALE);
  arm.setMaxAccelerationScalingFactor(ACC_SCALE);

  RCLCPP_INFO(log, "Planning frame: %s | EE link: %s",
              arm.getPlanningFrame().c_str(),
              arm.getEndEffectorLink().c_str());

  // --- Ziel-Posen (robot_base Frame, Meter) ---
  // Die Ausrichtung wird hier NICHT FIXIERT: im pre-grasp Plan wird ein yaw+tilt Raster probiert
  // (approachWithFallback) und die gewählte Ausrichtung auf descend/retreat übertragen.
  Pose pick;
  pick.position.x = screw.x;
  pick.position.y = screw.y;
  pick.position.z = screw.z + grasp_z_offset;

  Pose place;
  place.position.x = place_x;
  place.position.y = place_y;
  place.position.z = place_z;

  // --- Sequenz ---
  RCLCPP_INFO(log, ">> Zur Ready-Position fahren (constraint-frei)");
  arm.setNamedTarget("ready");
  if (!arm.move()) {
    RCLCPP_ERROR(log, "konnte nicht zu Ready fahren"); rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(log, ">> Gripper open");
  if (!setGripper(gripper_client,"open", log)) { rclcpp::shutdown(); spinner.join(); return 1; }

  // HINWEIS: auf die free-space joint plans wird KEIN orientation PATH CONSTRAINT gelegt.
  // Die senkrechte Ausrichtung ist in der Ziel-Pose (gripperDown) bereits vorhanden; die cartesian
  // Abstiegs-/Aufstiegs-Segmente halten die Ausrichtung konstant. (Siehe Erklärung am Dateianfang.)
  RCLCPP_INFO(log, ">> [PICK] joint plan → pick_pre (yaw+tilt fallback)");
  geometry_msgs::msg::Quaternion pick_orient;
  if (!approachWithFallback(arm, lift(pick, APPROACH_HEIGHT), log, "pick_pre", pick_orient)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }
  pick.orientation = pick_orient;   // descend/retreat nutzt die gleiche Ausrichtung

  RCLCPP_INFO(log, ">> [PICK] cartesian descend");
  if (!cartesianMove(arm, pick, log, "pick_descend")) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(log, ">> [PICK] gripper close");
  if (!setGripper(gripper_client,"closed", log)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(log, ">> [PICK] cartesian retreat");
  if (!cartesianMove(arm, lift(pick, APPROACH_HEIGHT), log, "pick_lift")) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(log, ">> [PLACE] joint plan → place_pre (yaw+tilt fallback)");
  geometry_msgs::msg::Quaternion place_orient;
  if (!approachWithFallback(arm, lift(place, APPROACH_HEIGHT), log, "place_pre", place_orient)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }
  place.orientation = place_orient;   // descend/retreat nutzt die gleiche Ausrichtung

  RCLCPP_INFO(log, ">> [PLACE] cartesian descend");
  if (!cartesianMove(arm, place, log, "place_descend")) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(log, ">> [PLACE] gripper open");
  if (!setGripper(gripper_client,"open", log)) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(log, ">> [PLACE] cartesian retreat");
  if (!cartesianMove(arm, lift(place, APPROACH_HEIGHT), log, "place_lift")) {
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  RCLCPP_INFO(log, ">> Zu Home zurueck");
  arm.setNamedTarget("home");
  arm.move();

  RCLCPP_INFO(log, "Pick & place abgeschlossen.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
