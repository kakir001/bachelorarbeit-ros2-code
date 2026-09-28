// myCobot 280 JN — Tilt-bewusster Wellen-PICK (Hover + Bestätigung + paralleler Abstieg + greifen + heben).
//
// Ablauf (Sitzung 25 Plan — ~/.claude/plans/shimmering-wondering-bachman.md):
//   home (0) → /welle/ziel warten (Detector gibt VOLLES Quaternion aus; ohne
//              zuverlässigen Tilt gibt er gar nichts aus → Roboter wartet hier,
//              Benutzer mischt die Wellen)
//   → HOVER: vom Greif-Pose entlang der ANNAEHERUNGSACHSE um approach_height zurück, Greifer in
//            Greif-Ausrichtung, Finger offen. IK-Fallback: Detector q, sonst q∘Ry(π)
//            (180° Finger-Flip — Annäherungsachse GLEICH, Griff symmetrisch). Wenn beides scheitert,
//            "unerreichbar" → 5s warten, mit dem AKTUELLSTEN Ziel erneut versuchen.
//   → /pick/confirm WARTEN (std_msgs/Empty; in cam_viewer 'g'). Ziel wird im Hover-Moment EINGEFROREN.
//   → PARALLELER ABSTIEG: cartesianMove von hover→grasp (Ausrichtung konstant = parallele Annäherung).
//   → Greifer SCHLIESSEN → senkrechtes LIFT → HALTEN + warten (Ctrl+C). Kein Place.
//
// Die /welle/ziel Orientation des Detectors ist Greif-Frame: tcp +Y = Annäherungsachse.
// Gerade Welle → Annäherung ~direkt-nach-unten (top-down); schiefe Welle → achsenparalleler Abstieg.

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using moveit::planning_interface::MoveGroupInterface;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Marker = visualization_msgs::msg::Marker;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GripperClient = rclcpp_action::Client<FollowJointTrajectory>;

namespace
{
constexpr double EEF_STEP        = 0.005;  // 5 mm Interpolationsschritt
constexpr double JUMP_THRESHOLD  = 5.0;    // >0: Sprung im Gelenk-Raum ABLEHNEN
constexpr double MIN_FRACTION    = 0.95;   // unter 95% nicht akzeptiert
constexpr double VEL_SCALE       = 0.20;   // Sicherheit: 20% Geschwindigkeit
constexpr double ACC_SCALE       = 0.20;
constexpr double TARGET_WAIT_SEC = 30.0;   // Warten auf das erste /welle/ziel

// EXAKT gleiche Reihenfolge wie detection.py CLASS_NAMES (class_id 0..4 = Farbe).
const std::vector<std::string> CLASS_NAMES{"gelb", "weiss", "schwarz", "gruen", "rot"};
}

// PLACE Ausrichtungs-Kandidaten (gleiche Logik wie goto_clicked_point): Greifer schaut nach unten,
// yaw = Azimut zum Punkt (+Varianten), tilt = Abweichung von der Senkrechten. Wenn die Boxen an der
// REICHWEITENGRENZE des Roboters sind, gibt das senkrechte Handgelenk (tilt=0) keine IK → stufenweise neigen
// (tilt 20..55°) + yaw drehen. tf2::setRPY(-pi/2+tilt, 0, yaw) = gripperOrientation. Der erste planbare wird gewählt.
static std::vector<geometry_msgs::msg::Quaternion> placeOrientations(double x, double y)
{
  const double yaw_out = std::atan2(y, x);
  const std::vector<double> yaws{
    yaw_out, yaw_out + M_PI_2, yaw_out - M_PI_2,
    yaw_out + M_PI, yaw_out + M_PI_4, yaw_out - M_PI_4};
  const std::vector<double> tilts{
    0.0, 20.0 * M_PI / 180.0, 35.0 * M_PI / 180.0,
    45.0 * M_PI / 180.0, 55.0 * M_PI / 180.0};
  std::vector<geometry_msgs::msg::Quaternion> cands;
  for (double tilt : tilts) {
    for (double yaw : yaws) {
      tf2::Quaternion q;
      q.setRPY(-M_PI_2 + tilt, 0.0, yaw);
      q.normalize();
      cands.push_back(tf2::toMsg(q));
    }
  }
  return cands;
}

// Richtung von tcp +Y (Annäherungsachse) in der base aus dem Greif-Quaternion holen.
static tf2::Vector3 approachAxis(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  return tf2::quatRotate(tq, tf2::Vector3(0.0, 1.0, 0.0));
}

// Greif-Ausrichtung um die tcp +Y (Annäherungs-)Achse um 180° drehen: Finger tauschen
// die Plätze, aber der Griff ist symmetrisch; die Annäherungsachse AENDERT SICH NICHT (IK-Alternative).
static geometry_msgs::msg::Quaternion flipAboutApproach(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  tf2::Quaternion roll; roll.setRotation(tf2::Vector3(0.0, 1.0, 0.0), M_PI);
  tf2::Quaternion out = tq * roll;     // Drehung im lokalen Frame (tcp)
  out.normalize();
  return tf2::toMsg(out);
}

static Pose lift(const Pose& p, double dz)
{
  Pose out = p;
  out.position.z += dz;
  return out;
}

using GateFn = std::function<bool(const std::string&)>;
using DispPub = rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr;

static bool cartesianMove(
    MoveGroupInterface& arm, const Pose& target,
    const rclcpp::Logger& log, const std::string& label,
    const GateFn& gate, const DispPub& disp_pub)
{
  std::vector<Pose> waypoints{ target };
  moveit_msgs::msg::RobotTrajectory traj;
  double fraction = arm.computeCartesianPath(waypoints, EEF_STEP, JUMP_THRESHOLD, traj);
  RCLCPP_INFO(log, "[%s] cartesian fraction = %.3f", label.c_str(), fraction);
  if (fraction < MIN_FRACTION) {
    RCLCPP_ERROR(log, "[%s] cartesian Pfad unvollstaendig (%.2f < %.2f) — abgebrochen",
                 label.c_str(), fraction, MIN_FRACTION);
    return false;
  }
  // RViz Vorschau: cartesian Trajektorie als DisplayTrajectory veröffentlichen (move_group
  // gibt cartesian nicht automatisch auf display_planned_path aus — hover/home plan() gibt aus).
  if (disp_pub) {
    moveit_msgs::msg::DisplayTrajectory disp;
    disp.model_id = arm.getRobotModel()->getName();
    disp.trajectory.push_back(traj);
    auto cs = arm.getCurrentState();
    if (cs) moveit::core::robotStateToRobotStateMsg(*cs, disp.trajectory_start);
    disp_pub->publish(disp);
  }
  if (!gate(label)) return false;        // wenn preview_confirm an ist, VOR execute auf Bestätigung warten
  if (!arm.execute(traj)) {
    RCLCPP_ERROR(log, "[%s] execute() fehlgeschlagen", label.c_str());
    return false;
  }
  return true;
}

// Greifer DIREKT an den Controller senden (MoveIt collision-aware Plan NICHT verwenden).
// SRDF: open=0.15, closed=-0.74 (URDF Gelenkbereich [-0.74, 0.15]).
static constexpr double kGripOpen   =  0.15;   // URDF upper  = ganz auf (100 %)
static constexpr double kGripClosed = -0.74;   // URDF lower  = ganz zu  (0 %)

static bool setGripper(
    const std::shared_ptr<GripperClient>& client,
    double pos, const std::string& label, const rclcpp::Logger& log)
{
  pos = std::min(std::max(pos, kGripClosed), kGripOpen);
  if (!client->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(log, "kein Greifer-Action-Server"); return false;
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
  RCLCPP_INFO(log, "Greifer -> %s (pos=%.2f, ~%.0f %% offen) OK", label.c_str(), pos,
              (pos - kGripClosed) / (kGripOpen - kGripClosed) * 100.0);
  return true;
}

// Bequemlichkeit fuer die beiden Endlagen.
static bool setGripper(
    const std::shared_ptr<GripperClient>& client,
    const std::string& named_state, const rclcpp::Logger& log)
{
  return setGripper(client, named_state == "open" ? kGripOpen : kGripClosed, named_state, log);
}

// Zum HOVER planen+gehen: in Greif-Ausrichtung, sonst 180° Finger-Flip. Bei Erfolg wird die gewählte
// Ausrichtung in `chosen` geschrieben (descend nutzt die GLEICHE Ausrichtung = paralleler Abstieg).
static bool moveToHover(
    MoveGroupInterface& arm, const Pose& hover,
    const geometry_msgs::msg::Quaternion& grasp_q,
    const rclcpp::Logger& log, geometry_msgs::msg::Quaternion& chosen,
    const GateFn& gate, const std::string& label = "hover")
{
  const std::vector<geometry_msgs::msg::Quaternion> cands{
    grasp_q, flipAboutApproach(grasp_q)
  };
  Pose target = hover;
  MoveGroupInterface::Plan plan;
  for (size_t i = 0; i < cands.size(); ++i) {
    target.orientation = cands[i];
    arm.setPoseTarget(target);
    // wenn plan() erfolgreich ist, veröffentlicht move_group die Trajektorie auf
    // /move_group/display_planned_path → RViz MotionPlanning zeigt sie → gate wartet vor execute auf Bestätigung.
    if (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(log, "[%s] geloest (%s Ausrichtung)", label.c_str(), i == 0 ? "Haupt" : "180°-flip");
      if (!gate(label)) return false;       // wenn preview_confirm an ist, auf Bestätigung warten
      if (!arm.execute(plan)) {
        RCLCPP_ERROR(log, "[%s] execute() fehlgeschlagen", label.c_str()); return false;
      }
      chosen = cands[i];
      return true;
    }
    RCLCPP_INFO(log, "  [%s] %s Ausrichtung nicht planbar, naechste",
                label.c_str(), i == 0 ? "Haupt" : "180°-flip");
  }
  RCLCPP_ERROR(log, "[%s] IK nicht loesbar — Ziel unerreichbar", label.c_str());
  return false;
}

// Zum HOVER planen+gehen, KANDIDATEN-AUSRICHTUNGSLISTE der Reihe nach probieren (yaw+tilt
// Suche für PLACE). Der erste planbare wird ausgeführt, die gewählte Ausrichtung in `chosen` geschrieben.
static bool moveToHoverCands(
    MoveGroupInterface& arm, const Pose& hover,
    const std::vector<geometry_msgs::msg::Quaternion>& cands,
    const rclcpp::Logger& log, geometry_msgs::msg::Quaternion& chosen,
    const GateFn& gate, const std::string& label)
{
  Pose target = hover;
  MoveGroupInterface::Plan plan;
  for (size_t i = 0; i < cands.size(); ++i) {
    target.orientation = cands[i];
    arm.setPoseTarget(target);
    if (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(log, "[%s] geloest (Kandidat %zu/%zu)", label.c_str(), i + 1, cands.size());
      if (!gate(label)) return false;
      if (!arm.execute(plan)) {
        RCLCPP_ERROR(log, "[%s] execute() fehlgeschlagen", label.c_str()); return false;
      }
      chosen = cands[i];
      return true;
    }
  }
  RCLCPP_ERROR(log, "[%s] keiner der %zu Kandidaten geloest — unerreichbar",
               label.c_str(), cands.size());
  return false;
}

static Marker makeTargetMarker(const geometry_msgs::msg::Point& p)
{
  Marker m;
  m.header.frame_id = "robot_base";
  m.ns = "pick_tilt_target";
  m.id = 0;
  m.type = Marker::SPHERE;
  m.action = Marker::ADD;
  m.pose.position = p;
  m.pose.orientation.w = 1.0;
  m.scale.x = m.scale.y = m.scale.z = 0.02;
  m.color.g = 1.0; m.color.a = 0.9;
  return m;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
      "pick_tilt",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto log = node->get_logger();

  auto param_d = [&](const std::string& name, double def) {
    if (!node->has_parameter(name)) node->declare_parameter<double>(name, def);
    return node->get_parameter(name).as_double();
  };
  const double approach_height = param_d("approach_height", 0.10);  // 5-10 cm
  // pregrasp_open: wie weit der Greifer VOR dem Abstieg aufgeht (Gelenkeinheiten,
  // 0.15 = ganz auf, -0.74 = zu). Liegen mehrere Wellen dicht beieinander, stossen
  // ganz geoeffnete Finger an die Nachbarn — dann nur so weit oeffnen, wie die Welle
  // breit ist, plus etwas Luft. Der passende Wert haengt vom Greifer ab und ist am
  // Objekt auszumessen; 0.15 laesst das bisherige Verhalten unveraendert.
  const double pregrasp_open   = param_d("pregrasp_open", 0.15);
  const double grasp_z_offset  = param_d("grasp_z_offset", 0.0);
  const double descend_vel     = param_d("descend_vel_scale", 0.10);  // langsamer Abstieg
  // rapid traverse: Freiraum-Übergänge OHNE Wellenkontakt (home, hover) mit dieser
  // höheren Geschwindigkeit. Pendant zu CNC G00. Abstieg/greifen/heben NICHT BETROFFEN (bleibt bei
  // descend_vel und VEL_SCALE). Vernünftige Obergrenze für Open-Loop-Brücke + Sicherheit; bei 0 oder
  // gleich VEL_SCALE altes Verhalten (alles 20%).
  const double rapid_vel       = param_d("rapid_vel_scale", 0.40);
  auto param_b = [&](const std::string& name, bool def) {
    if (!node->has_parameter(name)) node->declare_parameter<bool>(name, def);
    return node->get_parameter(name).as_bool();
  };
  // preview_confirm: jede Bewegung (home/hover/Abstieg/heben) wird zuerst in RViz vorab angezeigt,
  // und WIRD NICHT AUSGEFUEHRT, bis /pick/confirm kommt → der echte Roboter wird zuerst beobachtet.
  const bool preview_confirm = param_b("preview_confirm", false);

  // --- PLACE (Sortierung nach Farbe) Parameter ---
  // place_enabled: nach dem Greifen + Rückkehr zu home die Welle in die ihrer FARBE gehörende Box
  // aus der Luft fallen lassen. Box-Zentren aus dem box_<farbe> Parameter (box_map.yaml, teach_boxes.py).
  const bool   place_enabled      = param_b("place_enabled", true);
  // place_hover_z / place_drop_z: ABSOLUTES z über der Box (robot_base, m). Da die Boxen
  // an der Reichweitengrenze sind, wird nicht das gespeicherte z der Box, sondern eine feste Höhe über der Plattform
  // genutzt (z.B. 8 cm Ablage). Ausrichtung wird über yaw+tilt-Suche gefunden (nicht top-down).
  const double place_hover_z      = param_d("place_hover_z", 0.12);  // Annäherung über der Box (absolut)
  const double place_drop_z       = param_d("place_drop_z", 0.08);   // Greifer-Öffnungshöhe (absolut)
  // box_<farbe> Parameter lesen (robot_base x,y,z m). Wenn nicht vorhanden, ist diese Farbe nicht eingelernt → leer.
  std::vector<std::vector<double>> box_xyz(CLASS_NAMES.size());
  for (size_t i = 0; i < CLASS_NAMES.size(); ++i) {
    const std::string pname = "box_" + CLASS_NAMES[i];
    if (node->has_parameter(pname)) {
      auto v = node->get_parameter(pname).as_double_array();
      if (v.size() == 3) box_xyz[i] = v;
    }
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() { executor.spin(); });

  auto marker_pub = node->create_publisher<Marker>(
      "/approach/target_marker", rclcpp::QoS(1).transient_local());

  // --- /welle/ziel (VOLLE Pose: Position + Greif-Ausrichtung) ---
  PoseStamped target_msg;
  std::atomic<bool> have_target{false};
  auto target_sub = node->create_subscription<PoseStamped>(
      "/welle/ziel", rclcpp::QoS(1),
      [&](PoseStamped::SharedPtr m) { target_msg = *m; have_target = true; });

  // --- /welle/ziel_farbe (Farbklasse der gegriffenen Welle, synchron mit /welle/ziel) ---
  std::atomic<int> latched_color{-1};
  auto color_sub = node->create_subscription<std_msgs::msg::Int32>(
      "/welle/ziel_farbe", rclcpp::QoS(1),
      [&](std_msgs::msg::Int32::SharedPtr m) { latched_color = m->data; });

  // --- /pick/confirm (Bestätigung im Hover) ---
  std::atomic<bool> confirmed{false};
  auto confirm_sub = node->create_subscription<std_msgs::msg::Empty>(
      "/pick/confirm", rclcpp::QoS(1),
      [&](std_msgs::msg::Empty::SharedPtr) { confirmed = true; });

  // Cartesian Vorschau-Ausgabe (RViz MotionPlanning "Planned Path" liest von hier).
  auto disp_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/move_group/display_planned_path", rclcpp::QoS(1));

  // wenn preview_confirm an ist: geplante Bewegung in RViz zeigen + auf ENTER-Bestätigung warten.
  // wenn aus, sofort true zurückgeben (altes Verhalten). Ctrl+C → false (abbrechen).
  GateFn gate = [&](const std::string& label) -> bool {
    if (!preview_confirm) return rclcpp::ok();
    confirmed = false;
    RCLCPP_INFO(log, ">> VORSCHAU [%s]: Trajektorie in RViz pruefen → ENTER = BESTAETIGUNG "
                "(ausfuehren). Ctrl+C = abbrechen.", label.c_str());
    while (rclcpp::ok() && !confirmed)
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (rclcpp::ok()) RCLCPP_INFO(log, "✓ [%s] bestaetigt — wird ausgefuehrt.", label.c_str());
    return rclcpp::ok();
  };

  MoveGroupInterface arm(node, "arm");
  auto gripper_client = rclcpp_action::create_client<FollowJointTrajectory>(
      node, "/gripper_controller/follow_joint_trajectory");
  arm.setPoseReferenceFrame("robot_base");
  arm.setEndEffectorLink("tcp");
  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(10);
  arm.setMaxVelocityScalingFactor(rapid_vel);     // home = Freiraum rapid traverse
  arm.setMaxAccelerationScalingFactor(rapid_vel);
  RCLCPP_INFO(log, "Geschwindigkeitsprofil: rapid(Freiraum)=%.0f%% | Abstieg=%.0f%% | greifen/heben=%.0f%%",
              rapid_vel * 100.0, descend_vel * 100.0, VEL_SCALE * 100.0);

  // --- SCHRITT 1: home (0) — plan→(Vorschau+Bestätigung)→execute ---
  RCLCPP_INFO(log, ">> SCHRITT 1: home (Punkt 0)");
  arm.setNamedTarget("home");
  {
    MoveGroupInterface::Plan home_plan;
    if (arm.plan(home_plan) != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(log, "home nicht planbar"); rclcpp::shutdown(); spinner.join(); return 1;
    }
    if (!gate("home")) { rclcpp::shutdown(); spinner.join(); return 0; }
    if (!arm.execute(home_plan)) {
      RCLCPP_ERROR(log, "konnte nicht zu home fahren"); rclcpp::shutdown(); spinner.join(); return 1;
    }
  }

  // Bei home bleibt der Greifer ZU. Offene Finger stehen in der Nullstellung direkt
  // unter der Kamera, verdecken die Arbeitsflaeche und werden vom Detektor sogar selbst
  // als Welle erkannt (2026-09-09 gemessen: conf 0.87-0.91). Geoeffnet wird erst oben
  // ueber der Welle, im Hover.
  RCLCPP_INFO(log, ">> Greifer schliessen (bei home zu — freie Kamerasicht)");
  setGripper(gripper_client, "closed", log);

  // --- Auf erstes Ziel warten (UNBEGRENZT) ---
  // Wenn der Detector keinen zuverlässigen Tilt findet, kommt /welle/ziel nie → Roboter wartet bei home,
  // Benutzer mischt die Wellen. Bei gewünschtem Beenden Ctrl+C. (TARGET_WAIT_SEC ist nur
  // die Periode der "noch nicht gekommen" Warnung.)
  RCLCPP_INFO(log, "warte auf /welle/ziel — Detector muss ZUVERLAESSIGEN Tilt finden "
                   "(sonst gibt er nichts aus). Wenn kein Ziel, Wellen mischen. Ctrl+C = Beenden.");
  {
    auto t0 = node->now();
    while (rclcpp::ok() && !have_target) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if ((node->now() - t0).seconds() >= TARGET_WAIT_SEC) {
        RCLCPP_WARN(log, "immer noch kein /welle/ziel — ist Detector/Kamera an? "
                    "fuer zuverlaessigen Tilt Wellen deutlich schraeg legen/mischen.");
        t0 = node->now();
      }
    }
  }
  if (!rclcpp::ok()) { rclcpp::shutdown(); spinner.join(); return 0; }

  // --- SCHRITT 2: hover probieren BIS ERREICHT; bei Erfolg auf Bestätigung warten, dann absteigen+greifen ---
  bool picked = false;
  int color_at_pick = -1; // Farbklasse des Gegriffenen (bei Hover-Erfolg eingefroren → PLACE)
  int hover_fails = 0;    // bei jedem 5. aufeinanderfolgenden Fehlschlag zu home zurück, Pose auffrischen
  int descend_fails = 0;  // bei 3 Abstiegs-Fehlschlägen aufgeben (Endlosschleife vermeiden)
  while (rclcpp::ok() && !picked) {
    // hover + home-recovery = Freiraum → rapid traverse. (descend reduziert unten auf
    // descend_vel, vor lift zurück auf VEL_SCALE.)
    arm.setMaxVelocityScalingFactor(rapid_vel);
    arm.setMaxAccelerationScalingFactor(rapid_vel);
    PoseStamped t = target_msg;                 // AKTUELLSTES Ziel (Snapshot)
    Pose grasp;
    grasp.position = t.pose.position;
    grasp.position.z += grasp_z_offset;
    grasp.orientation = t.pose.orientation;     // Detector Greif-Frame-Ausrichtung
    marker_pub->publish(makeTargetMarker(grasp.position));

    const double r = std::hypot(grasp.position.x, grasp.position.y) * 1000.0;

    // Hover = vom Greif-Pose entlang der Annäherungsachse um approach_height ZURUECK.
    tf2::Vector3 ay = approachAxis(grasp.orientation);   // Einheit (Annäherungsrichtung)
    Pose hover = grasp;
    hover.position.x -= ay.x() * approach_height;
    hover.position.y -= ay.y() * approach_height;
    hover.position.z -= ay.z() * approach_height;
    RCLCPP_INFO(log, ">> SCHRITT 2: HOVER (%.0fcm zurueck von grasp, entlang der Annaeherungsachse) "
                "| grasp r=%.0fmm z=%.3f", approach_height * 100.0, r, grasp.position.z);

    geometry_msgs::msg::Quaternion chosen;
    if (!moveToHover(arm, hover, grasp.orientation, log, chosen, gate)) {
      ++hover_fails;
      RCLCPP_WARN(log, "⚠ Ziel UNERREICHBAR (r=%.0fmm, %d. in Folge). Welle NAEHER an den Roboter "
                  "legen (GRUENER Kreis in der Kamera). In 5s erneut mit AKTUELLSTEM Ziel...",
                  r, hover_fails);
      if (hover_fails % 5 == 0) {
        RCLCPP_WARN(log, "5 aufeinanderfolgende Hover-Fehlschlaege — Rueckkehr zu home (Pose auffrischen)");
        arm.setNamedTarget("home"); arm.move();
      }
      for (int k = 0; k < 50 && rclcpp::ok(); ++k)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    hover_fails = 0;
    color_at_pick = latched_color.load();   // Farbe dieses Ziels einfrieren (PLACE nutzt das)
    grasp.orientation = chosen;   // descend GLEICHE Ausrichtung = paralleler Abstieg

    // Erst JETZT oeffnen: der Greifer steht ueber der Welle, der Abstieg kommt als
    // naechstes. Nur so weit wie noetig (pregrasp_open) — in einer vollen Schale
    // schieben ganz geoeffnete Finger die Nachbarwellen beiseite.
    RCLCPP_INFO(log, ">> Greifer oeffnen (pre-grasp, im Hover)");
    setGripper(gripper_client, pregrasp_open, "pre-grasp", log);

    // --- SCHRITT 3: im HOVER auf Bestätigung warten — NUR wenn preview_confirm AUS ist.
    // wenn preview_confirm an ist, wurde hover bereits über gate bestätigt, und der Abstieg unten
    // über ein eigenes gate bestätigt → diese alte Einzel-Bestätigung wird übersprungen (keine Doppel-Bestätigung).
    if (!preview_confirm) {
      confirmed = false;
      RCLCPP_INFO(log, ">> SCHRITT 3: im HOVER auf Bestaetigung warten — im cam_viewer-Fenster 'g' "
                  "(oder: ros2 topic pub --once /pick/confirm std_msgs/msg/Empty '{}')");
      while (rclcpp::ok() && !confirmed) {
        marker_pub->publish(makeTargetMarker(grasp.position));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (!rclcpp::ok()) break;
      RCLCPP_INFO(log, "✓ Bestaetigung erhalten — Abstieg.");
    }

    // --- SCHRITT 4: paralleler Abstieg (Ausrichtung konstant) — LANGSAME Geschwindigkeit + (preview) Vorschau+Bestätigung ---
    arm.setMaxVelocityScalingFactor(descend_vel);
    arm.setMaxAccelerationScalingFactor(descend_vel);
    bool descend_ok = cartesianMove(arm, grasp, log, "descend", gate, disp_pub);
    arm.setMaxVelocityScalingFactor(VEL_SCALE);
    arm.setMaxAccelerationScalingFactor(ACC_SCALE);
    if (!descend_ok) {
      if (!rclcpp::ok()) break;
      ++descend_fails;
      RCLCPP_ERROR(log, "Abstieg fehlgeschlagen (%d/3) — zu home zurueck, erneut versuchen", descend_fails);
      arm.setNamedTarget("home"); arm.move();
      if (descend_fails >= 3) {
        RCLCPP_FATAL(log, "3 Abstiegs-Fehlschlaege — AUFGEGEBEN. Welle/Szene anpassen und "
                     "Skript neu starten.");
        break;
      }
      continue;
    }

    // --- SCHRITT 5: greifen ---
    RCLCPP_INFO(log, ">> SCHRITT 5: Greifer schliessen (greifen)");
    setGripper(gripper_client, "closed", log);

    // --- SCHRITT 5b: GREIF-BESTAETIGUNG (VOR dem Heben warten) ---
    // Der Roboter STEHT mit gehaltener Welle im Greif-Pose; der Benutzer prüft per Kamera, ob die Welle
    // wirklich gegriffen wurde, und lässt sie bei korrekt mit 'g' (/pick/confirm)
    // heben. Wenn nicht gegriffen, Ctrl+C (hebt nicht). Dies ist die zweite, separate Bestätigung.
    confirmed = false;
    RCLCPP_INFO(log, ">> SCHRITT 5b: warte auf GREIF-Bestaetigung — wurde die Welle gehalten? "
                "in cam_viewer 'g' = heben. Wenn nicht gegriffen, Ctrl+C.");
    while (rclcpp::ok() && !confirmed) {
      marker_pub->publish(makeTargetMarker(grasp.position));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok()) break;
    RCLCPP_INFO(log, "✓ Greif-Bestaetigung erhalten — heben.");

    // --- SCHRITT 6: senkrecht heben ---
    RCLCPP_INFO(log, ">> SCHRITT 6: senkrecht heben (%.0fcm)", approach_height * 100.0);
    if (!cartesianMove(arm, lift(grasp, approach_height), log, "lift", gate, disp_pub)) {
      RCLCPP_WARN(log, "Heben cartesian unvollstaendig/abgebrochen — trotzdem weiter halten");
    }

    // --- SCHRITT 7: Welle HALTEND DIREKT + SCHNELL zum Null-(home-)Punkt zurück (rabbit) ---
    // Freiraum, kein Wellenkontakt → KEINE Bestätigung/Vorschau, direkt mit rapid Geschwindigkeit.
    // (Bei langsamer Fahrt zittern die Servos durch stick-slip; schnell = glatt.)
    RCLCPP_INFO(log, ">> SCHRITT 7: Welle wird gehalten — schnell zum NULL-(home-)Punkt zurueck (%.0f%%)",
                rapid_vel * 100.0);
    arm.setMaxVelocityScalingFactor(rapid_vel);
    arm.setMaxAccelerationScalingFactor(rapid_vel);
    arm.setNamedTarget("home");
    {
      MoveGroupInterface::Plan home_plan;
      if (arm.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        arm.execute(home_plan);   // direkt — kein gate
      } else {
        RCLCPP_WARN(log, "Rueckkehr zu Null nicht planbar — Welle wird im Greif-Pose gehalten");
      }
    }
    RCLCPP_INFO(log, "✓ Welle gegriffen, am NULL-Punkt — jetzt PLACE (Box nach Farbe).");

    // --- SCHRITT 8: PLACE — Welle über die ihrer FARBE gehörende Box bringen, aus der Luft fallen lassen ---
    // Farbe = das beim Pick eingefrorene color_at_pick. Box-Zentrum = box_<farbe> (teach).
    // "Aus der Luft fallen lassen": kein senkrechtes Drehen/in-Loch-Setzen — über die Box absteigen, Greifer öffnen.
    const std::string color_name =
        (color_at_pick >= 0 && color_at_pick < (int)CLASS_NAMES.size())
            ? CLASS_NAMES[color_at_pick] : std::string("?");
    const bool have_box = (color_at_pick >= 0 &&
                           color_at_pick < (int)box_xyz.size() &&
                           !box_xyz[color_at_pick].empty());

    if (!place_enabled) {
      RCLCPP_INFO(log, "place_enabled=false — Welle wird bei NULL gehalten (Ctrl+C).");
    } else if (!have_box) {
      RCLCPP_WARN(log, "⚠ PLACE uebersprungen: keine Box fuer Farbe='%s' (id=%d). "
                  "Wurde box_map.yaml eingelernt (teach_boxes.py)? Welle wird bei NULL gehalten.",
                  color_name.c_str(), color_at_pick);
    } else {
      const auto& b = box_xyz[color_at_pick];
      RCLCPP_INFO(log, ">> SCHRITT 8: PLACE — Box der Farbe='%s' (x=%.3f y=%.3f z=%.3f)",
                  color_name.c_str(), b[0], b[1], b[2]);

      // HOVER über der Box (Freiraum, Welle in der Luft → rapid). ABSOLUTES z, yaw+tilt-Suche
      // (Boxen an der Reichweitengrenze → top-down nicht lösbar, schräge Ausrichtung nötig).
      arm.setMaxVelocityScalingFactor(rapid_vel);
      arm.setMaxAccelerationScalingFactor(rapid_vel);
      Pose place_hover;
      place_hover.position.x = b[0];
      place_hover.position.y = b[1];
      place_hover.position.z = place_hover_z;        // ABSOLUT (ab Plattform)
      geometry_msgs::msg::Point bp; bp.x = b[0]; bp.y = b[1]; bp.z = place_drop_z;
      marker_pub->publish(makeTargetMarker(bp));

      const auto place_cands = placeOrientations(b[0], b[1]);
      geometry_msgs::msg::Quaternion place_q;
      if (!moveToHoverCands(arm, place_hover, place_cands, log, place_q, gate, "place-hover")) {
        RCLCPP_WARN(log, "⚠ keine IK ueber der Box (kein yaw/tilt) — Welle wird gehalten, zu NULL zurueck.");
        arm.setNamedTarget("home"); arm.move();
      } else {
        // Kurzer Abstieg in die Box hinein (noch in der Luft, kein Kontakt) — kontrollierte Geschwindigkeit.
        arm.setMaxVelocityScalingFactor(VEL_SCALE);
        arm.setMaxAccelerationScalingFactor(VEL_SCALE);
        Pose drop = place_hover;
        drop.position.z = place_drop_z;              // ABSOLUTE Ablagehöhe (z.B. 8cm)
        drop.orientation = place_q;
        if (!cartesianMove(arm, drop, log, "place-descend", gate, disp_pub)) {
          RCLCPP_WARN(log, "place Abstieg unvollstaendig/abgebrochen — wird aus aktueller Hoehe abgelegt");
          drop = place_hover;   // aus der Hover-Höhe ablegen
        }

        // ABLEGEN: Greifer öffnen → Welle fällt in die Box.
        RCLCPP_INFO(log, ">> SCHRITT 8b: Greifer oeffnen — Welle wird in Box '%s' abgelegt",
                    color_name.c_str());
        setGripper(gripper_client, "open", log);

        // Senkrecht zurückziehen (rapid) — dann zu NULL zurück.
        arm.setMaxVelocityScalingFactor(rapid_vel);
        arm.setMaxAccelerationScalingFactor(rapid_vel);
        cartesianMove(arm, lift(drop, place_hover_z - place_drop_z),
                      log, "place-retract", gate, disp_pub);
      }
    }

    // --- Zu NULL zurück (Finger offen oder leer — freie Rückkehr) ---
    arm.setMaxVelocityScalingFactor(rapid_vel);
    arm.setMaxAccelerationScalingFactor(rapid_vel);
    arm.setNamedTarget("home");
    {
      MoveGroupInterface::Plan home_plan;
      if (arm.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        arm.execute(home_plan);
    }
    RCLCPP_INFO(log, "✅ FERTIG — Welle in Box '%s' abgelegt, Roboter bei NULL (Ctrl+C).",
                color_name.c_str());
    picked = true;
  }

  // Halten + warten (bis Ctrl+C).
  rclcpp::Rate rate(2.0);
  while (rclcpp::ok()) {
    rate.sleep();
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
