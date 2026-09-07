// RViz-Klick → Greifer senkrecht 5 cm über den Punkt hovern.
//
// Ablauf:
//   1) In RViz mit dem "Publish Point" Tool auf einen Punkt der Pointcloud klicken
//   2) Wenn /clicked_point kommt, Marker veröffentlichen + Sicherheits-/Reichweiten-Prüfung
//   3) Annäherungs-Pose: Punkt + 15 cm Z, Greifer schaut nach unten (tcp Y = world -Z)
//      Yaw frei — zuerst atan2(y,x), bei Fehlschlag werden andere Winkel probiert
//   4) Cartesian Abstieg: 15 cm → 5 cm Hover
//
// Wichtig:
//   - Fixed Frame robot_base (RViz Publish Point veröffentlicht in diesem Frame)
//   - TCP Frame +Y Achse ist "fingers" Richtung (gripper_base_to_tcp xyz="0 0.105 -0.01")
//   - position_only_ik = false (aus launch), TRAC-IK sucht 6DOF

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using moveit::planning_interface::MoveGroupInterface;

namespace
{
constexpr double HOVER_HEIGHT    = 0.05;   // 5 cm darüber — Ziel-Hover
constexpr double APPROACH_HEIGHT = 0.15;   // 15 cm darüber — Annäherung
constexpr double MAX_REACH_XY    = 0.25;   // myCobot 280: 28 cm Reichweite − 3 cm Sicherheit
// Innere Totzone: Schulter bei z=0.158 m; an Punkten sehr nah an der Basis muss der Arm den
// Greifer nicht senkrecht halten können und den Ellbogen über sich selbst falten → Plan abort.
// Der echte Arbeitsbereich ist ein Ring: MIN_REACH_XY .. MAX_REACH_XY.
constexpr double MIN_REACH_XY    = 0.13;   // 13 cm — innerhalb dieses Radius unerreichbar
constexpr double VEL_SCALE       = 0.20;
constexpr double ACC_SCALE       = 0.20;
constexpr double EEF_STEP        = 0.005;  // 5 mm
constexpr double JUMP_THRESHOLD  = 0.0;    // 0 = deaktiviert
constexpr double MIN_FRACTION    = 0.90;
}

class GotoClickedPoint : public rclcpp::Node
{
public:
  GotoClickedPoint()
  : Node("goto_clicked_point",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/clicked_target_marker", 10);

    boundary_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "/reach_boundary_marker", 10);

    click_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        std::bind(&GotoClickedPoint::onClick, this, std::placeholders::_1));

    // Reichweitengrenze periodisch veröffentlichen (nicht latched → alle 2 s aktualisieren).
    boundary_timer_ = create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&GotoClickedPoint::publishReachBoundary, this));

    RCLCPP_INFO(get_logger(),
                "Bereit. In RViz mit dem 'Publish Point' Tool auf die Pointcloud klicken. "
                "Klickbarer Bereich ist ein RING: ZWISCHEN dem roten inneren Kreis und dem "
                "orangefarbenen aeusseren Kreis. Das Innere (Totzone) und Aeussere (ausser Reichweite) funktioniert nicht.");
  }

  void setArm(std::shared_ptr<MoveGroupInterface> arm) { arm_ = std::move(arm); }

private:
  void onClick(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const auto& p = msg->point;
    const std::string& frame = msg->header.frame_id;

    if (frame != "robot_base") {
      RCLCPP_WARN(get_logger(),
                  "Geklickter Punkt im Frame '%s' — robot_base wurde erwartet. "
                  "Setze RViz Fixed Frame auf robot_base.",
                  frame.c_str());
      return;
    }

    const double dist_xy = std::hypot(p.x, p.y);
    const double dist_3d = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);

    RCLCPP_INFO(get_logger(), "================================================");
    RCLCPP_INFO(get_logger(), "Gewaehlter Punkt (robot_base Frame):");
    RCLCPP_INFO(get_logger(), "  x = %+.4f m  (%+.1f cm)", p.x, p.x * 100);
    RCLCPP_INFO(get_logger(), "  y = %+.4f m  (%+.1f cm)", p.y, p.y * 100);
    RCLCPP_INFO(get_logger(), "  z = %+.4f m  (%+.1f cm)", p.z, p.z * 100);
    RCLCPP_INFO(get_logger(), "Abstand zum Roboterzentrum:");
    RCLCPP_INFO(get_logger(), "  in XY-Ebene: %.4f m  (%.1f cm)", dist_xy, dist_xy * 100);
    RCLCPP_INFO(get_logger(), "  3D (gesamt) : %.4f m  (%.1f cm)", dist_3d, dist_3d * 100);
    RCLCPP_INFO(get_logger(), "================================================");

    publishMarker(p);

    if (dist_xy > MAX_REACH_XY) {
      RCLCPP_WARN(get_logger(),
                  "Punkt ausserhalb der Roboter-Reichweite (%.1f cm > %.1f cm). Bewegung abgebrochen.",
                  dist_xy * 100, MAX_REACH_XY * 100);
      return;
    }

    if (dist_xy < MIN_REACH_XY) {
      RCLCPP_WARN(get_logger(),
                  "Punkt zu nah am Roboter (%.1f cm < %.1f cm) — innere Totzone. "
                  "Der Greifer kann senkrecht nach unten nicht so nah heran. "
                  "Klicke auf den AEUSSEREN Teil des orangefarbenen Rings.",
                  dist_xy * 100, MIN_REACH_XY * 100);
      return;
    }

    if (!arm_) {
      RCLCPP_WARN(get_logger(), "Arm nicht bereit, nur Marker veroeffentlicht.");
      return;
    }

    // plan()/execute() blockierend + verstopft den Executor (Action-Client-Deadlock).
    // Den Subscription-Callback verlassen, Planung in eigenem Worker-Thread machen.
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(),
                  "Vorherige Bewegung laeuft noch — dieser Klick wurde ignoriert.");
      return;
    }

    geometry_msgs::msg::Point pt = p;
    std::thread([this, pt]() {
      moveToHover(pt);
      busy_.store(false);
    }).detach();
  }

  // Der echte Arbeitsbereich ist ein RING (in der z=0 Ebene):
  //   - äußerer orangefarbener Kreis = MAX_REACH_XY (außerhalb ist außer Reichweite)
  //   - innerer roter Kreis   = MIN_REACH_XY (innerhalb ist Totzone)
  // Der klickbare Bereich ist ZWISCHEN den beiden Kreisen.
  void publishReachBoundary()
  {
    publishCircle(MAX_REACH_XY, 0, 1.0f, 0.5f, 0.0f);  // außen: orange
    publishCircle(MIN_REACH_XY, 1, 1.0f, 0.0f, 0.0f);  // innen:  rot
  }

  void publishCircle(double radius, int id, float r, float g, float b)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "robot_base";
    m.header.stamp = now();
    m.ns = "reach_boundary";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.005;  // 5 mm Linie
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0);

    constexpr int N = 64;
    for (int i = 0; i <= N; ++i) {
      const double a = 2.0 * M_PI * static_cast<double>(i) / N;
      geometry_msgs::msg::Point pt;
      pt.x = radius * std::cos(a);
      pt.y = radius * std::sin(a);
      pt.z = 0.0;
      m.points.push_back(pt);
    }
    boundary_pub_->publish(m);
  }

  void publishMarker(const geometry_msgs::msg::Point& p)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "robot_base";
    m.header.stamp = now();
    m.ns = "clicked_target";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = p;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.02;
    m.color.r = 0.1f;
    m.color.g = 1.0f;
    m.color.b = 0.1f;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0);
    marker_pub_->publish(m);
  }

  // Greifer schaut nach unten (tcp Y -> -Z world). yaw = Azimut, tilt = Abweichung von der Senkrechten.
  // tilt=0 → direkt nach unten; tilt>0 → Greifer neigt sich in yaw-Richtung (an nahen
  // Punkten kann der Arm schräg von außen annähern statt den Ellbogen zu falten).
  static geometry_msgs::msg::Quaternion gripperOrientation(double yaw, double tilt)
  {
    tf2::Quaternion q;
    q.setRPY(-M_PI_2 + tilt, 0.0, yaw);
    q.normalize();
    return tf2::toMsg(q);
  }

  void moveToHover(const geometry_msgs::msg::Point& target)
  {
    geometry_msgs::msg::Pose approach;
    approach.position.x = target.x;
    approach.position.y = target.y;
    approach.position.z = target.z + APPROACH_HEIGHT;

    // Yaw frei: zuerst auf den Punkt gerichtet (atan2), dann ±90, ±180, ±45 probieren.
    const double yaw_out = std::atan2(target.y, target.x);
    const std::vector<double> yaws{
      yaw_out,
      yaw_out + M_PI_2,
      yaw_out - M_PI_2,
      yaw_out + M_PI,
      yaw_out + M_PI_4,
      yaw_out - M_PI_4,
    };

    // Zuerst direkt nach unten (tilt=0); wenn nicht lösbar, Greifer stufenweise neigen.
    // Die Neigung erleichtert das Ellbogen-Falten an armnahen/niedrigen Punkten.
    const std::vector<double> tilts{
      0.0,
      20.0 * M_PI / 180.0,
      35.0 * M_PI / 180.0,
    };

    MoveGroupInterface::Plan approach_plan;
    double chosen_yaw = 0.0;
    double chosen_tilt = 0.0;
    bool ok = false;

    for (double tilt : tilts) {
      for (double yaw : yaws) {
        approach.orientation = gripperOrientation(yaw, tilt);
        arm_->setPoseTarget(approach);
        if (arm_->plan(approach_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          chosen_yaw = yaw;
          chosen_tilt = tilt;
          ok = true;
          break;
        }
        RCLCPP_INFO(get_logger(),
                    "  yaw=%+.0f° tilt=%.0f° nicht planbar, naechste wird probiert",
                    yaw * 180.0 / M_PI, tilt * 180.0 / M_PI);
      }
      if (ok) break;
    }

    if (!ok) {
      RCLCPP_ERROR(get_logger(),
                   "Annaeherungs-Plan fehlgeschlagen — bei keinem yaw/tilt wurde eine Loesung gefunden. "
                   "Punkt ausser Reichweite oder Hindernis vorhanden.");
      return;
    }

    RCLCPP_INFO(get_logger(),
                "Annaeherung: 15 cm darueber, yaw=%+.0f° tilt=%.0f°, wird ausgefuehrt",
                chosen_yaw * 180.0 / M_PI, chosen_tilt * 180.0 / M_PI);

    if (!arm_->execute(approach_plan)) {
      RCLCPP_ERROR(get_logger(), "Annaeherungs-execute fehlgeschlagen.");
      return;
    }

    // Cartesian Abstieg 15cm → 5cm, Ausrichtung konstant (auch yaw konstant)
    geometry_msgs::msg::Pose descent = approach;
    descent.position.z = target.z + HOVER_HEIGHT;

    std::vector<geometry_msgs::msg::Pose> waypoints{descent};
    moveit_msgs::msg::RobotTrajectory traj;
    const double fraction = arm_->computeCartesianPath(
        waypoints, EEF_STEP, JUMP_THRESHOLD, traj);

    RCLCPP_INFO(get_logger(), "Cartesian Abstieg: %.0f%% Abdeckung", fraction * 100.0);
    if (fraction < MIN_FRACTION) {
      RCLCPP_ERROR(get_logger(),
                   "Cartesian Abstieg unzureichend (%.0f%% < %.0f%%). Abgebrochen.",
                   fraction * 100.0, MIN_FRACTION * 100.0);
      return;
    }

    if (!arm_->execute(traj)) {
      RCLCPP_ERROR(get_logger(), "Abstiegs-execute fehlgeschlagen.");
      return;
    }

    RCLCPP_INFO(get_logger(),
                "Roboter in Hover-Position (Punkt + 5 cm). Mit dem Lineal messen!");
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundary_pub_;
  rclcpp::TimerBase::SharedPtr boundary_timer_;
  std::shared_ptr<MoveGroupInterface> arm_;
  std::atomic<bool> busy_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GotoClickedPoint>();

  // SingleThreaded — der goal-register/response Race in Galactic rclcpp_action
  // wird nur im Multi-Thread ausgelöst. Damit der Subscription-Callback nicht lange
  // blockiert, wird plan/execute ohnehin in einem eigenen std::thread gemacht (in onClick).
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() { executor.spin(); });

  auto arm = std::make_shared<MoveGroupInterface>(node, "arm");
  arm->setPoseReferenceFrame("robot_base");
  arm->setEndEffectorLink("tcp");
  arm->setPlanningTime(3.0);
  arm->setNumPlanningAttempts(5);
  arm->setMaxVelocityScalingFactor(VEL_SCALE);
  arm->setMaxAccelerationScalingFactor(ACC_SCALE);

  node->setArm(arm);
  RCLCPP_INFO(node->get_logger(), "MoveGroup bereit. Wartet.");

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
