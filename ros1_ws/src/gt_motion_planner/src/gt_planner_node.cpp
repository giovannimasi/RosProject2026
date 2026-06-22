// ============================================================================
//  Game Theoretic Motion Planner
//  Gioco congiunto (Nash/Pareto) su 1+ robot controllati. I pedoni sono rilevati
//  dal lidar (cluster->tracking->predizione) e inseriti nel gioco come ostacoli
//  dinamici. Pianificazione e controllo girano su thread separati.
// ============================================================================

#include <ros/ros.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/util/Console.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace ob = ompl::base;
namespace oc = ompl::control;

namespace
{
constexpr double kInf = 1e12;
}

// ----------------------------------------------------------------------------
//  Tipi base
// ----------------------------------------------------------------------------
struct State
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Goal
{
  double x{0.0};
  double y{0.0};
};

// Ostacolo statico (X^obj). box: semi-lati hx,hy; cylinder: raggio r.
struct Obstacle
{
  bool is_box{true};
  double x{0.0};
  double y{0.0};
  double hx{0.0};
  double hy{0.0};
  double r{0.0};
};

// Ostacolo dinamico rilevato dal lidar: un pedone.
// Stimati posizione e velocita' nel frame mondo dal tracking dei cluster lidar.
struct DynObs
{
  State pos;        // posizione corrente (centroide del cluster)
  double vx{0.0};
  double vy{0.0};
};

// Traccia interna del tracker lidar (associazione cluster<->traccia nel tempo).
struct Track
{
  int id{0};
  double x{0.0}, y{0.0};     // ultimo centroide (mondo)
  double vx{0.0}, vy{0.0};   // velocita' stimata su finestra temporale
  int hits{0};               // associazioni consecutive
  int misses{0};             // frame consecutivi senza associazione
  bool matched{false};       // associata in questo frame
  int dyn_latch{0};          // >0 => dinamica (ponte sui cali di velocita')
  bool ever_dynamic{false};  // confermata dinamica una volta -> pedone a vita
  double ax{0.0}, ay{0.0};   // ancora per la velocita' su finestra temporale
  ros::Time at;
  bool has_anchor{false};
};

// Un'azione = una traiettoria con la sua serie di controlli.
struct Action
{
  std::vector<State> trajectory;  // stati ricampionati a passo integrator_dt
  double v_cmd{0.0};              // primo controllo lineare
  double w_cmd{0.0};              // primo controllo angolare
  double cost{0.0};               // Jhat = Length
  bool group_blocked{false};      // true se attraversa il corridoio di un gruppo
  bool obstacle_blocked{false};   // true se attraversa un ostacolo statico
  std::string name;
};

struct AgentActions
{
  std::string name;
  State state;
  bool is_robot{false};
  std::vector<Action> actions;
};

// Un robot CONTROLLATO dal planner (agente controllabile).
// Ogni robot ha: goal propri, publisher cmd, scan lidar, e stato di
// pianificazione per-robot (ego/escape, isteresi, memorized).
struct ControlledRobot
{
  std::string name;            // nome del model in /gazebo/model_states
  std::string cmd_topic;
  std::string scan_topic;
  std::vector<Goal> goals;
  size_t goal_idx{0};
  bool loop_goals{true};
  bool final_reached{false};
  double goal_x{0.0};
  double goal_y{0.0};

  State state;
  bool have_state{false};

  ros::Publisher cmd_pub;
  ros::Subscriber scan_sub;
  sensor_msgs::LaserScan scan;
  bool have_scan{false};

  // contesto di pianificazione per-robot
  bool escape{false};
  std::vector<State> prev_traj;
  bool have_prev{false};
  bool braked{false};          // freno di sicurezza attivo questo ciclo

  // ---- piano condiviso col controller ad alta frequenza (mutex) ----
  std::vector<State> follow_path;  // ultima traiettoria scelta da seguire
  bool follow_stop{false};         // true => fermo (collisione inevitabile/brake)
  bool have_plan{false};
  double last_w_pub{0.0};          // ultimo w pubblicato (per lo smoothing)
  int stuck_count{0};              // cicli consecutivi con azione scelta = INF
};

// Memoria di un equilibrio: per ciascun attore
// la traiettoria che quell'equilibrio assumeva.
struct EqRecord
{
  size_t row{0};
  double robot_cost{0.0};
  std::vector<State> robot_traj;
  std::map<std::string, std::vector<State>> actor_traj;
};

// ----------------------------------------------------------------------------
//  Control sampler discreto
// ----------------------------------------------------------------------------
class Eq8ControlSampler : public oc::ControlSampler
{
public:
  Eq8ControlSampler(const oc::ControlSpace *space, double v, double w, double c)
    : oc::ControlSampler(space), v_(v), w_(w), c_(c)
  {
    options_ = {0.0, w_, -w_, c_ * w_, -c_ * w_};
  }

  void sample(oc::Control *control) override
  {
    double *vals = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    vals[0] = v_;
    vals[1] = options_[rng_.uniformInt(0, static_cast<int>(options_.size()) - 1)];
  }

private:
  double v_;
  double w_;
  double c_;
  std::vector<double> options_;
  ompl::RNG rng_;
};

// ============================================================================
//  Nodo
// ============================================================================
class GameTheoryPlannerNode
{
public:
  GameTheoryPlannerNode()
  {
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("turtlebot_name", turtlebot_name_, "turtlebot3_waffle_pi");

    if (!pnh.getParam("actor_names", actor_names_))
      actor_names_ = {"actor", "actor_2", "actor_3_group_L", "actor_4_group_R"};

    LoadGroupPairs(pnh);
    pnh.param<double>("group_corridor_half_width", group_corridor_half_width_, 0.70);

    pnh.param<bool>("loop_goals", loop_goals_, true);
    pnh.param<double>("goal_tolerance", goal_tolerance_, 0.30);
    pnh.param<double>("dyn_collision_dist", dyn_collision_dist_, 0.45);
    pnh.param<double>("ped_comfort_dist", ped_comfort_dist_, 0.75);
    pnh.param<double>("ped_comfort_weight", ped_comfort_weight_, 4.0);
    pnh.param<double>("robot_collision_dist", robot_collision_dist_, 0.55);
    pnh.param<int>("dyn_stuck_limit", dyn_stuck_limit_, 6);
    pnh.param<double>("w_ema", w_ema_, 0.6);
    pnh.param<double>("goal_region_x", goal_region_x_, 0.30);
    pnh.param<double>("goal_region_y", goal_region_y_, 0.50);
    LoadObstacles(pnh);
    pnh.param<bool>("obstacle_autodetect", obstacle_autodetect_, true);
    pnh.param<double>("obstacle_default_radius", obstacle_default_radius_, 0.40);

    // layer lidar (in aggiunta agli oggetti): evita i punti /scan reali
    pnh.param<bool>("use_lidar", use_lidar_, true);
    pnh.param<double>("lidar_grid_res", lidar_grid_res_, 0.10);
    pnh.param<double>("static_margin", static_margin_, 0.12);
    pnh.param<double>("lidar_inflation", lidar_inflation_, agent_radius_ + static_margin_);
    pnh.param<double>("lidar_actor_filter_radius", lidar_actor_filter_radius_, 0.60);
    pnh.param<double>("lidar_robot_filter_radius", lidar_robot_filter_radius_, 0.45);

    // ---- Tracker ostacoli DINAMICI dal lidar (pedoni, NON noti per nome) ----
    pnh.param<bool>("track_dynamic", track_dynamic_, true);
    pnh.param<double>("cluster_cell", cluster_cell_, 0.18);       // lato cella cluster
    pnh.param<int>("cluster_min_cells", cluster_min_cells_, 1);   // celle min per cluster
    pnh.param<double>("track_gate", track_gate_, 0.55);           // dist max associazione
    pnh.param<int>("track_max_misses", track_max_misses_, 6);
    pnh.param<int>("track_min_hits", track_min_hits_, 3);         // hit prima di usarla
    pnh.param<double>("dyn_speed_thresh", dyn_speed_thresh_, 0.07); // m/s -> dinamico
    pnh.param<double>("vel_ema", vel_ema_, 0.6);                  // (non usato: vel su finestra)
    pnh.param<double>("pos_ema", pos_ema_, 0.5);                  // (non usato)
    pnh.param<double>("vel_window", vel_window_, 0.5);           // finestra stima velocita' [s]
    pnh.param<double>("ped_max_speed", ped_max_speed_, 0.6);      // clamp vel. predetta
    pnh.param<double>("ped_predict_horizon", ped_predict_horizon_, 7.0); // orizzonte predizione [s]
    pnh.param<int>("dyn_latch_frames", dyn_latch_frames_, 8);    // tieni "dinamico" N frame
    pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh.param<double>("ego_bubble", ego_bubble_, 0.35);

    // corridoio strada (mantiene i bot in carreggiata)
    pnh.param<bool>("road_keep", road_keep_, false);
    pnh.param<double>("road_min_x", road_min_x_, -1.4);
    pnh.param<double>("road_max_x", road_max_x_, 1.4);
    pnh.param<double>("road_penalty", road_penalty_, 6.0);

    pnh.param<double>("goal_block_penalty", goal_block_penalty_, 100.0);
    pnh.param<double>("hysteresis_weight", hysteresis_weight_, 2.5);

    // ---- Freno di sicurezza (paper: emergency stop) ----
    pnh.param<bool>("safety_brake", safety_brake_, true);
    pnh.param<double>("brake_lookahead", brake_lookahead_, 0.8);
    pnh.param<double>("brake_margin", brake_margin_, 0.12);
    pnh.param<double>("brake_cone", brake_cone_, 1.05);  // ~60 gradi

    pnh.param<double>("interaction_radius", interaction_radius_, 5.0);
    pnh.param<double>("agent_radius", agent_radius_, 0.375);
    pnh.param<double>("collision_margin", collision_margin_, 0.10);

    pnh.param<double>("replan_dt", replan_dt_, 0.10);
    pnh.param<double>("integrator_dt", integrator_dt_, 0.05);
    pnh.param<double>("planning_horizon", planning_horizon_, 2.0);
    pnh.param<double>("control_rate", control_rate_, 10.0);

    pnh.param<double>("robot_v", robot_v_, 0.22);
    pnh.param<double>("w_min", w_min_, 0.10);
    pnh.param<double>("w_max", w_max_, 0.55);
    pnh.param<double>("curvature_factor", curvature_factor_, 0.5);
    pnh.param<double>("prop_min_lo", prop_min_lo_, 0.35);
    pnh.param<double>("prop_min_hi", prop_min_hi_, 0.65);
    pnh.param<double>("prop_max_lo", prop_max_lo_, 0.75);
    pnh.param<double>("prop_max_hi", prop_max_hi_, 1.25);
    pnh.param<int>("robot_num_actions", robot_num_actions_, 16);
    pnh.param<double>("rrt_solve_time", rrt_solve_time_, 0.02);

    pnh.param<double>("ws_min_x", ws_min_x_, -3.5);
    pnh.param<double>("ws_max_x", ws_max_x_, 3.5);
    pnh.param<double>("ws_min_y", ws_min_y_, -3.5);
    pnh.param<double>("ws_max_y", ws_max_y_, 3.5);

    pnh.param<int>("actor_num_actions", actor_num_actions_, 5);
    pnh.param<double>("actor_turn_rate", actor_turn_rate_, 0.25);
    pnh.param<double>("min_actor_speed", min_actor_speed_, 0.05);
    pnh.param<double>("max_actor_speed", max_actor_speed_, 1.2);
    pnh.param<double>("default_actor_speed", default_actor_speed_, 0.55);

    // controller ad alta frequenza (segue la traiettoria pianificata)
    pnh.param<double>("control_pub_rate", control_pub_rate_, 15.0);
    pnh.param<double>("lookahead", lookahead_, 0.45);

    pnh.param<bool>("enable_debug", enable_debug_, true);
    pnh.param<bool>("publish_markers", publish_markers_, true);
    pnh.param<std::string>("marker_frame", marker_frame_, "odom");

    num_traj_steps_ = std::max(1, static_cast<int>(std::round(planning_horizon_ / integrator_dt_)));

    SetupOmpl();

    SetupRobots(pnh);

    model_sub_ = nh_.subscribe("/gazebo/model_states", 1,
                               &GameTheoryPlannerNode::ModelStatesCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("gt_planner/markers", 1);

    // Timer ONESHOT auto-riarmato
    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                             &GameTheoryPlannerNode::TimerCallback, this,
                             /*oneshot=*/true);

    // Controller DISACCOPPIATO ad alta frequenza: insegue la traiettoria
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_pub_rate_),
                                     &GameTheoryPlannerNode::ControlPublish, this);

    ROS_INFO("[gt_planner] Avviato. robot controllati=%zu  M_n=%d  Delta_t=%.2f  R=%.3f",
             robots_.size(), robot_num_actions_, replan_dt_, agent_radius_);
    for (const auto &r : robots_)
      ROS_INFO("[gt_planner]   %s -> cmd=%s scan=%s goal0=[%.2f %.2f]",
               r.name.c_str(), r.cmd_topic.c_str(), r.scan_topic.c_str(), r.goal_x, r.goal_y);
  }

private:
  // -------------------- utilita' geometriche --------------------
  static double WrapToPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  static double Dist2D(const State &a, const State &b) { return std::hypot(a.x - b.x, a.y - b.y); }

  static double PathLength(const std::vector<State> &t)
  {
    double L = 0.0;
    for (size_t i = 1; i < t.size(); ++i)
      L += std::hypot(t[i].x - t[i - 1].x, t[i].y - t[i - 1].y);
    return L;
  }

  static State StepUnicycle(const State &s, double v, double w, double dt)
  {
    State o;
    o.x = s.x + dt * v * std::cos(s.yaw);
    o.y = s.y + dt * v * std::sin(s.yaw);
    o.yaw = WrapToPi(s.yaw + dt * w);
    return o;
  }

  static double YawFromQuat(const geometry_msgs::Quaternion &q)
  {
    tf::Quaternion tq(q.x, q.y, q.z, q.w);
    double r, p, y;
    tf::Matrix3x3(tq).getRPY(r, p, y);
    return y;
  }

  // direzione (versore) primo->ultimo punto di una traiettoria
  static void TrajDir(const std::vector<State> &t, double &dx, double &dy)
  {
    dx = 0.0;
    dy = 0.0;
    if (t.size() < 2)
      return;
    double ex = t.back().x - t.front().x;
    double ey = t.back().y - t.front().y;
    double n = std::hypot(ex, ey);
    if (n > 1e-6)
    {
      dx = ex / n;
      dy = ey / n;
    }
  }

  // -------------------- parametri da file --------------------
  void LoadGroupPairs(ros::NodeHandle &pnh)
  {
    XmlRpc::XmlRpcValue list;
    if (!pnh.getParam("group_pairs", list) || list.getType() != XmlRpc::XmlRpcValue::TypeArray)
      return;

    for (int i = 0; i < list.size(); ++i)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || list[i].size() < 2)
        continue;
      group_pairs_.push_back({static_cast<std::string>(list[i][0]),
                              static_cast<std::string>(list[i][1])});
    }
    ROS_INFO("[gt_planner] Coppie di gruppo caricate: %zu", group_pairs_.size());
  }

  static double XmlNum(XmlRpc::XmlRpcValue &v)
  {
    if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
      return static_cast<int>(v);
    return static_cast<double>(v);
  }

  static std::vector<Goal> ParseGoals(XmlRpc::XmlRpcValue &list)
  {
    std::vector<Goal> out;
    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
      return out;
    for (int i = 0; i < list.size(); ++i)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || list[i].size() < 2)
        continue;
      out.push_back({XmlNum(list[i][0]), XmlNum(list[i][1])});
    }
    return out;
  }

  // Costruisce la lista dei robot CONTROLLATI. Param "robots" (lista di struct
  // {name, cmd_topic, scan_topic, goals, loop_goals}). 
  void SetupRobots(ros::NodeHandle &pnh)
  {
    XmlRpc::XmlRpcValue list;
    bool have_multi = pnh.getParam("robots", list) &&
                      list.getType() == XmlRpc::XmlRpcValue::TypeArray && list.size() > 0;

    if (have_multi)
    {
      for (int i = 0; i < list.size(); ++i)
      {
        if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
          continue;
        ControlledRobot r;
        r.name = list[i].hasMember("name") ? static_cast<std::string>(list[i]["name"]) : "robot";
        r.cmd_topic = list[i].hasMember("cmd_topic") ? static_cast<std::string>(list[i]["cmd_topic"])
                                                      : ("/" + r.name + "/cmd_vel");
        r.scan_topic = list[i].hasMember("scan_topic") ? static_cast<std::string>(list[i]["scan_topic"])
                                                       : ("/" + r.name + "/scan");
        r.loop_goals = list[i].hasMember("loop_goals") ? static_cast<bool>(list[i]["loop_goals"]) : true;
        if (list[i].hasMember("goals"))
          r.goals = ParseGoals(list[i]["goals"]);
        if (r.goals.empty())
          r.goals.push_back({2.0, 0.0});
        robots_.push_back(r);
      }
    }
    else
    {
      // legacy: un solo robot
      ControlledRobot r;
      r.name = turtlebot_name_;
      r.cmd_topic = "/cmd_vel";
      r.scan_topic = scan_topic_;
      r.loop_goals = loop_goals_;
      XmlRpc::XmlRpcValue glist;
      if (pnh.getParam("goals", glist))
        r.goals = ParseGoals(glist);
      if (r.goals.empty())
        r.goals.push_back({2.0, 0.0});
      robots_.push_back(r);
    }

    // pubblisher cmd + subscriber scan per ogni robot
    for (size_t i = 0; i < robots_.size(); ++i)
    {
      auto &r = robots_[i];
      r.goal_x = r.goals[0].x;
      r.goal_y = r.goals[0].y;
      r.cmd_pub = nh_.advertise<geometry_msgs::Twist>(r.cmd_topic, 1);
      if (use_lidar_)
        r.scan_sub = nh_.subscribe<sensor_msgs::LaserScan>(
          r.scan_topic, 1, boost::bind(&GameTheoryPlannerNode::ScanCb, this, _1, i));
    }
  }

  void ScanCb(const sensor_msgs::LaserScan::ConstPtr &msg, size_t idx)
  {
    if (idx < robots_.size())
    {
      robots_[idx].scan = *msg;
      robots_[idx].have_scan = true;
    }
  }

  bool IsControlledRobotName(const std::string &nm) const
  {
    for (const auto &r : robots_)
      if (r.name == nm)
        return true;
    return false;
  }

  void LoadObstacles(ros::NodeHandle &pnh)
  {
    XmlRpc::XmlRpcValue list;
    if (!pnh.getParam("static_obstacles", list) ||
        list.getType() != XmlRpc::XmlRpcValue::TypeArray)
      return;

    auto num = [](XmlRpc::XmlRpcValue &v) -> double {
      if (v.getType() == XmlRpc::XmlRpcValue::TypeInt)
        return static_cast<int>(v);
      return static_cast<double>(v);
    };

    for (int i = 0; i < list.size(); ++i)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
        continue;
      Obstacle o;
      std::string type = list[i].hasMember("type") ? static_cast<std::string>(list[i]["type"]) : "box";
      o.is_box = (type != "cylinder");
      if (list[i].hasMember("x")) o.x = num(list[i]["x"]);
      if (list[i].hasMember("y")) o.y = num(list[i]["y"]);
      if (o.is_box)
      {
        if (list[i].hasMember("half_x")) o.hx = num(list[i]["half_x"]);
        if (list[i].hasMember("half_y")) o.hy = num(list[i]["half_y"]);
      }
      else if (list[i].hasMember("radius"))
        o.r = num(list[i]["radius"]);
      base_obstacles_.push_back(o);
    }
    obstacles_ = base_obstacles_;
    ROS_INFO("[gt_planner] Ostacoli statici da file: %zu", base_obstacles_.size());
  }

  // Ricostruisce la lista live degli ostacoli
  void RefreshObstacles()
  {
    obstacles_ = base_obstacles_;
    if (!obstacle_autodetect_)
      return;

    for (size_t i = 0; i < work_msg_.name.size(); ++i)
    {
      const std::string &nm = work_msg_.name[i];
      if (nm == "ground_plane" || nm == "sun" || IsControlledRobotName(nm))
        continue;
      if (std::find(actor_names_.begin(), actor_names_.end(), nm) != actor_names_.end())
        continue;
      if (nm.rfind("obs_", 0) == 0)  // gia' descritto con forma esatta nel file
        continue;

      Obstacle o;
      o.is_box = false;
      o.x = work_msg_.pose[i].position.x;
      o.y = work_msg_.pose[i].position.y;
      o.r = obstacle_default_radius_;
      obstacles_.push_back(o);
    }
  }

  // uno stato e' bloccato se dentro un oggetto modellato OPPURE vicino a un punto lidar 
  bool StateBlockedRaw(double x, double y) const
  {
    return InsideObstacle(x, y) || IsLidarOccupied(x, y);
  }

  // distanza dal punto (x,y) all'ostacolo piu' vicino (lidar + oggetti).
  // Usata in escape-mode per allontanarsi.
  double Clearance(double x, double y) const
  {
    double m = 1e9;
    for (const auto &p : lidar_points_)
      m = std::min(m, std::hypot(x - p.x, y - p.y));
    for (const auto &o : obstacles_)
    {
      if (o.is_box)
      {
        double dx = std::max(0.0, std::abs(x - o.x) - o.hx);
        double dy = std::max(0.0, std::abs(y - o.y) - o.hy);
        m = std::min(m, std::hypot(dx, dy));
      }
      else
        m = std::min(m, std::max(0.0, std::hypot(x - o.x, y - o.y) - o.r));
    }
    // include anche gli altri agenti (robot/attori)
    for (const auto &a : other_agents_)
      m = std::min(m, std::hypot(x - a.x, y - a.y));
    return m;
  }

  bool StateBlocked(double x, double y) const
  {
    // ego-bubble CONDIZIONALE: si attiva SOLO se il robot e' gia' dentro una
    // zona bloccata (intrappolato, es. ostacolo comparso sopra di lui). 
    if (escape_mode_ && std::hypot(x - ego_x_, y - ego_y_) < ego_bubble_)
      return false;
    return StateBlockedRaw(x, y);
  }

  // Freno di sicurezza (paper: emergency stop)
  bool SafetyBrake(const Action &act) const
  {
    if (act.v_cmd <= 1e-6 && std::abs(act.w_cmd) <= 1e-6)
      return false;  // gia' fermo
    const int look =
        std::max(1, static_cast<int>(std::round(brake_lookahead_ / integrator_dt_)));
    const double dmin_static = agent_radius_ + brake_margin_;
    const double cos_cone = std::cos(brake_cone_);
    const int n = static_cast<int>(act.trajectory.size());
    if (n == 0)
      return false;

    const State &r0 = act.trajectory.front();   // posa corrente del robot
    const double hx = std::cos(r0.yaw), hy = std::sin(r0.yaw);

    auto ahead = [&](double ox, double oy) {
      double vx = ox - r0.x, vy = oy - r0.y;
      double d = std::hypot(vx, vy);
      if (d < 1e-6)
        return true;
      return (vx * hx + vy * hy) / d >= cos_cone;
    };

    for (int k = 1; k <= look && k < n; ++k)
    {
      const State &p = act.trajectory[k];

      // ostacoli statici a oggetto (cilindri/box): distanza alla superficie
      for (const auto &o : obstacles_)
      {
        double dx = std::abs(p.x - o.x), dy = std::abs(p.y - o.y), dsurf;
        if (o.is_box)
          dsurf = std::hypot(std::max(0.0, dx - o.hx), std::max(0.0, dy - o.hy));
        else
          dsurf = std::max(0.0, std::hypot(p.x - o.x, p.y - o.y) - o.r);
        if (dsurf < dmin_static && ahead(o.x, o.y))
          return true;
      }

      // ostacoli statici da lidar: punto grezzo piu' vicino, solo se davanti
      for (const auto &lp : lidar_points_)
        if (std::hypot(p.x - lp.x, p.y - lp.y) < dmin_static && ahead(lp.x, lp.y))
          return true;
    }
    return false;
  }

  // true se una traiettoria entra in un ostacolo (oggetto o punto lidar)
  bool TrajectoryHitsObstacle(const std::vector<State> &t) const
  {
    for (const auto &p : t)
      if (StateBlocked(p.x, p.y))
        return true;
    return false;
  }

  // true se il segmento (a)->(b) attraversa un ostacolo
  bool SegmentBlocked(double ax, double ay, double bx, double by) const
  {
    const double d = std::hypot(bx - ax, by - ay);
    const int n = std::max(1, static_cast<int>(std::ceil(d / 0.10)));
    for (int i = 0; i <= n; ++i)
    {
      const double t = static_cast<double>(i) / n;
      if (StateBlocked(ax + t * (bx - ax), ay + t * (by - ay)))
        return true;
    }
    return false;
  }

  // costo = lunghezza percorsa + stima del completamento fino al goal
  double TrajectoryCost(const std::vector<State> &t) const
  {
    const double traveled = PathLength(t);
    const State &end = t.back();
    double remaining = std::hypot(goal_x_ - end.x, goal_y_ - end.y);
    if (SegmentBlocked(end.x, end.y, goal_x_, goal_y_))
      remaining += goal_block_penalty_;

    // corridoio strada: penalita' SOFT per ogni punto fuori dalla carreggiata
    double off = 0.0;
    if (road_keep_)
      for (const auto &p : t)
      {
        if (p.x < road_min_x_)
          off += (road_min_x_ - p.x);
        else if (p.x > road_max_x_)
          off += (p.x - road_max_x_);
      }
    return traveled + remaining + road_penalty_ * off;
  }

  // -------------------- layer lidar --------------------
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    latest_scan_ = *msg;
    have_scan_ = true;
  }

  long CellKey(double x, double y) const
  {
    long ix = static_cast<long>(std::floor((x - ws_min_x_) / lidar_grid_res_));
    long iy = static_cast<long>(std::floor((y - ws_min_y_) / lidar_grid_res_));
    long nx = static_cast<long>((ws_max_x_ - ws_min_x_) / lidar_grid_res_) + 1;
    return iy * nx + ix;
  }

  bool IsLidarOccupied(double x, double y) const
  {
    if (lidar_cells_.empty())
      return false;
    return lidar_cells_.find(CellKey(x, y)) != lidar_cells_.end();
  }

  // Inserisce nella griglia statica un disco di celle gonfiate (lidar_inflation)
  void InsertInflated(double px, double py)
  {
    const int rc = std::max(0, static_cast<int>(std::ceil(lidar_inflation_ / lidar_grid_res_)));
    const double r2 = lidar_inflation_ * lidar_inflation_;
    const long nx = static_cast<long>((ws_max_x_ - ws_min_x_) / lidar_grid_res_) + 1;
    long cix = static_cast<long>(std::floor((px - ws_min_x_) / lidar_grid_res_));
    long ciy = static_cast<long>(std::floor((py - ws_min_y_) / lidar_grid_res_));
    for (int di = -rc; di <= rc; ++di)
      for (int dj = -rc; dj <= rc; ++dj)
      {
        double cx = (cix + di) * lidar_grid_res_ + ws_min_x_;
        double cy = (ciy + dj) * lidar_grid_res_ + ws_min_y_;
        if (std::hypot(cx - px, cy - py) <= lidar_inflation_ + 1e-9 ||
            (di * di + dj * dj) * lidar_grid_res_ * lidar_grid_res_ <= r2)
          lidar_cells_.insert((ciy + dj) * nx + (cix + di));
      }
  }

  // ============================================================
  //  Percezione lidar: cluster -> tracking -> classifica statico/dinamico
  // ============================================================
  void BuildPerception()
  {
    lidar_cells_.clear();
    lidar_points_.clear();
    dyn_obs_.clear();
    if (!use_lidar_)
      return;

    // ---- raccolta punti mondo (esclude SOLO i corpi della flotta) ----
    std::vector<std::array<double, 2>> pts;
    pts.reserve(2048);
    for (size_t rbi = 0; rbi < robots_.size(); ++rbi)
    {
      const auto &rb = robots_[rbi];
      if (!rb.have_scan || !rb.have_state)
        continue;
      const auto &scan = rb.scan;
      const double rx = rb.state.x, ry = rb.state.y, ryaw = rb.state.yaw;
      const double cyaw = std::cos(ryaw), syaw = std::sin(ryaw);
      for (size_t i = 0; i < scan.ranges.size(); ++i)
      {
        const double r = scan.ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max)
          continue;
        const double a = scan.angle_min + i * scan.angle_increment;
        const double lx = r * std::cos(a), ly = r * std::sin(a);
        const double px = rx + lx * cyaw - ly * syaw;
        const double py = ry + lx * syaw + ly * cyaw;
        // scarta solo i corpi degli ALTRI robot controllati (posa nota); mai il
        // proprio centro (il lidar non vede se stesso).
        bool fleet = false;
        for (size_t rj = 0; rj < robots_.size() && !fleet; ++rj)
        {
          if (rj == rbi || !robots_[rj].have_state)
            continue;
          if (std::hypot(px - robots_[rj].state.x, py - robots_[rj].state.y) <
              lidar_robot_filter_radius_)
            fleet = true;
        }
        if (!fleet)
          pts.push_back({px, py});
      }
    }

    if (!track_dynamic_)
    {
      // tracker disattivato: tutto statico (comportamento legacy)
      for (const auto &p : pts)
      {
        lidar_points_.push_back({p[0], p[1], 0.0});
        InsertInflated(p[0], p[1]);
      }
      return;
    }

    // ---- clustering: connected-components su griglia cluster_cell_ ----
    const double cc = cluster_cell_;
    const long STR = 100000;  // stride per codificare (ccx,ccy) in una chiave
    auto key = [&](double x, double y) {
      long cx = static_cast<long>(std::floor((x - ws_min_x_) / cc)) + 50000;
      long cy = static_cast<long>(std::floor((y - ws_min_y_) / cc)) + 50000;
      return cy * STR + cx;
    };
    std::unordered_map<long, std::vector<int>> cells;
    for (int i = 0; i < static_cast<int>(pts.size()); ++i)
      cells[key(pts[i][0], pts[i][1])].push_back(i);

    struct Comp { double cx, cy; std::vector<int> idx; };
    std::vector<Comp> comps;
    std::unordered_set<long> visited;
    for (const auto &kv : cells)
    {
      if (visited.count(kv.first))
        continue;
      // BFS sulle 8 celle vicine occupate
      std::vector<long> stack{kv.first};
      visited.insert(kv.first);
      std::vector<int> members;
      while (!stack.empty())
      {
        long k = stack.back();
        stack.pop_back();
        auto it = cells.find(k);
        if (it != cells.end())
          members.insert(members.end(), it->second.begin(), it->second.end());
        long kcx = k % STR, kcy = k / STR;
        for (int di = -1; di <= 1; ++di)
          for (int dj = -1; dj <= 1; ++dj)
          {
            if (!di && !dj)
              continue;
            long nk = (kcy + dj) * STR + (kcx + di);
            if (cells.count(nk) && !visited.count(nk))
            {
              visited.insert(nk);
              stack.push_back(nk);
            }
          }
      }
      if (static_cast<int>(members.size()) < cluster_min_cells_)
        continue;
      double sx = 0, sy = 0;
      for (int m : members)
      {
        sx += pts[m][0];
        sy += pts[m][1];
      }
      Comp c;
      c.cx = sx / members.size();
      c.cy = sy / members.size();
      c.idx = std::move(members);
      comps.push_back(std::move(c));
    }

    // ---- tracking: associa cluster<->tracce, stima velocita' ----
    ros::Time now = ros::Time::now();
    double dt = 0.0;
    if (have_perc_time_)
      dt = (now - last_perc_time_).toSec();
    last_perc_time_ = now;
    have_perc_time_ = true;
    if (dt <= 1e-3 || dt > 1.0)
      dt = 0.0;  // dt non affidabile -> niente aggiornamento velocita'

    for (auto &t : tracks_)
      t.matched = false;
    std::vector<bool> comp_used(comps.size(), false);

    // greedy: per ogni componente la traccia non ancora associata piu' vicina
    for (size_t ci = 0; ci < comps.size(); ++ci)
    {
      int best = -1;
      double bestd = track_gate_;
      for (size_t ti = 0; ti < tracks_.size(); ++ti)
      {
        if (tracks_[ti].matched)
          continue;
        double d = std::hypot(comps[ci].cx - tracks_[ti].x, comps[ci].cy - tracks_[ti].y);
        if (d < bestd)
        {
          bestd = d;
          best = static_cast<int>(ti);
        }
      }
      if (best >= 0)
      {
        Track &t = tracks_[best];
        // velocita' su finestra temporale (~vel_window_ s), non frame-frame:
        // il movimento delle gambe non falsa piu' la stima
        if (!t.has_anchor)
        {
          t.ax = comps[ci].cx;
          t.ay = comps[ci].cy;
          t.at = now;
          t.has_anchor = true;
        }
        else
        {
          double dtw = (now - t.at).toSec();
          if (dtw >= vel_window_)
          {
            double mvx = (comps[ci].cx - t.ax) / dtw;
            double mvy = (comps[ci].cy - t.ay) / dtw;
            // scarta velocita' impossibili (glitch di associazione), poi EMA
            if (std::hypot(mvx, mvy) <= 1.5 * ped_max_speed_)
            {
              t.vx = 0.5 * t.vx + 0.5 * mvx;
              t.vy = 0.5 * t.vy + 0.5 * mvy;
            }
            t.ax = comps[ci].cx;
            t.ay = comps[ci].cy;
            t.at = now;
          }
        }
        t.x = comps[ci].cx;   // posizione = centroide corrente (assoluto, mondo)
        t.y = comps[ci].cy;
        t.hits++;
        t.misses = 0;
        t.matched = true;
        comp_used[ci] = true;
        // latch dinamico: se confermata e in movimento, (ri)arma il latch.
        if (t.hits >= track_min_hits_ &&
            std::hypot(t.vx, t.vy) >= dyn_speed_thresh_)
        {
          t.dyn_latch = dyn_latch_frames_;
          t.ever_dynamic = true;  // confermata mobile -> pedone per sempre
        }
        else if (t.dyn_latch > 0)
          --t.dyn_latch;
      }
    }
    // componenti non associate -> nuove tracce
    for (size_t ci = 0; ci < comps.size(); ++ci)
    {
      if (comp_used[ci])
        continue;
      Track t;
      t.id = next_track_id_++;
      t.x = comps[ci].cx;
      t.y = comps[ci].cy;
      t.hits = 1;
      t.matched = true;
      tracks_.push_back(t);
    }
    // tracce non associate -> miss; rimosse dopo troppi miss
    for (auto &t : tracks_)
      if (!t.matched)
        t.misses++;
    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const Track &t) { return t.misses > track_max_misses_; }),
                  tracks_.end());

    // ---- classifica e costruisci griglia statica + lista dinamici ----
    for (size_t ci = 0; ci < comps.size(); ++ci)
    {
      // traccia piu' vicina (gia' aggiornata sopra)
      int near = -1;
      double bd = track_gate_;
      for (size_t ti = 0; ti < tracks_.size(); ++ti)
      {
        double d = std::hypot(comps[ci].cx - tracks_[ti].x, comps[ci].cy - tracks_[ti].y);
        if (d < bd)
        {
          bd = d;
          near = static_cast<int>(ti);
        }
      }
      bool dynamic = false;
      double vx = 0, vy = 0;
      if (near >= 0 && tracks_[near].ever_dynamic)
      {
        dynamic = true;
        vx = tracks_[near].vx;
        vy = tracks_[near].vy;
      }
      if (dynamic)
      {
        DynObs d;
        d.pos.x = comps[ci].cx;
        d.pos.y = comps[ci].cy;
        d.pos.yaw = std::atan2(vy, vx);
        d.vx = vx;
        d.vy = vy;
        dyn_obs_.push_back(d);
      }
      else
      {
        // statico (o non ancora confermato): in griglia, cosi' viene evitato
        for (int m : comps[ci].idx)
        {
          lidar_points_.push_back({pts[m][0], pts[m][1], 0.0});
          InsertInflated(pts[m][0], pts[m][1]);
        }
      }
    }
  }

  // Traiettoria predetta di un pedone: moto rettilineo uniforme con la velocita'
  // stimata, ricampionata a integrator_dt per num_traj_steps passi.
  std::vector<Action> PredictDynObs(const DynObs &d) const
  {
    double sp = std::hypot(d.vx, d.vy);
    sp = std::min(sp, ped_max_speed_);
    double yaw = (sp > 1e-3) ? std::atan2(d.vy, d.vx) : d.pos.yaw;

    Action a;
    a.name = "pred";
    a.v_cmd = sp;
    a.w_cmd = 0.0;
    State s = d.pos;
    s.yaw = yaw;
    a.trajectory.push_back(s);
    // orizzonte di predizione PIU' LUNGO delle azioni robot: cosi' lo scontro
    // frontale viene rilevato da lontano e il bot puo' deviare in anticipo.
    const int steps = std::max(1, static_cast<int>(
                                     std::round(ped_predict_horizon_ / integrator_dt_)));
    for (int k = 0; k < steps; ++k)
    {
      s = StepUnicycle(s, sp, 0.0, integrator_dt_);
      a.trajectory.push_back(s);
    }
    a.cost = PathLength(a.trajectory);
    return {a};
  }

  // true se (px,py) e' dentro un ostacolo gonfiato del raggio agente
  bool InsideObstacle(double px, double py) const
  {
    for (const auto &o : obstacles_)
    {
      const double inf = agent_radius_ + static_margin_;  // margine di sicurezza statico
      if (o.is_box)
      {
        if (std::abs(px - o.x) <= o.hx + inf &&
            std::abs(py - o.y) <= o.hy + inf)
          return true;
      }
      else if (std::hypot(px - o.x, py - o.y) < o.r + inf)
        return true;
    }
    return false;
  }

  void AdvanceGoal(ControlledRobot &r)
  {
    if (r.goal_idx + 1 < r.goals.size())
      ++r.goal_idx;
    else if (r.loop_goals)
      r.goal_idx = 0;
    else
    {
      r.final_reached = true;
      return;
    }
    r.goal_x = r.goals[r.goal_idx].x;
    r.goal_y = r.goals[r.goal_idx].y;
    // nuovo goal: azzera il riferimento di isteresi del vecchio lato, cosi' il
    // robot ridecide liberamente la direzione verso il nuovo goal.
    r.have_prev = false;
    ROS_INFO("[gt_planner] [%s] nuovo goal %zu/%zu: %.2f %.2f",
             r.name.c_str(), r.goal_idx + 1, r.goals.size(), r.goal_x, r.goal_y);
  }

  // ============================================================
  //  OMPL  -  RRT control-based (Sect. 4.3.1)
  // ============================================================
  void SetupOmpl()
  {
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);  // silenzia gli Info di OMPL

    auto se2 = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, ws_min_x_);
    bounds.setHigh(0, ws_max_x_);
    bounds.setLow(1, ws_min_y_);
    bounds.setHigh(1, ws_max_y_);
    se2->setBounds(bounds);
    space_ = se2;

    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space_, 2);
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0, 0.0);          // v >= 0
    cbounds.setHigh(0, robot_v_);
    cbounds.setLow(1, -w_max_);      // w in [-w_max, w_max]
    cbounds.setHigh(1, w_max_);
    cspace->setBounds(cbounds);
    cspace_ = cspace;

    ss_ = std::make_shared<oc::SimpleSetup>(cspace_);

    // propagatore unicycle integrato a passo integrator_dt
    const double idt = integrator_dt_;
    ss_->setStatePropagator(
      [idt](const ob::State *from, const oc::Control *ctrl, const double dur, ob::State *to)
      {
        const auto *s = from->as<ob::SE2StateSpace::StateType>();
        const double *u = ctrl->as<oc::RealVectorControlSpace::ControlType>()->values;
        const double v = u[0];
        const double w = u[1];

        double x = s->getX();
        double y = s->getY();
        double yaw = s->getYaw();

        int steps = std::max(1, static_cast<int>(std::round(dur / idt)));
        for (int k = 0; k < steps; ++k)
        {
          x += idt * v * std::cos(yaw);
          y += idt * v * std::sin(yaw);
          yaw = std::atan2(std::sin(yaw + idt * w), std::cos(yaw + idt * w));
        }

        auto *o = to->as<ob::SE2StateSpace::StateType>();
        o->setX(x);
        o->setY(y);
        o->setYaw(yaw);
      });

    // spazio ammissibile X^adm = workspace meno ostacoli statici X^obj
    ss_->setStateValidityChecker(
      [this](const ob::State *s) -> bool {
        const auto *se2 = s->as<ob::SE2StateSpace::StateType>();
        return !StateBlocked(se2->getX(), se2->getY());
      });

    ss_->getSpaceInformation()->setPropagationStepSize(integrator_dt_);

    auto planner = std::make_shared<oc::RRT>(ss_->getSpaceInformation());
    ss_->setPlanner(planner);
    ss_->setup();
  }

  // genera M_n traiettorie diverse verso il goal corrente
  std::vector<Action> GenerateRobotActions(const State &robot)
  {
    std::vector<Action> actions;
    auto si = ss_->getSpaceInformation();

    for (int m = 0; m < robot_num_actions_; ++m)
    {
      // w_n estratto a caso in [w_min,w_max]; bound di delta_t da Gamma^min/max
      const double w = w_min_ + rng_.uniformReal(0.0, 1.0) * (w_max_ - w_min_);
      const double dtmin = prop_min_lo_ + rng_.uniformReal(0.0, 1.0) * (prop_min_hi_ - prop_min_lo_);
      const double dtmax = prop_max_lo_ + rng_.uniformReal(0.0, 1.0) * (prop_max_hi_ - prop_max_lo_);

      int min_steps = std::max(1, static_cast<int>(std::round(dtmin / integrator_dt_)));
      int max_steps = std::max(min_steps, static_cast<int>(std::round(dtmax / integrator_dt_)));
      si->setMinMaxControlDuration(min_steps, max_steps);

      const double vv = robot_v_;
      const double cc = curvature_factor_;
      cspace_->setControlSamplerAllocator(
        [vv, w, cc](const oc::ControlSpace *cs) -> oc::ControlSamplerPtr
        { return std::make_shared<Eq8ControlSampler>(cs, vv, w, cc); });

      ss_->clear();

      // clamp dentro i bound: uno start/goal fuori bound fa fallire l'RRT
      const double eps = 1e-3;
      const double sx = std::min(std::max(robot.x, ws_min_x_ + eps), ws_max_x_ - eps);
      const double sy = std::min(std::max(robot.y, ws_min_y_ + eps), ws_max_y_ - eps);
      const double gx = std::min(std::max(goal_x_, ws_min_x_ + eps), ws_max_x_ - eps);
      const double gy = std::min(std::max(goal_y_, ws_min_y_ + eps), ws_max_y_ - eps);

      ob::ScopedState<ob::SE2StateSpace> start(space_);
      start->setX(sx);
      start->setY(sy);
      start->setYaw(robot.yaw);

      ob::ScopedState<ob::SE2StateSpace> goal(space_);
      goal->setX(gx);
      goal->setY(gy);
      goal->setYaw(std::atan2(goal_y_ - robot.y, goal_x_ - robot.x));

      const double thr = 0.5 * std::hypot(goal_region_x_, goal_region_y_);
      ss_->setStartAndGoalStates(start, goal, thr);

      ss_->solve(rrt_solve_time_);

      if (!ss_->haveSolutionPath())
        continue;

      oc::PathControl path = ss_->getSolutionPath();
      path.interpolate();

      Action a;
      a.name = "rrt_" + std::to_string(m);

      const auto &states = path.getStates();
      for (const auto *st : states)
      {
        const auto *se2 = st->as<ob::SE2StateSpace::StateType>();
        State p;
        p.x = se2->getX();
        p.y = se2->getY();
        p.yaw = se2->getYaw();
        a.trajectory.push_back(p);
      }
      if (a.trajectory.empty())
        continue;

      if (path.getControlCount() > 0)
      {
        const double *u =
          path.getControl(0)->as<oc::RealVectorControlSpace::ControlType>()->values;
        a.v_cmd = u[0];
        a.w_cmd = u[1];
      }

      a.cost = TrajectoryCost(a.trajectory);

      actions.push_back(a);
    }

    // Azioni fallback deterministiche, aggiunte SOLO se l'RRT non ha prodotto nessuna traiettoria
    if (actions.empty())
    {
      auto add_arc = [&](double v, double w, const std::string &nm) {
        Action a;
        a.v_cmd = v;
        a.w_cmd = w;
        a.name = nm;
        State s = robot;
        a.trajectory.push_back(s);
        for (int k = 0; k < num_traj_steps_; ++k)
        {
          s = StepUnicycle(s, v, w, integrator_dt_);
          a.trajectory.push_back(s);
        }
        a.cost = TrajectoryCost(a.trajectory);
        actions.push_back(a);
      };
      const double W = w_max_, c = curvature_factor_;
      add_arc(robot_v_, 0.0, "arc_fwd");
      add_arc(robot_v_, W, "arc_L");
      add_arc(robot_v_, -W, "arc_R");
      add_arc(robot_v_, c * W, "arc_sL");
      add_arc(robot_v_, -c * W, "arc_sR");
    }


    // NB: l'azione "memorizzata" (ri-offrire la traiettoria scelta al tick precedente) e' stata RIMOSSA.
    // Sostituita con l'isteresi
    double max_cost = 0.0;
    for (const auto &a : actions)
      max_cost = std::max(max_cost, a.cost);

    Action stop;
    stop.name = "stand_still";
    stop.v_cmd = 0.0;
    stop.w_cmd = 0.0;
    stop.trajectory.assign(num_traj_steps_ + 1, robot);
    stop.cost = max_cost * 1.1 + 0.05;
    actions.push_back(stop);

    // marca le azioni che attraversano un corridoio di gruppo o un ostacolo
    MarkGroupBlocked(actions);
    for (auto &a : actions)
      a.obstacle_blocked = TrajectoryHitsObstacle(a.trajectory);

    // ESCAPE MODE: 
    if (escape_mode_)
      for (auto &a : actions)
        a.cost = -Clearance(a.trajectory.back().x, a.trajectory.back().y);

    return actions;
  }

  // ============================================================
  //  Predizione attori (umani) - action set predetto
  // ============================================================
  State EstimateActorVelocity(const std::string &name, const State &cur)
  {
    State vel;
    ros::Time now = ros::Time::now();
    auto is = previous_actor_states_.find(name);
    auto it = previous_actor_times_.find(name);

    if (is == previous_actor_states_.end() || it == previous_actor_times_.end())
    {
      previous_actor_states_[name] = cur;
      previous_actor_times_[name] = now;
      return vel;
    }

    double dt = (now - it->second).toSec();
    if (dt > 1e-3 && dt < 1.0)
    {
      vel.x = (cur.x - is->second.x) / dt;
      vel.y = (cur.y - is->second.y) / dt;
    }
    return vel;  // gli stati precedenti vengono aggiornati una sola volta per tick
  }

  std::vector<Action> GenerateActorPrediction(const std::string &name, const State &actor)
  {
    State vel = EstimateActorVelocity(name, actor);
    double speed = std::hypot(vel.x, vel.y);
    double base_yaw = (speed > min_actor_speed_) ? std::atan2(vel.y, vel.x) : actor.yaw;
    if (speed < min_actor_speed_)
      speed = default_actor_speed_;
    speed = std::max(min_actor_speed_, std::min(speed, max_actor_speed_));

    // ventaglio simmetrico di sterzate
    std::vector<double> turns;
    int n = std::max(1, actor_num_actions_);
    if (n == 1)
      turns = {0.0};
    else
    {
      for (int i = 0; i < n; ++i)
      {
        double frac = (n == 1) ? 0.0 : (2.0 * i / (n - 1) - 1.0);  // [-1,1]
        turns.push_back(frac * actor_turn_rate_);
      }
    }

    std::vector<Action> actions;
    for (size_t i = 0; i < turns.size(); ++i)
    {
      Action a;
      a.name = "pred_" + std::to_string(i);
      a.v_cmd = speed;
      a.w_cmd = turns[i];

      State s = actor;
      s.yaw = base_yaw;
      a.trajectory.push_back(s);
      for (int k = 0; k < num_traj_steps_; ++k)
      {
        s = StepUnicycle(s, speed, turns[i], integrator_dt_);
        a.trajectory.push_back(s);
      }
      // costo predizione: lunghezza + piccolo prior di rettilineita'
      a.cost = PathLength(a.trajectory) + 0.5 * std::abs(turns[i]);
      actions.push_back(a);
    }
    return actions;
  }

  // ============================================================
  //  Vincoli: collisione e corridoio di gruppo
  // ============================================================
  bool TrajectoriesCollide(const std::vector<State> &a, const std::vector<State> &b,
                           double dmin) const
  {
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i)
      if (std::hypot(a[i].x - b[i].x, a[i].y - b[i].y) < dmin)
        return true;
    return false;
  }

  // distanza minima centro-centro tra due traiettorie e l'ISTANTE (s) in cui avviene
  void ClosestApproach(const std::vector<State> &a, const std::vector<State> &b,
                       double &dmin, double &tmin) const
  {
    dmin = 1e9;
    tmin = -1.0;
    const size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i)
    {
      double d = std::hypot(a[i].x - b[i].x, a[i].y - b[i].y);
      if (d < dmin)
      {
        dmin = d;
        tmin = i * integrator_dt_;
      }
    }
  }

  // distanza punto-segmento (per il corridoio del gruppo)
  static double PointSegDist(double px, double py,
                             double ax, double ay, double bx, double by)
  {
    double vx = bx - ax, vy = by - ay;
    double wx = px - ax, wy = py - ay;
    double c1 = vx * wx + vy * wy;
    double c2 = vx * vx + vy * vy;
    double t = (c2 > 1e-9) ? std::max(0.0, std::min(1.0, c1 / c2)) : 0.0;
    double projx = ax + t * vx, projy = ay + t * vy;
    return std::hypot(px - projx, py - projy);
  }

  // segna le azioni robot che passano nel corridoio tra membri di un gruppo
  void MarkGroupBlocked(std::vector<Action> &robot_actions) const
  {
    if (group_pairs_.empty())
      return;

    std::vector<std::pair<State, State>> corridors;
    for (const auto &pair : group_pairs_)
    {
      State l, r;
      if (FindActorState(pair.first, l) && FindActorState(pair.second, r))
        corridors.emplace_back(l, r);
    }
    if (corridors.empty())
      return;

    for (auto &a : robot_actions)
    {
      for (const auto &c : corridors)
      {
        for (const auto &p : a.trajectory)
        {
          double d = PointSegDist(p.x, p.y, c.first.x, c.first.y, c.second.x, c.second.y);
          if (d < group_corridor_half_width_)
          {
            a.group_blocked = true;
            break;
          }
        }
        if (a.group_blocked)
          break;
      }
    }
  }

  bool FindActorState(const std::string &name, State &out) const
  {
    return ExtractState(work_msg_, name, out);
  }

  // ============================================================
  //  Gioco statico: tabella dei costi, Nash, Pareto
  // ============================================================
  std::vector<std::vector<int>> EnumerateAllocations(const std::vector<AgentActions> &agents) const
  {
    std::vector<std::vector<int>> out;
    if (agents.empty())
      return out;
    std::vector<int> cur(agents.size(), 0);
    while (true)
    {
      out.push_back(cur);
      int idx = static_cast<int>(agents.size()) - 1;
      while (idx >= 0)
      {
        if (++cur[idx] < static_cast<int>(agents[idx].actions.size()))
          break;
        cur[idx] = 0;
        --idx;
      }
      if (idx < 0)
        break;
    }
    return out;
  }

  std::vector<std::vector<double>> BuildCostTable(
    const std::vector<AgentActions> &agents,
    const std::vector<std::vector<int>> &alloc) const
  {
    std::vector<std::vector<double>> costs(alloc.size(),
                                           std::vector<double>(agents.size(), 0.0));
    for (size_t r = 0; r < alloc.size(); ++r)
    {
      std::vector<bool> collided(agents.size(), false);

      // J_tilde: collisione tra coppie di agenti
      for (size_t i = 0; i < agents.size(); ++i)
        for (size_t j = i + 1; j < agents.size(); ++j)
        {
          double dmin = (agents[i].is_robot && agents[j].is_robot)
                            ? robot_collision_dist_
                            : dyn_collision_dist_;
          if (TrajectoriesCollide(agents[i].actions[alloc[r][i]].trajectory,
                                  agents[j].actions[alloc[r][j]].trajectory, dmin))
          {
            collided[i] = true;
            collided[j] = true;
          }
        }

      for (size_t i = 0; i < agents.size(); ++i)
      {
        const Action &act = agents[i].actions[alloc[r][i]];
        if (collided[i] ||
            (agents[i].is_robot && (act.group_blocked || act.obstacle_blocked)))
          costs[r][i] = kInf;            // J_tilde = INF
        else
          costs[r][i] = act.cost;        // J = Jhat = Length
      }

      // Penalita' SOFT di comfort
      for (size_t i = 0; i < agents.size(); ++i)
      {
        if (!agents[i].is_robot || costs[r][i] >= 0.5 * kInf)
          continue;
        for (size_t j = 0; j < agents.size(); ++j)
        {
          if (agents[j].is_robot)
            continue;  // solo robot-pedone (i robot hanno gia' il loro buffer)
          double d, t;
          ClosestApproach(agents[i].actions[alloc[r][i]].trajectory,
                          agents[j].actions[alloc[r][j]].trajectory, d, t);
          if (d < ped_comfort_dist_)
            costs[r][i] += ped_comfort_weight_ * (ped_comfort_dist_ - d);
        }
      }
    }
    return costs;
  }

  bool IsNash(size_t row,
              const std::vector<AgentActions> &agents,
              const std::vector<std::vector<int>> &alloc,
              const std::vector<std::vector<double>> &costs) const
  {
    const double eps = 1e-9;
    for (size_t ag = 0; ag < agents.size(); ++ag)
    {
      double cur = costs[row][ag];
      for (size_t alt = 0; alt < agents[ag].actions.size(); ++alt)
      {
        if (static_cast<int>(alt) == alloc[row][ag])
          continue;
        std::vector<int> aa = alloc[row];
        aa[ag] = static_cast<int>(alt);
        auto it = allocation_to_row_.find(aa);
        if (it == allocation_to_row_.end())
          continue;
        if (costs[it->second][ag] + eps < cur)
          return false;  // esiste deviazione unilaterale migliore -> non Nash
      }
    }
    return true;
  }

  std::vector<size_t> FindNash(const std::vector<AgentActions> &agents,
                               const std::vector<std::vector<int>> &alloc,
                               const std::vector<std::vector<double>> &costs)
  {
    allocation_to_row_.clear();
    for (size_t i = 0; i < alloc.size(); ++i)
      allocation_to_row_[alloc[i]] = i;

    std::vector<size_t> nash;
    for (size_t r = 0; r < alloc.size(); ++r)
      if (IsNash(r, agents, alloc, costs))
        nash.push_back(r);
    return nash;
  }

  std::vector<size_t> ParetoFilter(const std::vector<size_t> &rows,
                                   const std::vector<std::vector<double>> &costs) const
  {
    std::vector<size_t> out;
    for (size_t i = 0; i < rows.size(); ++i)
    {
      bool dominated = false;
      for (size_t j = 0; j < rows.size() && !dominated; ++j)
      {
        if (i == j)
          continue;
        bool all_leq = true, one_lt = false;
        for (size_t a = 0; a < costs[rows[i]].size(); ++a)
        {
          if (costs[rows[j]][a] > costs[rows[i]][a])
            all_leq = false;
          if (costs[rows[j]][a] < costs[rows[i]][a])
            one_lt = true;
        }
        if (all_leq && one_lt)
          dominated = true;
      }
      if (!dominated)
        out.push_back(rows[i]);
    }
    return out;
  }

  // ============================================================
  //  Coordinamento tra umani
  // ============================================================
  std::vector<EqRecord> BuildEqRecords(const std::vector<size_t> &rows,
                                       const std::vector<AgentActions> &agents,
                                       const std::vector<std::vector<int>> &alloc,
                                       const std::vector<std::vector<double>> &costs) const
  {
    std::vector<EqRecord> recs;
    for (size_t r : rows)
    {
      EqRecord e;
      e.row = r;
      e.robot_cost = costs[r][0];
      e.robot_traj = agents[0].actions[alloc[r][0]].trajectory;
      for (size_t i = 1; i < agents.size(); ++i)  // i=0 e' il robot
        e.actor_traj[agents[i].name] = agents[i].actions[alloc[r][i]].trajectory;
      recs.push_back(e);
    }
    return recs;
  }

  // distanza tra due traiettorie come distanza tra i versori di direzione
  static double DirDistance(const std::vector<State> &a, const std::vector<State> &b)
  {
    double ax, ay, bx, by;
    TrajDir(a, ax, ay);
    TrajDir(b, bx, by);
    return std::hypot(ax - bx, ay - by);
  }

  // sceglie l'equilibrio epsilon* 
  size_t SelectEquilibrium(const std::vector<EqRecord> &cur_eqs,
                           const std::vector<AgentActions> &agents)
  {
    if (cur_eqs.empty())
      return 0;

    // direzioni osservate degli attori nell'ultimo Delta t
    std::map<std::string, std::pair<double, double>> obs;
    for (size_t i = 1; i < agents.size(); ++i)
    {
      const std::string &name = agents[i].name;
      auto it = obs_prev_state_.find(name);
      if (it == obs_prev_state_.end())
        continue;
      double dx = agents[i].state.x - it->second.x;
      double dy = agents[i].state.y - it->second.y;
      double n = std::hypot(dx, dy);
      if (n > 1e-4)
        obs[name] = {dx / n, dy / n};
    }

    // fallback  
    if (prev_eqs_.empty() || obs.empty())
      return LowestRobotCostRow(cur_eqs);

    const EqRecord *prev_best = nullptr;
    double best_obs_score = std::numeric_limits<double>::max();
    for (const auto &pe : prev_eqs_)
    {
      double score = 0.0;
      int cnt = 0;
      for (const auto &kv : obs)
      {
        auto it = pe.actor_traj.find(kv.first);
        if (it == pe.actor_traj.end())
          continue;
        double dx, dy;
        TrajDir(it->second, dx, dy);
        score += std::hypot(dx - kv.second.first, dy - kv.second.second);
        ++cnt;
      }
      if (cnt == 0)
        continue;
      score /= cnt;
      if (score < best_obs_score)
      {
        best_obs_score = score;
        prev_best = &pe;
      }
    }

    if (!prev_best)
      return LowestRobotCostRow(cur_eqs);

    // tra gli equilibri CORRENTI, quello piu' somigliante a prev_best
    size_t best_row = cur_eqs.front().row;
    double best_res = std::numeric_limits<double>::max();
    double best_cost = std::numeric_limits<double>::max();
    for (const auto &ce : cur_eqs)
    {
      double score = 0.0;
      int cnt = 0;
      for (const auto &kv : ce.actor_traj)
      {
        auto it = prev_best->actor_traj.find(kv.first);
        if (it == prev_best->actor_traj.end())
          continue;
        score += DirDistance(kv.second, it->second);
        ++cnt;
      }
      if (cnt > 0)
        score /= cnt;
      // + isteresi
      double total = score + HysteresisPenalty(ce.robot_traj);
      if (total < best_res - 1e-6 ||
          (std::abs(total - best_res) <= 1e-6 && ce.robot_cost < best_cost))
      {
        best_res = total;
        best_cost = ce.robot_cost;
        best_row = ce.row;
      }
    }
    return best_row;
  }

  // penalita' di isteresi 
  void EarlyDir(const std::vector<State> &t, double &dx, double &dy) const
  {
    dx = 0.0;
    dy = 0.0;
    if (t.size() < 2)
      return;
    size_t ke = std::min(t.size() - 1,
                         static_cast<size_t>(std::max(1, static_cast<int>(std::round(1.0 / integrator_dt_)))));
    double ex = t[ke].x - t[0].x;
    double ey = t[ke].y - t[0].y;
    double n = std::hypot(ex, ey);
    if (n > 1e-6)
    {
      dx = ex / n;
      dy = ey / n;
    }
  }

  double HysteresisPenalty(const std::vector<State> &traj) const
  {
    if (!have_prev_robot_traj_ || escape_mode_)
      return 0.0;
    double ax, ay, bx, by;
    EarlyDir(traj, ax, ay);
    EarlyDir(prev_robot_traj_, bx, by);
    return hysteresis_weight_ * std::hypot(ax - bx, ay - by);
  }

  size_t LowestRobotCostRow(const std::vector<EqRecord> &eqs) const
  {
    size_t best = eqs.front().row;
    double bs = std::numeric_limits<double>::max();
    for (const auto &e : eqs)
    {
      double s = e.robot_cost + HysteresisPenalty(e.robot_traj);
      if (s < bs)
      {
        bs = s;
        best = e.row;
      }
    }
    return best;
  }

  // ============================================================
  //  Estrazione stato da /gazebo/model_states
  // ============================================================
  bool ExtractState(const gazebo_msgs::ModelStates &msg, const std::string &name, State &out) const
  {
    auto it = std::find(msg.name.begin(), msg.name.end(), name);
    if (it == msg.name.end())
      return false;
    size_t idx = std::distance(msg.name.begin(), it);
    out.x = msg.pose[idx].position.x;
    out.y = msg.pose[idx].position.y;
    out.yaw = YawFromQuat(msg.pose[idx].orientation);
    return true;
  }

  void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    latest_msg_ = *msg;
    have_model_states_ = true;
  }

  // ============================================================
  //  Controller ad alta frequenza (pure-pursuit della traiettoria)
  // ============================================================
  void ControlPublish(const ros::TimerEvent &)
  {
    gazebo_msgs::ModelStates msg;
    bool have;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      have = have_model_states_;
      if (have)
        msg = latest_msg_;
    }
    if (!have)
      return;

    std::string clog;
    for (auto &r : robots_)
    {
      // snapshot del piano
      std::vector<State> path;
      bool stop, has_plan, final_reached;
      {
        std::lock_guard<std::mutex> lk(plan_mutex_);
        path = r.follow_path;
        stop = r.follow_stop;
        has_plan = r.have_plan;
        final_reached = r.final_reached;
      }

      geometry_msgs::Twist cmd;  // default 0
      State cur;
      const bool have_pose = ExtractState(msg, r.name, cur);
      // fermo totale entro goal_tolerance
      const bool at_goal =
          have_pose && std::hypot(r.goal_x - cur.x, r.goal_y - cur.y) < goal_tolerance_;
      const bool moving =
          have_pose && !final_reached && !stop && !at_goal && has_plan && path.size() > 1;
      if (moving)
        cmd = PurePursuit(cur, path, r.goal_x, r.goal_y);
      // smoothing del comando angolare
      double w_smooth = moving ? (w_ema_ * r.last_w_pub + (1.0 - w_ema_) * cmd.angular.z) : 0.0;
      cmd.angular.z = w_smooth;
      r.last_w_pub = w_smooth;
      r.cmd_pub.publish(cmd);

      if (enable_debug_)
      {
        char cb[96];
        std::snprintf(cb, sizeof(cb), "[%s pub=%.2f/%.2f%s%s%s] ", r.name.c_str(),
                      cmd.linear.x, cmd.angular.z, stop ? " STOP" : "",
                      at_goal ? " ATGOAL" : "", final_reached ? " FIN" : "");
        clog += cb;
      }
    }
    if (enable_debug_)
      ROS_INFO_THROTTLE(0.5, "[gt_planner][ctrl] %s", clog.c_str());
  }

  geometry_msgs::Twist PurePursuit(const State &cur, const std::vector<State> &path,
                                   double gx, double gy) const
  {
    geometry_msgs::Twist cmd;
    // indice del punto piu' vicino alla posa corrente
    size_t nearest = 0;
    double bd = 1e18;
    for (size_t i = 0; i < path.size(); ++i)
    {
      double d = std::hypot(path[i].x - cur.x, path[i].y - cur.y);
      if (d < bd) { bd = d; nearest = i; }
    }
    // avanza fino al primo punto a distanza >= lookahead
    size_t li = nearest;
    for (size_t i = nearest; i < path.size(); ++i)
    {
      if (std::hypot(path[i].x - cur.x, path[i].y - cur.y) >= lookahead_)
      {
        li = i;
        break;
      }
      li = i;
    }
    const double lx = path[li].x, ly = path[li].y;

    double alpha = std::atan2(ly - cur.y, lx - cur.x) - cur.yaw;
    alpha = std::atan2(std::sin(alpha), std::cos(alpha));  // wrap

    double v = robot_v_;
    // rallenta avvicinandosi al goal vero
    const double dgoal = std::hypot(gx - cur.x, gy - cur.y);
    if (dgoal < lookahead_)
      v *= std::max(0.2, dgoal / lookahead_);
    // rallentamento in curva DOLCE 
    {
      double turn_scale = 1.0 - 0.5 * (std::abs(alpha) / M_PI);  // alpha=pi -> 0.5
      v *= std::max(0.55, turn_scale);
    }

    double w = 2.0 * v * std::sin(alpha) / std::max(1e-3, lookahead_);
    w = std::max(-w_max_, std::min(w_max_, w));

    cmd.linear.x = v;
    cmd.angular.z = w;
    return cmd;
  }

  // ============================================================
  //  Visualizzazione RViz
  // ============================================================
  void PublishMarkers(const std::vector<AgentActions> &agents,
                      const std::vector<size_t> &nash,
                      const std::vector<size_t> &pareto,
                      const std::vector<std::vector<int>> &alloc,
                      size_t selected_row,
                      bool valid,
                      size_t n_controlled)
  {
    if (!publish_markers_)
      return;

    visualization_msgs::MarkerArray arr;

    visualization_msgs::Marker del;
    del.header.frame_id = marker_frame_;
    del.header.stamp = ros::Time::now();
    del.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(del);

    int id = 0;
    auto make_line = [&](const std::vector<State> &t, double w,
                         double r, double g, double b, double alpha,
                         const std::string &ns) {
      visualization_msgs::Marker m;
      m.header.frame_id = marker_frame_;
      m.header.stamp = ros::Time::now();
      m.ns = ns;
      m.id = id++;
      m.type = visualization_msgs::Marker::LINE_STRIP;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = w;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = alpha;
      m.pose.orientation.w = 1.0;
      for (const auto &p : t)
      {
        geometry_msgs::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.05;
        m.points.push_back(pt);
      }
      arr.markers.push_back(m);
    };

    // traiettorie valutate di TUTTI i robot controllati (grigio sottile)
    for (size_t i = 0; i < n_controlled; ++i)
      for (const auto &a : agents[i].actions)
      {
        bool blocked = a.group_blocked || a.obstacle_blocked;
        make_line(a.trajectory, 0.02,
                  blocked ? 0.6 : 0.5, blocked ? 0.1 : 0.5, blocked ? 0.1 : 0.5,
                  0.5, "robot_actions");
      }

    // predizioni attori (arancione sottile)
    for (size_t i = n_controlled; i < agents.size(); ++i)
      for (const auto &a : agents[i].actions)
        make_line(a.trajectory, 0.02, 1.0, 0.55, 0.0, 0.45, "actor_pred");

    // equilibri di Nash (azzurro) - traiettoria del robot 0 come rappresentante
    for (size_t r : nash)
      make_line(agents[0].actions[alloc[r][0]].trajectory, 0.035, 0.1, 0.6, 1.0, 0.7, "nash");

    // equilibri Pareto (blu)
    for (size_t r : pareto)
      make_line(agents[0].actions[alloc[r][0]].trajectory, 0.05, 0.0, 0.2, 1.0, 0.85, "pareto");

    // epsilon* scelto: traiettoria di OGNI robot controllato (verde spesso)
    if (valid)
      for (size_t i = 0; i < n_controlled; ++i)
        make_line(agents[i].actions[alloc[selected_row][i]].trajectory,
                  0.09, 0.0, 1.0, 0.0, 1.0, "chosen");

    // corridoi di gruppo (rosso)
    for (const auto &pair : group_pairs_)
    {
      State l, r;
      if (FindActorState(pair.first, l) && FindActorState(pair.second, r))
      {
        std::vector<State> seg = {l, r};
        make_line(seg, 0.12, 1.0, 0.0, 0.0, 0.9, "group");
      }
    }

    // ostacoli statici (grigio semitrasparente)
    for (const auto &o : obstacles_)
    {
      visualization_msgs::Marker m;
      m.header.frame_id = marker_frame_;
      m.header.stamp = ros::Time::now();
      m.ns = "obstacles";
      m.id = id++;
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = o.x;
      m.pose.position.y = o.y;
      m.pose.position.z = 0.3;
      m.pose.orientation.w = 1.0;
      m.color.r = 0.5;
      m.color.g = 0.5;
      m.color.b = 0.55;
      m.color.a = 0.6;
      if (o.is_box)
      {
        m.type = visualization_msgs::Marker::CUBE;
        m.scale.x = 2.0 * o.hx;
        m.scale.y = 2.0 * o.hy;
        m.scale.z = 0.6;
      }
      else
      {
        m.type = visualization_msgs::Marker::CYLINDER;
        m.scale.x = 2.0 * o.r;
        m.scale.y = 2.0 * o.r;
        m.scale.z = 1.0;
      }
      arr.markers.push_back(m);
    }

    // punti lidar trattati come ostacolo (giallo)
    if (!lidar_points_.empty())
    {
      visualization_msgs::Marker m;
      m.header.frame_id = marker_frame_;
      m.header.stamp = ros::Time::now();
      m.ns = "lidar_obs";
      m.id = id++;
      m.type = visualization_msgs::Marker::POINTS;
      m.action = visualization_msgs::Marker::ADD;
      m.scale.x = 0.06;
      m.scale.y = 0.06;
      m.color.r = 1.0;
      m.color.g = 0.9;
      m.color.b = 0.0;
      m.color.a = 0.9;
      m.pose.orientation.w = 1.0;
      for (const auto &p : lidar_points_)
      {
        geometry_msgs::Point pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.z = 0.1;
        m.points.push_back(pt);
      }
      arr.markers.push_back(m);
    }

    marker_pub_.publish(arr);
  }

  // Richiede lo stop di tutti i robot 
  void PublishStop()
  {
    std::lock_guard<std::mutex> lk(plan_mutex_);
    for (auto &r : robots_)
      r.follow_stop = true;
  }

  // selezione dell'equilibrio (Pareto o Nash)
  size_t SelectJoint(const std::vector<EqRecord> &cur_eqs,
                     const std::vector<AgentActions> &agents,
                     const std::vector<std::vector<int>> &alloc,
                     const std::vector<std::vector<double>> &costs,
                     size_t n_controlled,
                     const std::vector<size_t> &agent_robot_idx) const
  {
    size_t best_row = cur_eqs.front().row;
    double best_score = std::numeric_limits<double>::max();
    for (const auto &ce : cur_eqs)
    {
      double score = 0.0;
      for (size_t ai = 0; ai < n_controlled; ++ai)
      {
        score += costs[ce.row][ai];
        const ControlledRobot &rb = robots_[agent_robot_idx[ai]];
        if (rb.have_prev && !rb.escape)
        {
          double ax, ay, bx, by;
          EarlyDir(agents[ai].actions[alloc[ce.row][ai]].trajectory, ax, ay);
          EarlyDir(rb.prev_traj, bx, by);
          score += hysteresis_weight_ * std::hypot(ax - bx, ay - by);
        }
      }
      if (score < best_score)
      {
        best_score = score;
        best_row = ce.row;
      }
    }
    return best_row;
  }

  // ============================================================
  //  Loop principale
  // ============================================================
  void TimerCallback(const ros::TimerEvent &)
  {
    PlanCycle();
    timer_.stop();
    timer_.setPeriod(ros::Duration(1.0 / control_rate_), /*reset=*/true);
    timer_.start();
  }

  void PlanCycle()
  {
    // snapshot dei model_states
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      if (!have_model_states_)
        return;
      work_msg_ = latest_msg_;
    }

    // stati di tutti i robot controllati
    size_t n_with_state = 0;
    for (auto &r : robots_)
    {
      r.have_state = ExtractState(work_msg_, r.name, r.state);
      if (r.have_state)
        ++n_with_state;
    }
    if (n_with_state == 0)
    {
      ROS_WARN_THROTTLE(1.0, "[gt_planner] nessun robot trovato in model_states");
      PublishStop();
      return;
    }

    // Percezione
    BuildPerception();
    RefreshObstacles();

    // posizioni correnti dei pedoni rilevati (per escape-clearance e freno)
    std::vector<State> dyn_centers;
    for (const auto &d : dyn_obs_)
      dyn_centers.push_back(d.pos);

    if (enable_debug_)
    {
      std::string pl;
      char pb[96];
      for (size_t i = 0; i < dyn_obs_.size(); ++i)
      {
        std::snprintf(pb, sizeof(pb), "[ped%zu @(%.2f,%.2f) v=(%.2f,%.2f) |v|=%.2f] ",
                      i, dyn_obs_[i].pos.x, dyn_obs_[i].pos.y,
                      dyn_obs_[i].vx, dyn_obs_[i].vy,
                      std::hypot(dyn_obs_[i].vx, dyn_obs_[i].vy));
        pl += pb;
      }
      ROS_INFO_THROTTLE(0.5, "[gt_planner] pedoni rilevati=%zu tracce=%zu %s",
                        dyn_obs_.size(), tracks_.size(), pl.c_str());
    }

    // ---- per ogni robot: avanza goal + genera action set (context swap) ----
    std::vector<AgentActions> agents;
    std::vector<size_t> agent_robot_idx;

    for (size_t ri = 0; ri < robots_.size(); ++ri)
    {
      auto &r = robots_[ri];
      if (!r.have_state)
        continue;

      if (!r.final_reached &&
          std::hypot(r.goal_x - r.state.x, r.goal_y - r.state.y) < goal_tolerance_)
      {
        ROS_INFO("[gt_planner] [%s] goal raggiunto.", r.name.c_str());
        AdvanceGoal(r);
      }

      // carica il contesto di pianificazione del robot
      goal_x_ = r.goal_x;
      goal_y_ = r.goal_y;
      ego_x_ = r.state.x;
      ego_y_ = r.state.y;
      // escape 
      escape_mode_ = StateBlockedRaw(r.state.x, r.state.y) ||
                     (r.stuck_count > dyn_stuck_limit_);
      r.escape = escape_mode_;

      // altri agenti (altri robot + pedoni rilevati) per la clearance in escape
      other_agents_.clear();
      for (size_t rj = 0; rj < robots_.size(); ++rj)
        if (rj != ri && robots_[rj].have_state)
          other_agents_.push_back(robots_[rj].state);
      for (const auto &dc : dyn_centers)
        other_agents_.push_back(dc);
      prev_robot_traj_ = r.prev_traj;
      have_prev_robot_traj_ = r.have_prev;

      AgentActions ra;
      ra.name = r.name;
      ra.state = r.state;
      ra.is_robot = true;
      ra.actions = GenerateRobotActions(r.state);
      if (ra.actions.empty())
        continue;

      agents.push_back(ra);
      agent_robot_idx.push_back(ri);
    }

    if (agents.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[gt_planner] nessuna azione robot generata. Stop.");
      PublishStop();
      return;
    }
    // i primi n_controlled agenti sono i robot
    const size_t n_controlled = agents.size();

    // pedoni rilevati 
    for (size_t di = 0; di < dyn_obs_.size(); ++di)
    {
      AgentActions pa;
      pa.name = "ped_" + std::to_string(di);
      pa.state = dyn_obs_[di].pos;
      pa.is_robot = false;
      pa.actions = PredictDynObs(dyn_obs_[di]);
      agents.push_back(pa);
    }

    // ---- gioco congiunto: costi, Nash, Pareto ----
    auto alloc = EnumerateAllocations(agents);
    auto costs = BuildCostTable(agents, alloc);
    auto nash = FindNash(agents, alloc, costs);
    auto pareto = ParetoFilter(nash, costs);

    std::vector<EqRecord> cur_eqs;
    if (!pareto.empty())
      cur_eqs = BuildEqRecords(pareto, agents, alloc, costs);
    else if (!nash.empty())
      cur_eqs = BuildEqRecords(nash, agents, alloc, costs);

    size_t selected_row = 0;
    bool valid = false;
    if (!cur_eqs.empty())
    {
      selected_row = SelectJoint(cur_eqs, agents, alloc, costs, n_controlled, agent_robot_idx);
      valid = true;
    }

    // ---- dispatch comandi + memorizzazione per-robot ----
    if (valid)
    {
      for (size_t ai = 0; ai < n_controlled; ++ai)
      {
        auto &r = robots_[agent_robot_idx[ai]];
        const Action &act = agents[ai].actions[alloc[selected_row][ai]];

        // L'azione scelta si scontra se il suo costo nel gioco e' INF
	const bool collides = costs[selected_row][ai] >= 0.5 * kInf;

        bool stop_flag = false;
        if (!r.final_reached)
        {
          if (collides)
            stop_flag = true;  // nessun path evita la collisione -> STOP
          else if (safety_brake_ && SafetyBrake(act))
            stop_flag = true;  // rete di sicurezza statica (lampione/lidar davanti)
        }
        r.braked = stop_flag;

        // conteggio gridlock 
	if (collides)
          ++r.stuck_count;
        else if (!r.escape)
          r.stuck_count = 0;
        else if (r.stuck_count > 0)
          --r.stuck_count;

        // il piano viene consegnato al controller ad alta frequenza (ControlPublish)
        {
          std::lock_guard<std::mutex> lk(plan_mutex_);
          r.follow_path = act.trajectory;
          r.follow_stop = stop_flag || r.final_reached;
          r.have_plan = true;
        }

        r.prev_traj = act.trajectory;  // riferimento per l'isteresi del ciclo dopo
        r.have_prev = true;
      }
    }
    else
    {
      ROS_WARN_THROTTLE(0.5, "[gt_planner] nessun equilibrio Nash/Pareto. Stop.");
      std::lock_guard<std::mutex> lk(plan_mutex_);
      for (auto &r : robots_)
        r.follow_stop = true;   // il controller terra' fermi i robot
    }

    // ---- RViz + log ----
    PublishMarkers(agents, nash, pareto, alloc, selected_row, valid, n_controlled);

    if (enable_debug_)
    {
      std::string line;
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "[gt_planner] controllati=%zu alloc=%zu nash=%zu pareto=%zu valid=%d | ",
                    n_controlled, alloc.size(), nash.size(), pareto.size(), valid ? 1 : 0);
      line += buf;
      for (size_t ai = 0; ai < n_controlled; ++ai)
      {
        const auto &r = robots_[agent_robot_idx[ai]];
        const Action &act = valid ? agents[ai].actions[alloc[selected_row][ai]] : agents[ai].actions.back();
        // costo della scelta (INF = collisione con un pedone/robot inevitabile)
        double sc = valid ? costs[selected_row][ai] : -1.0;
        char costbuf[24];
        if (sc < 0) std::snprintf(costbuf, sizeof(costbuf), "n/a");
        else if (sc >= 0.5 * kInf) std::snprintf(costbuf, sizeof(costbuf), "INF");
        else std::snprintf(costbuf, sizeof(costbuf), "%.1f", sc);
        // closest approach al pedone piu' "pericoloso" (min dist nel tempo)
        double best_ca = 1e9, best_t = -1.0;
        for (size_t pj = n_controlled; pj < agents.size(); ++pj)
        {
          double d, t;
          ClosestApproach(act.trajectory, agents[pj].actions[0].trajectory, d, t);
          if (d < best_ca) { best_ca = d; best_t = t; }
        }
        char cabuf[40];
        if (best_t >= 0)
          std::snprintf(cabuf, sizeof(cabuf), " ca=%.2f@%.1fs", best_ca, best_t);
        else
          std::snprintf(cabuf, sizeof(cabuf), " ca=-");
        // closest approach all'ALTRO robot (per capire se l'INF e' robot-robot)
        double rr_ca = 1e9, rr_t = -1.0;
        for (size_t aj = 0; aj < n_controlled; ++aj)
        {
          if (aj == ai) continue;
          double d, t;
          ClosestApproach(act.trajectory, agents[aj].actions[alloc[selected_row][aj]].trajectory, d, t);
          if (d < rr_ca) { rr_ca = d; rr_t = t; }
        }
        char rrbuf[40];
        if (rr_t >= 0)
          std::snprintf(rrbuf, sizeof(rrbuf), " rr=%.2f@%.1fs", rr_ca, rr_t);
        else
          rrbuf[0] = '\0';
        std::snprintf(buf, sizeof(buf), "[%s sel=%s cmd=%.2f/%.2f cost=%s%s%s%s%s] ",
                      r.name.c_str(), act.name.c_str(),
                      act.v_cmd, act.w_cmd, costbuf, cabuf, rrbuf,
                      r.escape ? " ESC" : "", r.braked ? " BRAKE" : "");
        line += buf;
      }
      ROS_INFO_THROTTLE(0.5, "%s", line.c_str());
    }
  }

  // -------------------- membri ROS --------------------
  ros::NodeHandle nh_;
  ros::Subscriber model_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;
  ros::Timer control_timer_;
  double control_pub_rate_{15.0};
  double lookahead_{0.45};
  std::mutex state_mutex_;   // protegge latest_msg_/have_model_states_
  std::mutex plan_mutex_;    // protegge follow_path/follow_stop/have_plan
  gazebo_msgs::ModelStates work_msg_;  // snapshot usato dentro PlanCycle

  gazebo_msgs::ModelStates latest_msg_;
  bool have_model_states_{false};

  // -------------------- OMPL --------------------
  ob::StateSpacePtr space_;
  oc::ControlSpacePtr cspace_;
  oc::SimpleSetupPtr ss_;
  ompl::RNG rng_;

  // -------------------- robot controllati --------------------
  std::vector<ControlledRobot> robots_;

  // -------------------- parametri --------------------
  std::string turtlebot_name_;
  std::vector<std::string> actor_names_;
  std::vector<std::pair<std::string, std::string>> group_pairs_;
  double group_corridor_half_width_{0.70};
  std::vector<Obstacle> base_obstacles_;   // dal file (forma esatta)
  std::vector<Obstacle> obstacles_;        // live = file + rilevati a runtime
  bool obstacle_autodetect_{true};
  double obstacle_default_radius_{0.40};

  // layer lidar
  bool use_lidar_{true};
  std::string scan_topic_{"/scan"};
  double lidar_grid_res_{0.10};
  double lidar_inflation_{0.50};
  double lidar_actor_filter_radius_{0.60};
  double lidar_robot_filter_radius_{0.45};

  // tracker dinamico
  bool track_dynamic_{true};
  double cluster_cell_{0.18};
  int cluster_min_cells_{1};
  double track_gate_{0.55};
  int track_max_misses_{6};
  int track_min_hits_{3};
  double dyn_speed_thresh_{0.07};
  double vel_ema_{0.6};
  double pos_ema_{0.5};
  double vel_window_{0.5};
  double ped_max_speed_{0.6};
  double ped_predict_horizon_{7.0};
  int dyn_latch_frames_{8};
  std::vector<Track> tracks_;
  int next_track_id_{0};
  ros::Time last_perc_time_;
  bool have_perc_time_{false};
  std::vector<DynObs> dyn_obs_;       // pedoni rilevati questo ciclo
  double static_margin_{0.12};
  double ego_bubble_{0.35};
  double ego_x_{0.0};
  double ego_y_{0.0};
  bool escape_mode_{false};
  std::vector<State> other_agents_;   // altri agenti (per la clearance in escape)
  sensor_msgs::LaserScan latest_scan_;
  bool have_scan_{false};
  std::unordered_set<long> lidar_cells_;
  std::vector<State> lidar_points_;
  tf::TransformListener tf_listener_;
  double goal_block_penalty_{100.0};
  bool road_keep_{false};
  double road_min_x_{-1.4};
  double road_max_x_{1.4};
  double road_penalty_{6.0};
  double hysteresis_weight_{2.5};
  std::vector<State> prev_robot_traj_;
  bool have_prev_robot_traj_{false};

  std::vector<Goal> goals_;
  size_t current_goal_idx_{0};
  bool loop_goals_{true};
  bool final_goal_reached_{false};
  double goal_x_{2.0}, goal_y_{0.0};
  double goal_tolerance_{0.30};
  double dyn_collision_dist_{0.45};
  double ped_comfort_dist_{0.75};
  double ped_comfort_weight_{4.0};
  double robot_collision_dist_{0.55};
  int dyn_stuck_limit_{6};
  double w_ema_{0.6};
  double goal_region_x_{0.30}, goal_region_y_{0.50};

  double interaction_radius_{5.0};
  double agent_radius_{0.375};
  double collision_margin_{0.10};

  double replan_dt_{0.10};
  double integrator_dt_{0.05};
  double planning_horizon_{2.0};
  double control_rate_{10.0};
  int num_traj_steps_{40};

  double robot_v_{0.22};
  double w_min_{0.10}, w_max_{0.55};
  double curvature_factor_{0.5};
  double prop_min_lo_{0.35}, prop_min_hi_{0.65};
  double prop_max_lo_{0.75}, prop_max_hi_{1.25};
  int robot_num_actions_{16};
  double rrt_solve_time_{0.02};
  double ws_min_x_{-3.5}, ws_max_x_{3.5}, ws_min_y_{-3.5}, ws_max_y_{3.5};

  int actor_num_actions_{5};
  double actor_turn_rate_{0.25};
  double min_actor_speed_{0.05}, max_actor_speed_{1.2}, default_actor_speed_{0.55};

  bool safety_brake_{true};
  double brake_lookahead_{0.8};
  double brake_margin_{0.12};
  double brake_cone_{1.05};

  bool enable_debug_{true};
  bool publish_markers_{true};
  std::string marker_frame_{"odom"};

  // -------------------- stato --------------------
  std::map<std::string, State> previous_actor_states_;
  std::map<std::string, ros::Time> previous_actor_times_;
  std::map<std::vector<int>, size_t> allocation_to_row_;

  std::vector<EqRecord> prev_eqs_;            // E[t - Delta t]
  std::map<std::string, State> obs_prev_state_;  // per T^obs
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gt_planner_node");
  GameTheoryPlannerNode node;
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
