// ============================================================================
//  Game Theoretic Motion Planner  -  Motion Planning Among Humans (Sect. 4.3.4)
//
//  Implementazione fedele a:
//    Turnwald & Wollherr, "Human-Like Motion Planning Based on Game Theoretic
//    Decision Making", Int J Soc Robotics (2019) 11:151-170.
//
//  Architettura disaccoppiata (Fig. 4), eseguita ogni Delta t:
//    1. Generazione action set T_n con RRT control-based (OMPL)        [4.3.1]
//    2. Costruzione del gioco statico:  J_n = Jhat + Jtilde            [4.1-4.3.2]
//         - Jhat = Length(traiettoria)                                  (Eq. 9)
//         - Jtilde = INF se collisione, 0 altrimenti                    (Eq. 4)
//    3. Calcolo equilibri di Nash                                       (Def. 2)
//    4. Filtro di Pareto                                                [4.2]
//    5. Coordinamento "tra umani": selezione dell'equilibrio tramite
//       osservazione delle traiettorie reali degli attori              [4.3.4]
//    6. Memorizzazione di epsilon* per il loop successivo               [4.3.3]
//
//  Il robot (turtlebot) e' l'unico agente controllato. Gli attori si muovono
//  da soli: il planner ne PREDICE i movimenti. Le coppie in gruppo vengono
//  riconosciute e non si pianifica mai di passare in mezzo a loro.
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
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <random>
#include <string>
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

// Un'azione = una traiettoria (Sect. 3) con la sua serie di controlli.
struct Action
{
  std::vector<State> trajectory;  // stati ricampionati a passo integrator_dt
  double v_cmd{0.0};              // primo controllo lineare (Eq. 8)
  double w_cmd{0.0};              // primo controllo angolare
  double cost{0.0};               // Jhat = Length  (Eq. 9)
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

// Un robot CONTROLLATO dal planner (agente controllabile, paper §4.3.3).
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
  double mem_v{0.0};
  double mem_w{0.0};
  bool have_mem{false};
};

// Memoria di un equilibrio (per il ragionamento 4.3.4): per ciascun attore
// la traiettoria che quell'equilibrio assumeva.
struct EqRecord
{
  size_t row{0};
  double robot_cost{0.0};
  std::vector<State> robot_traj;
  std::map<std::string, std::vector<State>> actor_traj;
};

// ----------------------------------------------------------------------------
//  Control sampler discreto (set di controlli Eq. 8)
//    U_n = { (v,0), (v,w), (v,-w), (v,cw), (v,-cw) }
//  con v fisso (specifico dell'agente), w estratto a caso in [w_min,w_max]
//  prima di pianificare la traiettoria, c fattore di curvatura.
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
    pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh.param<double>("ego_bubble", ego_bubble_, 0.35);

    pnh.param<double>("goal_block_penalty", goal_block_penalty_, 100.0);
    pnh.param<double>("hysteresis_weight", hysteresis_weight_, 2.5);

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

    pnh.param<bool>("enable_debug", enable_debug_, true);
    pnh.param<bool>("publish_markers", publish_markers_, true);
    pnh.param<std::string>("marker_frame", marker_frame_, "odom");

    num_traj_steps_ = std::max(1, static_cast<int>(std::round(planning_horizon_ / integrator_dt_)));

    SetupOmpl();

    SetupRobots(pnh);

    model_sub_ = nh_.subscribe("/gazebo/model_states", 1,
                               &GameTheoryPlannerNode::ModelStatesCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("gt_planner/markers", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                             &GameTheoryPlannerNode::TimerCallback, this);

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
  // {name, cmd_topic, scan_topic, goals, loop_goals}). In assenza, modalita'
  // singolo robot retro-compatibile dai parametri legacy.
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

  // Ricostruisce la lista live degli ostacoli: quelli del file (forma esatta)
  // + quelli rilevati a runtime da /gazebo/model_states (cerchi, raggio
  // default). Qualunque model che non sia robot/attore/ground/sun e che non
  // inizi per "obs_" (gia' nel file) viene trattato come ostacolo statico.
  void RefreshObstacles()
  {
    obstacles_ = base_obstacles_;
    if (!obstacle_autodetect_)
      return;

    for (size_t i = 0; i < latest_msg_.name.size(); ++i)
    {
      const std::string &nm = latest_msg_.name[i];
      if (nm == "ground_plane" || nm == "sun" || IsControlledRobotName(nm))
        continue;
      if (std::find(actor_names_.begin(), actor_names_.end(), nm) != actor_names_.end())
        continue;
      if (nm.rfind("obs_", 0) == 0)  // gia' descritto con forma esatta nel file
        continue;

      Obstacle o;
      o.is_box = false;
      o.x = latest_msg_.pose[i].position.x;
      o.y = latest_msg_.pose[i].position.y;
      o.r = obstacle_default_radius_;
      obstacles_.push_back(o);
    }
  }

  // uno stato e' bloccato se dentro un oggetto modellato OPPURE vicino a un
  // punto lidar (layer aggiuntivo che cattura la geometria reale, es. spigoli).
  // blocco "crudo": dentro oggetto o vicino a punto lidar (senza ego-bubble)
  bool StateBlockedRaw(double x, double y) const
  {
    return InsideObstacle(x, y) || IsLidarOccupied(x, y);
  }

  // distanza dal punto (x,y) all'ostacolo piu' vicino (lidar + oggetti).
  // Usata in escape-mode per allontanarsi (massimizzare clearance).
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
    // include anche gli altri agenti (robot/attori): in escape la fuga deve
    // allontanarsi da TUTTO, non puntare verso l'altro robot (-> deadlock).
    for (const auto &a : other_agents_)
      m = std::min(m, std::hypot(x - a.x, y - a.y));
    return m;
  }

  bool StateBlocked(double x, double y) const
  {
    // ego-bubble CONDIZIONALE: si attiva SOLO se il robot e' gia' dentro una
    // zona bloccata (intrappolato, es. ostacolo comparso sopra di lui). In quel
    // caso si scava un disco attorno alla posa attuale per garantire start RRT
    // valido e permettere la fuga.
    // In condizioni normali (robot libero) la bolla NON e' attiva: cosi' i path
    // verso l'ostacolo restano invalidi e il robot mantiene lo standoff invece
    // di poterci entrare (la bolla sempre-attiva scavava un buco nell'ostacolo
    // e ci faceva entrare).
    if (escape_mode_ && std::hypot(x - ego_x_, y - ego_y_) < ego_bubble_)
      return false;
    return StateBlockedRaw(x, y);
  }

  // true se una traiettoria entra in un ostacolo (oggetto o punto lidar)
  bool TrajectoryHitsObstacle(const std::vector<State> &t) const
  {
    for (const auto &p : t)
      if (StateBlocked(p.x, p.y))
        return true;
    return false;
  }

  // true se il segmento (a)->(b) attraversa un ostacolo. Usato per capire se la
  // stima Euclidea della distanza residua al goal e' valida (LoS libera) o
  // ottimistica (passa dentro un ostacolo).
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

  // costo = lunghezza percorsa + stima del completamento fino al goal.
  // Se la retta end->goal e' ostruita, la stima Euclidea e' ottimistica:
  // si penalizza, altrimenti i path corti puntati sull'ostacolo sembrano
  // economici e vengono scelti (-> bot dritto contro l'ostacolo).
  double TrajectoryCost(const std::vector<State> &t) const
  {
    const double traveled = PathLength(t);
    const State &end = t.back();
    double remaining = std::hypot(goal_x_ - end.x, goal_y_ - end.y);
    if (SegmentBlocked(end.x, end.y, goal_x_, goal_y_))
      remaining += goal_block_penalty_;
    return traveled + remaining;
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

  // Ricostruisce la griglia di occupazione dai punti /scan (trasformati in
  // frame mondo), gonfiati di lidar_inflation. Scarta i punti vicini agli
  // attori: quelli restano agenti dinamici gestiti dal gioco, non ostacoli
  // statici. Salva anche i punti grezzi per la visualizzazione.
  // Griglia di occupazione statica CONDIVISA, costruita unendo gli /scan di
  // TUTTI i robot. Ogni punto e' portato in frame mondo tramite la posa mondo
  // del robot (da model_states) -> niente TF. I punti su agenti dinamici
  // (robot + attori) vengono scartati: restano gestiti dal gioco.
  void BuildLidarGrid(const std::vector<State> &dynamic_centers)
  {
    lidar_cells_.clear();
    lidar_points_.clear();
    if (!use_lidar_)
      return;

    const int rc = std::max(0, static_cast<int>(std::ceil(lidar_inflation_ / lidar_grid_res_)));
    const double r2 = lidar_inflation_ * lidar_inflation_;
    const long nx = static_cast<long>((ws_max_x_ - ws_min_x_) / lidar_grid_res_) + 1;

    for (const auto &rb : robots_)
    {
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
        const double lx = r * std::cos(a);
        const double ly = r * std::sin(a);
        // punto del raggio in frame mondo (base_scan ~ base_link, offset ~6cm
        // trascurabile vs risoluzione griglia)
        const double px = rx + lx * cyaw - ly * syaw;
        const double py = ry + lx * syaw + ly * cyaw;

        // scarta punti su agenti dinamici (altri robot + attori)
        bool on_dyn = false;
        for (const auto &dc : dynamic_centers)
          if (std::hypot(px - dc.x, py - dc.y) < lidar_actor_filter_radius_)
          {
            on_dyn = true;
            break;
          }
        if (on_dyn)
          continue;

        lidar_points_.push_back({px, py, 0.0});

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
    }
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
    r.have_prev = false;  // nuovo goal: ridecide il lato senza isteresi
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

    // propagatore unicycle (Eq. 7) integrato a passo integrator_dt
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
    // (gli agenti dinamici sono gestiti dal gioco, cfr. Sect. 3).
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
      // ("no valid initial states").
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

      // Jhat = Length  (Eq. 9). Con orizzonte finito le traiettorie non
      // raggiungono il goal, quindi si stima la lunghezza TOTALE del percorso
      // fino alla regione di goal: tratto percorso + distanza residua (stima
      // ammissibile del completamento in linea retta). Coincide con la
      // lunghezza del paper quando la traiettoria arriva effettivamente al goal.
      // Full-to-goal (paper §4.3.1): si tiene il path RRT COMPLETO (non
      // troncato). Costo = lunghezza piena + distanza residua al goal: se la
      // soluzione raggiunge la goal region remaining~0 e Jhat = Length (Eq.9
      // esatta); se e' approssimata il termine residuo mantiene la direzione.
      // La collisione dinamica con gli attori resta limitata alla prima
      // finestra ~planning_horizon perche' le traiettorie predette degli
      // attori sono lunghe solo num_traj_steps_ (confronto ad indici di tempo).
      a.cost = TrajectoryCost(a.trajectory);

      actions.push_back(a);
    }

    // Azioni fallback deterministiche (set di controlli Eq.8), aggiunte SOLO se
    // l'RRT non ha prodotto nessuna traiettoria: garantiscono movimento (evita
    // n=2/stallo) senza competere coi path full-to-goal in condizioni normali
    // (altrimenti gli archi corti, col costo ~lineare, vincerebbero sempre).
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

    // azione memorizzata (Sect. 4.3.3 "il gioco ricorda la combinazione
    // vincente"): si rioffre la TRAIETTORIA epsilon* scelta al tick precedente
    // (path full-to-goal gia' valido), NON un replay del comando (v,w) che
    // produrrebbe spirali o, se il precedente era stop, un path fermo a costo
    // minimo -> congelamento. Saltata se il precedente non si muoveva.
    if (have_memorized_robot_ && std::abs(memorized_v_) > 1e-3 && prev_robot_traj_.size() > 1)
    {
      Action a;
      a.name = "memorized";
      a.v_cmd = memorized_v_;
      a.w_cmd = memorized_w_;
      a.trajectory = prev_robot_traj_;
      a.cost = TrajectoryCost(a.trajectory);
      actions.push_back(a);
    }

    // azione "stand-still" tau^0 (Sect. 4.3.3): costo maggiore di ogni
    // traiettoria ma molto minore del costo di collisione.
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

    // ESCAPE MODE: il robot e' dentro una zona bloccata -> obiettivo non e'
    // raggiungere il goal ma ALLONTANARSI dall'ostacolo. Costo = -clearance
    // dell'endpoint (piu' lontano = meglio). Cosi' non punta verso l'ostacolo
    // e non resta fermo (stand_still ha clearance bassa -> costo alto).
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
      // costo predizione: lunghezza + piccolo prior di rettilineita' (un umano
      // tende a proseguire dritto; e' una prior di predizione, non il costo di
      // gioco del robot che resta puramente Eq.9).
      a.cost = PathLength(a.trajectory) + 0.5 * std::abs(turns[i]);
      actions.push_back(a);
    }
    return actions;
  }

  // ============================================================
  //  Vincoli: collisione (Eq. 2/4) e corridoio di gruppo
  // ============================================================
  bool TrajectoriesCollide(const std::vector<State> &a, const std::vector<State> &b) const
  {
    const size_t n = std::min(a.size(), b.size());
    const double dmin = 2.0 * agent_radius_ + collision_margin_;
    for (size_t i = 0; i < n; ++i)
      if (std::hypot(a[i].x - b[i].x, a[i].y - b[i].y) < dmin)
        return true;
    return false;
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
    return ExtractState(latest_msg_, name, out);
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

      // J_tilde: collisione tra coppie di agenti (Eq. 2/4)
      for (size_t i = 0; i < agents.size(); ++i)
        for (size_t j = i + 1; j < agents.size(); ++j)
          if (TrajectoriesCollide(agents[i].actions[alloc[r][i]].trajectory,
                                  agents[j].actions[alloc[r][j]].trajectory))
          {
            collided[i] = true;
            collided[j] = true;
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
  //  Coordinamento tra umani (Sect. 4.3.4)
  //    - costruisce i record degli equilibri correnti
  //    - sceglie epsilon* confrontando con le traiettorie OSSERVATE
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
  // (allineamento all'origine: confronta la "forma"/heading, robusto all'avanzamento)
  static double DirDistance(const std::vector<State> &a, const std::vector<State> &b)
  {
    double ax, ay, bx, by;
    TrajDir(a, ax, ay);
    TrajDir(b, bx, by);
    return std::hypot(ax - bx, ay - by);
  }

  // sceglie l'equilibrio epsilon* (ritorna la riga della tabella costi)
  size_t SelectEquilibrium(const std::vector<EqRecord> &cur_eqs,
                           const std::vector<AgentActions> &agents)
  {
    if (cur_eqs.empty())
      return 0;

    // direzioni osservate degli attori nell'ultimo Delta t  (T^obs)
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

    // fallback (t=0 oppure nessuna storia/osservazione utile):
    // equilibrio Pareto-ottimo a costo robot minimo
    if (prev_eqs_.empty() || obs.empty())
      return LowestRobotCostRow(cur_eqs);

    // (b) tra gli equilibri PRECEDENTI, quello piu' simile a cio' che gli
    //     attori hanno realmente fatto (T^obs)
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

    // (c) tra gli equilibri CORRENTI, quello piu' somigliante a prev_best
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
      // + isteresi: preferisci il lato gia' scelto (evita lo zig-zag)
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

  // penalita' di isteresi: scoraggia il cambio di "lato"/omotopia rispetto alla
  // traiettoria scelta al tick precedente. Senza, l'RRT casuale fa scegliere a
  // ogni iterazione il lato piu' corto del momento -> cammino a zig-zag che non
  // aggira l'ostacolo da nessun lato.
  // direzione INIZIALE (~1s) della traiettoria: distingue il "lato" (sx/dx)
  // della manovra di aggiramento, che il versore primo->ultimo NON cattura
  // (entrambi i lati finiscono verso il goal -> stessa direzione globale).
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
    latest_msg_ = *msg;
    have_model_states_ = true;
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

  void PublishStop()
  {
    geometry_msgs::Twist c;
    for (auto &r : robots_)
      r.cmd_pub.publish(c);
  }

  // selezione congiunta dell'equilibrio (paper §4.3.3, tutti controllabili):
  // tra gli equilibri candidati (Pareto o Nash) sceglie quello che minimizza la
  // somma dei costi dei robot controllati + penalita' di isteresi per-robot.
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
  //  Loop principale (ogni Delta t)
  // ============================================================
  void TimerCallback(const ros::TimerEvent &)
  {
    if (!have_model_states_)
      return;

    // stati di tutti i robot controllati + centri dinamici (per filtrare lidar)
    std::vector<State> dyn_centers;
    size_t n_with_state = 0;
    for (auto &r : robots_)
    {
      r.have_state = ExtractState(latest_msg_, r.name, r.state);
      if (r.have_state)
      {
        dyn_centers.push_back(r.state);
        ++n_with_state;
      }
    }
    if (n_with_state == 0)
    {
      ROS_WARN_THROTTLE(1.0, "[gt_planner] nessun robot trovato in model_states");
      PublishStop();
      return;
    }

    std::vector<State> actor_states;
    for (const std::string &an : actor_names_)
    {
      State as;
      if (ExtractState(latest_msg_, an, as))
      {
        actor_states.push_back(as);
        dyn_centers.push_back(as);
      }
    }

    // mappa statica condivisa: lidar unito (filtrando robot+attori) + oggetti
    BuildLidarGrid(dyn_centers);
    RefreshObstacles();

    // ---- 1. per ogni robot: avanza goal + genera action set (context swap) ----
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

      // carica il contesto di pianificazione del robot nei membri usati da
      // GenerateRobotActions / StateBlocked / cost
      goal_x_ = r.goal_x;
      goal_y_ = r.goal_y;
      ego_x_ = r.state.x;
      ego_y_ = r.state.y;
      escape_mode_ = StateBlockedRaw(r.state.x, r.state.y);
      r.escape = escape_mode_;

      // altri agenti (altri robot + attori) per la clearance in escape
      other_agents_.clear();
      for (size_t rj = 0; rj < robots_.size(); ++rj)
        if (rj != ri && robots_[rj].have_state)
          other_agents_.push_back(robots_[rj].state);
      for (const auto &as : actor_states)
        other_agents_.push_back(as);
      prev_robot_traj_ = r.prev_traj;
      have_prev_robot_traj_ = r.have_prev;
      memorized_v_ = r.mem_v;
      memorized_w_ = r.mem_w;
      have_memorized_robot_ = r.have_mem;

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
    const size_t n_controlled = agents.size();

    // ---- 2-4. gioco congiunto: costi, Nash, Pareto ----
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

    // ---- 6. dispatch comandi + memorizzazione per-robot ----
    if (valid)
    {
      for (size_t ai = 0; ai < n_controlled; ++ai)
      {
        auto &r = robots_[agent_robot_idx[ai]];
        const Action &act = agents[ai].actions[alloc[selected_row][ai]];

        geometry_msgs::Twist cmd;
        if (!r.final_reached)
        {
          cmd.linear.x = act.v_cmd;
          cmd.angular.z = act.w_cmd;
        }
        r.cmd_pub.publish(cmd);

        r.mem_v = act.v_cmd;
        r.mem_w = act.w_cmd;
        r.have_mem = true;
        r.prev_traj = act.trajectory;
        r.have_prev = true;
      }
    }
    else
    {
      ROS_WARN_THROTTLE(0.5, "[gt_planner] nessun equilibrio Nash/Pareto. Stop.");
      PublishStop();
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
        std::snprintf(buf, sizeof(buf), "[%s n=%zu sel=%s cmd=%.2f/%.2f%s] ",
                      r.name.c_str(), agents[ai].actions.size(), act.name.c_str(),
                      act.v_cmd, act.w_cmd, r.escape ? " ESC" : "");
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
  double hysteresis_weight_{2.5};
  std::vector<State> prev_robot_traj_;
  bool have_prev_robot_traj_{false};

  std::vector<Goal> goals_;
  size_t current_goal_idx_{0};
  bool loop_goals_{true};
  bool final_goal_reached_{false};
  double goal_x_{2.0}, goal_y_{0.0};
  double goal_tolerance_{0.30};
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

  bool enable_debug_{true};
  bool publish_markers_{true};
  std::string marker_frame_{"odom"};

  // -------------------- stato --------------------
  std::map<std::string, State> previous_actor_states_;
  std::map<std::string, ros::Time> previous_actor_times_;
  std::map<std::vector<int>, size_t> allocation_to_row_;

  std::vector<EqRecord> prev_eqs_;            // E[t - Delta t]
  std::map<std::string, State> obs_prev_state_;  // per T^obs

  bool have_memorized_{false};
  bool have_memorized_robot_{false};
  double memorized_v_{0.0}, memorized_w_{0.0};
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gt_planner_node");
  GameTheoryPlannerNode node;
  ros::spin();
  return 0;
}
