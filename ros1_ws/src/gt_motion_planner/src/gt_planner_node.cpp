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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>

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

// Un'azione = una traiettoria (Sect. 3) con la sua serie di controlli.
struct Action
{
  std::vector<State> trajectory;  // stati ricampionati a passo integrator_dt
  double v_cmd{0.0};              // primo controllo lineare (Eq. 8)
  double w_cmd{0.0};              // primo controllo angolare
  double cost{0.0};               // Jhat = Length  (Eq. 9)
  bool group_blocked{false};      // true se attraversa il corridoio di un gruppo
  std::string name;
};

struct AgentActions
{
  std::string name;
  State state;
  bool is_robot{false};
  std::vector<Action> actions;
};

// Memoria di un equilibrio (per il ragionamento 4.3.4): per ciascun attore
// la traiettoria che quell'equilibrio assumeva.
struct EqRecord
{
  size_t row{0};
  double robot_cost{0.0};
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
    LoadGoals(pnh);

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

    model_sub_ = nh_.subscribe("/gazebo/model_states", 1,
                               &GameTheoryPlannerNode::ModelStatesCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("gt_planner/markers", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                             &GameTheoryPlannerNode::TimerCallback, this);

    ROS_INFO("[gt_planner] Avviato. robot=%s  M_n=%d  Delta_t=%.2f  R=%.3f",
             turtlebot_name_.c_str(), robot_num_actions_, replan_dt_, agent_radius_);
    ROS_INFO("[gt_planner] Goal iniziale: %.2f %.2f", goal_x_, goal_y_);
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

  // pad/troncamento a num_traj_steps_ stati (per confronto a indici di tempo)
  void NormalizeTrajectory(std::vector<State> &t) const
  {
    if (t.empty())
      return;
    if (static_cast<int>(t.size()) > num_traj_steps_ + 1)
      t.resize(num_traj_steps_ + 1);
    while (static_cast<int>(t.size()) < num_traj_steps_ + 1)
      t.push_back(t.back());
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

  void LoadGoals(ros::NodeHandle &pnh)
  {
    XmlRpc::XmlRpcValue list;
    if (pnh.getParam("goals", list) && list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < list.size(); ++i)
      {
        if (list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || list[i].size() < 2)
          continue;
        goals_.push_back({static_cast<double>(list[i][0]), static_cast<double>(list[i][1])});
      }
    }
    if (goals_.empty())
      goals_.push_back({2.0, 0.0});

    goal_x_ = goals_[0].x;
    goal_y_ = goals_[0].y;
    ROS_INFO("[gt_planner] Goal caricati: %zu", goals_.size());
  }

  void AdvanceGoal()
  {
    if (current_goal_idx_ + 1 < goals_.size())
      ++current_goal_idx_;
    else if (loop_goals_)
      current_goal_idx_ = 0;
    else
    {
      final_goal_reached_ = true;
      return;
    }
    goal_x_ = goals_[current_goal_idx_].x;
    goal_y_ = goals_[current_goal_idx_].y;
    ROS_INFO("[gt_planner] Nuovo goal %zu/%zu: %.2f %.2f",
             current_goal_idx_ + 1, goals_.size(), goal_x_, goal_y_);
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

    // nessun ostacolo statico nel mondo: lo spazio ammissibile e' il workspace
    // (gli agenti dinamici sono gestiti dal gioco, cfr. Sect. 3).
    ss_->setStateValidityChecker(
      [](const ob::State *) -> bool { return true; });

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
      const double traveled = PathLength(a.trajectory);
      const double remaining =
        std::hypot(goal_x_ - a.trajectory.back().x, goal_y_ - a.trajectory.back().y);
      a.cost = traveled + remaining;

      NormalizeTrajectory(a.trajectory);
      actions.push_back(a);
    }

    // azione memorizzata: epsilon* del passo precedente, ri-radicata sullo stato
    // attuale del robot  (Sect. 4.3.3 "il gioco ricorda la combinazione vincente").
    if (have_memorized_robot_)
    {
      Action a;
      a.name = "memorized";
      a.v_cmd = memorized_v_;
      a.w_cmd = memorized_w_;
      State s = robot;
      a.trajectory.push_back(s);
      for (int k = 0; k < num_traj_steps_; ++k)
      {
        s = StepUnicycle(s, memorized_v_, memorized_w_, integrator_dt_);
        a.trajectory.push_back(s);
      }
      a.cost = PathLength(a.trajectory) +
               std::hypot(goal_x_ - a.trajectory.back().x, goal_y_ - a.trajectory.back().y);
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

    // marca le azioni che attraversano un corridoio di gruppo
    MarkGroupBlocked(actions);

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
        if (collided[i] || (agents[i].is_robot && act.group_blocked))
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
      // a parita' di somiglianza, preferisci costo robot minore
      if (score < best_res - 1e-6 ||
          (std::abs(score - best_res) <= 1e-6 && ce.robot_cost < best_cost))
      {
        best_res = score;
        best_cost = ce.robot_cost;
        best_row = ce.row;
      }
    }
    return best_row;
  }

  size_t LowestRobotCostRow(const std::vector<EqRecord> &eqs) const
  {
    size_t best = eqs.front().row;
    double bc = eqs.front().robot_cost;
    for (const auto &e : eqs)
      if (e.robot_cost < bc)
      {
        bc = e.robot_cost;
        best = e.row;
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
                      bool valid)
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

    // tutte le traiettorie robot valutate (grigio sottile) + costo come testo
    for (const auto &a : agents[0].actions)
    {
      bool blocked = a.group_blocked;
      make_line(a.trajectory, 0.02,
                blocked ? 0.6 : 0.5, blocked ? 0.1 : 0.5, blocked ? 0.1 : 0.5,
                0.5, "robot_actions");
    }

    // predizioni attori (arancione sottile)
    for (size_t i = 1; i < agents.size(); ++i)
      for (const auto &a : agents[i].actions)
        make_line(a.trajectory, 0.02, 1.0, 0.55, 0.0, 0.45, "actor_pred");

    // equilibri di Nash (azzurro)
    for (size_t r : nash)
    {
      const auto &a = agents[0].actions[alloc[r][0]];
      make_line(a.trajectory, 0.035, 0.1, 0.6, 1.0, 0.7, "nash");
    }

    // equilibri Pareto (blu piu' marcato)
    for (size_t r : pareto)
    {
      const auto &a = agents[0].actions[alloc[r][0]];
      make_line(a.trajectory, 0.05, 0.0, 0.2, 1.0, 0.85, "pareto");
    }

    // epsilon* scelto (verde spesso)
    if (valid)
    {
      const auto &a = agents[0].actions[alloc[selected_row][0]];
      make_line(a.trajectory, 0.09, 0.0, 1.0, 0.0, 1.0, "chosen");
    }

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

    marker_pub_.publish(arr);
  }

  void PublishStop()
  {
    geometry_msgs::Twist c;
    cmd_pub_.publish(c);
  }

  // ============================================================
  //  Loop principale (ogni Delta t)
  // ============================================================
  void TimerCallback(const ros::TimerEvent &)
  {
    if (!have_model_states_)
      return;

    State robot;
    if (!ExtractState(latest_msg_, turtlebot_name_, robot))
    {
      ROS_WARN_THROTTLE(1.0, "[gt_planner] modello robot non trovato: %s", turtlebot_name_.c_str());
      PublishStop();
      return;
    }

    const double dist_goal = std::hypot(goal_x_ - robot.x, goal_y_ - robot.y);

    if (final_goal_reached_)
    {
      ROS_INFO_THROTTLE(2.0, "[gt_planner] goal finale raggiunto.");
      PublishStop();
      return;
    }
    if (dist_goal < goal_tolerance_)
    {
      ROS_INFO("[gt_planner] goal %zu raggiunto (d=%.3f).", current_goal_idx_ + 1, dist_goal);
      AdvanceGoal();
      if (final_goal_reached_)
      {
        PublishStop();
        return;
      }
    }

    // ---- 1. costruzione agenti + action set ----
    std::vector<AgentActions> agents;

    AgentActions r;
    r.name = turtlebot_name_;
    r.state = robot;
    r.is_robot = true;
    r.actions = GenerateRobotActions(robot);
    agents.push_back(r);

    for (const std::string &an : actor_names_)
    {
      State as;
      if (!ExtractState(latest_msg_, an, as))
        continue;
      if (Dist2D(robot, as) > interaction_radius_)
        continue;
      AgentActions aa;
      aa.name = an;
      aa.state = as;
      aa.actions = GenerateActorPrediction(an, as);
      agents.push_back(aa);
    }

    if (agents[0].actions.empty())
    {
      ROS_WARN_THROTTLE(1.0, "[gt_planner] nessuna traiettoria RRT generata. Stop.");
      PublishStop();
      return;
    }

    // ---- 2-4. gioco statico: costi, Nash, Pareto ----
    auto alloc = EnumerateAllocations(agents);
    auto costs = BuildCostTable(agents, alloc);
    auto nash = FindNash(agents, alloc, costs);
    auto pareto = ParetoFilter(nash, costs);

    // ---- 5. coordinamento tra umani: scelta di epsilon* ----
    std::vector<EqRecord> cur_eqs;
    if (!pareto.empty())
      cur_eqs = BuildEqRecords(pareto, agents, alloc, costs);
    else if (!nash.empty())
      cur_eqs = BuildEqRecords(nash, agents, alloc, costs);

    size_t selected_row = 0;
    bool valid = false;
    if (!cur_eqs.empty())
    {
      selected_row = SelectEquilibrium(cur_eqs, agents);
      valid = true;
    }

    // ---- 6. comando + memorizzazione ----
    geometry_msgs::Twist cmd;
    std::string chosen_name = "none";
    if (valid)
    {
      int ridx = alloc[selected_row][0];
      const Action &act = agents[0].actions[ridx];
      cmd.linear.x = act.v_cmd;
      cmd.angular.z = act.w_cmd;
      chosen_name = act.name;

      memorized_v_ = act.v_cmd;
      memorized_w_ = act.w_cmd;
      have_memorized_ = true;
      have_memorized_robot_ = true;
    }
    else
    {
      ROS_WARN_THROTTLE(0.5, "[gt_planner] nessun equilibrio Nash/Pareto. Stop.");
    }
    cmd_pub_.publish(cmd);

    // ---- memoria per 4.3.4 al prossimo tick ----
    prev_eqs_ = cur_eqs;
    obs_prev_state_.clear();
    for (size_t i = 1; i < agents.size(); ++i)
      obs_prev_state_[agents[i].name] = agents[i].state;

    // aggiorna stati attori per la stima di velocita' (una volta per tick)
    ros::Time now = ros::Time::now();
    for (size_t i = 1; i < agents.size(); ++i)
    {
      previous_actor_states_[agents[i].name] = agents[i].state;
      previous_actor_times_[agents[i].name] = now;
    }

    // ---- RViz + log costi ----
    PublishMarkers(agents, nash, pareto, alloc, selected_row, valid);

    if (enable_debug_)
    {
      ROS_INFO_THROTTLE(
        0.5,
        "[gt_planner] agenti=%zu azioni_robot=%zu alloc=%zu nash=%zu pareto=%zu "
        "scelto=%s cmd=[%.2f %.2f] d_goal=%.2f",
        agents.size(), agents[0].actions.size(), alloc.size(),
        nash.size(), pareto.size(), chosen_name.c_str(),
        cmd.linear.x, cmd.angular.z, dist_goal);

      // costo di ogni traiettoria robot valutata
      std::string line = "[gt_planner] costi azioni robot: ";
      for (size_t i = 0; i < agents[0].actions.size(); ++i)
      {
        const auto &a = agents[0].actions[i];
        char buf[96];
        std::snprintf(buf, sizeof(buf), "%s=%.2f%s ",
                      a.name.c_str(), a.cost, a.group_blocked ? "(GRP)" : "");
        line += buf;
      }
      ROS_INFO_THROTTLE(0.5, "%s", line.c_str());
    }
  }

  // -------------------- membri ROS --------------------
  ros::NodeHandle nh_;
  ros::Subscriber model_sub_;
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

  // -------------------- parametri --------------------
  std::string turtlebot_name_;
  std::vector<std::string> actor_names_;
  std::vector<std::pair<std::string, std::string>> group_pairs_;
  double group_corridor_half_width_{0.70};

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
