#include <ros/ros.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <vector>

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

struct Action
{
  std::vector<State> trajectory;
  double v_cmd{0.0};
  double w_cmd{0.0};
  double cost{0.0};
  std::string name;
};

struct AgentActions
{
  std::string name;
  State state;
  std::vector<Action> actions;
};

class GameTheoryPlannerNode
{
public:
  GameTheoryPlannerNode()
  {
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("turtlebot_name", turtlebot_name_, "turtlebot3_waffle_pi");

    if (!pnh.getParam("actor_names", actor_names_))
    {
      actor_names_ = {"actor", "actor_2", "actor_3_group_L", "actor_4_group_R"};
    }

    pnh.param<double>("goal_x", goal_x_, 2.0);
    pnh.param<double>("goal_y", goal_y_, 0.0);
    pnh.param<bool>("loop_goals", loop_goals_, true);

    LoadGoals(pnh);

    if (!goals_.empty())
    {
      goal_x_ = goals_[current_goal_idx_].x;
      goal_y_ = goals_[current_goal_idx_].y;
    }

    pnh.param<double>("interaction_radius", interaction_radius_, 5.0);
    pnh.param<double>("agent_radius", agent_radius_, 0.375);
    pnh.param<double>("collision_margin", collision_margin_, 0.15);

    pnh.param<double>("dt", dt_, 0.1);
    pnh.param<double>("planning_horizon", planning_horizon_, 2.0);
    pnh.param<double>("control_rate", control_rate_, 10.0);

    pnh.param<double>("robot_v", robot_v_, 0.16);
    pnh.param<double>("robot_w", robot_w_, 0.65);
    pnh.param<double>("actor_turn_rate", actor_turn_rate_, 0.25);

    pnh.param<double>("goal_weight", goal_weight_, 2.0);
    pnh.param<double>("heading_weight", heading_weight_, 0.2);
    pnh.param<double>("stop_cost", stop_cost_, 20.0);

    pnh.param<double>("goal_tolerance", goal_tolerance_, 0.25);
    pnh.param<double>("max_actor_speed", max_actor_speed_, 1.2);
    pnh.param<double>("min_actor_speed", min_actor_speed_, 0.05);

    pnh.param<bool>("enable_debug", enable_debug_, true);

    model_sub_ = nh_.subscribe("/gazebo/model_states", 1, &GameTheoryPlannerNode::ModelStatesCallback, this);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &GameTheoryPlannerNode::TimerCallback, this);

    ROS_INFO("[gt_planner_node] Started.");
    ROS_INFO("[gt_planner_node] TurtleBot model: %s", turtlebot_name_.c_str());
    ROS_INFO("[gt_planner_node] Goal: %.2f %.2f", goal_x_, goal_y_);
  }

private:
  static double WrapToPi(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  static double Clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  static double Dist2D(const State &a, const State &b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static double PathLength(const std::vector<State> &traj)
  {
    if (traj.size() < 2)
      return 0.0;

    double length = 0.0;

    for (size_t i = 1; i < traj.size(); ++i)
      length += std::hypot(traj[i].x - traj[i - 1].x, traj[i].y - traj[i - 1].y);

    return length;
  }

  static State StepUnicycle(const State &s, double v, double w, double dt)
  {
    State out;
    out.x = s.x + dt * v * std::cos(s.yaw);
    out.y = s.y + dt * v * std::sin(s.yaw);
    out.yaw = WrapToPi(s.yaw + dt * w);
    return out;
  }

  static double YawFromQuat(const geometry_msgs::Quaternion &q)
  {
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

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

  void LoadGoals(ros::NodeHandle &pnh)
  {
    XmlRpc::XmlRpcValue goal_list;

    if (!pnh.getParam("goals", goal_list))
    {
      goals_.push_back({goal_x_, goal_y_});
      return;
    }

    if (goal_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("[gt_planner_node] Parameter 'goals' is not a list. Using goal_x/goal_y.");
      goals_.push_back({goal_x_, goal_y_});
      return;
    }

    for (int i = 0; i < goal_list.size(); ++i)
    {
      if (goal_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray ||
          goal_list[i].size() < 2)
      {
        ROS_WARN("[gt_planner_node] Invalid goal entry at index %d. Skipping.", i);
        continue;
      }

      double gx = static_cast<double>(goal_list[i][0]);
      double gy = static_cast<double>(goal_list[i][1]);

      goals_.push_back({gx, gy});
    }

    if (goals_.empty())
    {
      ROS_WARN("[gt_planner_node] Goal list is empty. Using goal_x/goal_y.");
      goals_.push_back({goal_x_, goal_y_});
    }

    ROS_INFO("[gt_planner_node] Loaded %zu goals.", goals_.size());
  }

  void AdvanceGoal()
  {
    if (goals_.empty())
      return;

    if (current_goal_idx_ + 1 < goals_.size())
    {
      current_goal_idx_++;
    }
    else
    {
      if (loop_goals_)
        current_goal_idx_ = 0;
      else
        final_goal_reached_ = true;
    }

    goal_x_ = goals_[current_goal_idx_].x;
    goal_y_ = goals_[current_goal_idx_].y;

    ROS_INFO("[gt_planner_node] New goal %zu/%zu: %.2f %.2f",
             current_goal_idx_ + 1,
             goals_.size(),
             goal_x_,
             goal_y_);
  }

  State EstimateActorVelocityState(const std::string &name, const State &current)
  {
    State vel;

    ros::Time now = ros::Time::now();

    auto it_s = previous_actor_states_.find(name);
    auto it_t = previous_actor_times_.find(name);

    if (it_s == previous_actor_states_.end() || it_t == previous_actor_times_.end())
    {
      previous_actor_states_[name] = current;
      previous_actor_times_[name] = now;
      return vel;
    }

    double dt = (now - it_t->second).toSec();

    if (dt <= 1e-3 || dt > 1.0)
    {
      previous_actor_states_[name] = current;
      previous_actor_times_[name] = now;
      return vel;
    }

    vel.x = (current.x - it_s->second.x) / dt;
    vel.y = (current.y - it_s->second.y) / dt;
    vel.yaw = WrapToPi(current.yaw - it_s->second.yaw) / dt;

    previous_actor_states_[name] = current;
    previous_actor_times_[name] = now;

    return vel;
  }

  std::vector<Action> GenerateRobotActions(const State &robot)
  {
    std::vector<Action> actions;

    const std::vector<std::pair<double, double>> controls = {
      {robot_v_, 0.0},
      {robot_v_, robot_w_},
      {robot_v_, -robot_w_},
      {robot_v_, 0.5 * robot_w_},
      {robot_v_, -0.5 * robot_w_},
      {0.0, 0.0}
    };

    const std::vector<std::string> names = {
      "forward",
      "left",
      "right",
      "soft_left",
      "soft_right",
      "stop"
    };

    for (size_t i = 0; i < controls.size(); ++i)
    {
      double v = controls[i].first;
      double w = controls[i].second;

      Action a;
      a.v_cmd = v;
      a.w_cmd = w;
      a.name = names[i];

      State s = robot;
      a.trajectory.push_back(s);

      int steps = std::max(1, static_cast<int>(std::round(planning_horizon_ / dt_)));

      for (int k = 0; k < steps; ++k)
      {
        s = StepUnicycle(s, v, w, dt_);
        a.trajectory.push_back(s);
      }

      double length = PathLength(a.trajectory);
      State last = a.trajectory.back();

      double goal_dist = std::hypot(goal_x_ - last.x, goal_y_ - last.y);
      double desired_yaw = std::atan2(goal_y_ - robot.y, goal_x_ - robot.x);
      double heading_err = std::abs(WrapToPi(desired_yaw - robot.yaw));

      a.cost = length + goal_weight_ * goal_dist + heading_weight_ * heading_err;

      if (std::abs(v) < 1e-6 && std::abs(w) < 1e-6)
        a.cost += stop_cost_;

      actions.push_back(a);
    }

    return actions;
  }

  std::vector<Action> GenerateActorPredictionActions(const std::string &name, const State &actor)
  {
    State vel = EstimateActorVelocityState(name, actor);

    double speed = std::hypot(vel.x, vel.y);

    if (speed < min_actor_speed_)
      speed = 0.55;

    speed = Clamp(speed, min_actor_speed_, max_actor_speed_);

    double base_yaw = actor.yaw;

    if (std::hypot(vel.x, vel.y) > min_actor_speed_)
      base_yaw = std::atan2(vel.y, vel.x);

    const std::vector<double> turn_rates = {
      0.0,
      actor_turn_rate_,
      -actor_turn_rate_,
      0.5 * actor_turn_rate_,
      -0.5 * actor_turn_rate_
    };

    const std::vector<double> speed_scales = {
      1.0,
      1.0,
      1.0,
      0.75,
      0.75
    };

    const std::vector<std::string> names = {
      "actor_forward",
      "actor_left",
      "actor_right",
      "actor_soft_left",
      "actor_soft_right"
    };

    std::vector<Action> actions;

    for (size_t i = 0; i < turn_rates.size(); ++i)
    {
      Action a;
      a.v_cmd = speed * speed_scales[i];
      a.w_cmd = turn_rates[i];
      a.name = names[i];

      State s = actor;
      s.yaw = base_yaw;
      a.trajectory.push_back(s);

      int steps = std::max(1, static_cast<int>(std::round(planning_horizon_ / dt_)));

      for (int k = 0; k < steps; ++k)
      {
        s = StepUnicycle(s, a.v_cmd, a.w_cmd, dt_);
        a.trajectory.push_back(s);
      }

      a.cost = PathLength(a.trajectory);

      actions.push_back(a);
    }

    return actions;
  }

  bool TrajectoriesCollide(const std::vector<State> &a, const std::vector<State> &b) const
  {
    const size_t n = std::min(a.size(), b.size());
    const double min_dist = 2.0 * agent_radius_ + collision_margin_;

    for (size_t i = 0; i < n; ++i)
    {
      double d = std::hypot(a[i].x - b[i].x, a[i].y - b[i].y);

      if (d < min_dist)
        return true;
    }

    return false;
  }

  std::vector<std::vector<int>> EnumerateAllocations(const std::vector<AgentActions> &agents) const
  {
    std::vector<std::vector<int>> allocations;

    if (agents.empty())
      return allocations;

    std::vector<int> current(agents.size(), 0);

    while (true)
    {
      allocations.push_back(current);

      int idx = static_cast<int>(agents.size()) - 1;

      while (idx >= 0)
      {
        current[idx]++;

        if (current[idx] < static_cast<int>(agents[idx].actions.size()))
          break;

        current[idx] = 0;
        idx--;
      }

      if (idx < 0)
        break;
    }

    return allocations;
  }

  std::vector<std::vector<double>> BuildCostTable(
    const std::vector<AgentActions> &agents,
    const std::vector<std::vector<int>> &allocations) const
  {
    const double INF = 1e12;

    std::vector<std::vector<double>> costs(
      allocations.size(),
      std::vector<double>(agents.size(), 0.0));

    for (size_t r = 0; r < allocations.size(); ++r)
    {
      std::vector<bool> collided(agents.size(), false);

      for (size_t i = 0; i < agents.size(); ++i)
      {
        for (size_t j = i + 1; j < agents.size(); ++j)
        {
          const Action &ai = agents[i].actions[allocations[r][i]];
          const Action &aj = agents[j].actions[allocations[r][j]];

          if (TrajectoriesCollide(ai.trajectory, aj.trajectory))
          {
            collided[i] = true;
            collided[j] = true;
          }
        }
      }

      for (size_t i = 0; i < agents.size(); ++i)
      {
        if (collided[i])
          costs[r][i] = INF;
        else
          costs[r][i] = agents[i].actions[allocations[r][i]].cost;
      }
    }

    return costs;
  }

  bool IsNash(
    size_t row,
    const std::vector<AgentActions> &agents,
    const std::vector<std::vector<int>> &allocations,
    const std::vector<std::vector<double>> &costs) const
  {
    const double EPS = 1e-9;

    for (size_t agent_idx = 0; agent_idx < agents.size(); ++agent_idx)
    {
      double current_cost = costs[row][agent_idx];

      for (size_t alt = 0; alt < agents[agent_idx].actions.size(); ++alt)
      {
        if (static_cast<int>(alt) == allocations[row][agent_idx])
          continue;

        std::vector<int> alt_alloc = allocations[row];
        alt_alloc[agent_idx] = static_cast<int>(alt);

        auto it = allocation_to_row_.find(alt_alloc);

        if (it == allocation_to_row_.end())
          continue;

        double alt_cost = costs[it->second][agent_idx];

        if (alt_cost + EPS < current_cost)
          return false;
      }
    }

    return true;
  }

  std::vector<size_t> FindNashEquilibria(
    const std::vector<AgentActions> &agents,
    const std::vector<std::vector<int>> &allocations,
    const std::vector<std::vector<double>> &costs)
  {
    allocation_to_row_.clear();

    for (size_t i = 0; i < allocations.size(); ++i)
      allocation_to_row_[allocations[i]] = i;

    std::vector<size_t> nash_rows;

    for (size_t r = 0; r < allocations.size(); ++r)
    {
      if (IsNash(r, agents, allocations, costs))
        nash_rows.push_back(r);
    }

    return nash_rows;
  }

  std::vector<size_t> ParetoFilter(
    const std::vector<size_t> &rows,
    const std::vector<std::vector<double>> &costs) const
  {
    std::vector<size_t> out;

    for (size_t i = 0; i < rows.size(); ++i)
    {
      bool dominated = false;

      for (size_t j = 0; j < rows.size(); ++j)
      {
        if (i == j)
          continue;

        bool all_leq = true;
        bool one_lt = false;

        for (size_t a = 0; a < costs[rows[i]].size(); ++a)
        {
          if (costs[rows[j]][a] > costs[rows[i]][a])
            all_leq = false;

          if (costs[rows[j]][a] < costs[rows[i]][a])
            one_lt = true;
        }

        if (all_leq && one_lt)
        {
          dominated = true;
          break;
        }
      }

      if (!dominated)
        out.push_back(rows[i]);
    }

    return out;
  }

  size_t SelectEquilibriumForRobot(
    const std::vector<size_t> &candidate_rows,
    const std::vector<std::vector<double>> &costs) const
  {
    if (candidate_rows.empty())
      return 0;

    size_t best = candidate_rows.front();
    double best_robot_cost = costs[best][0];

    for (size_t r : candidate_rows)
    {
      if (costs[r][0] < best_robot_cost)
      {
        best_robot_cost = costs[r][0];
        best = r;
      }
    }

    return best;
  }

  void PublishStop()
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_.publish(cmd);
  }

  void TimerCallback(const ros::TimerEvent &)
  {
    if (!have_model_states_)
      return;

    State robot;

    if (!ExtractState(latest_msg_, turtlebot_name_, robot))
    {
      ROS_WARN_THROTTLE(1.0, "[gt_planner_node] TurtleBot model not found: %s", turtlebot_name_.c_str());
      PublishStop();
      return;
    }

    const double dist_to_goal = std::hypot(goal_x_ - robot.x, goal_y_ - robot.y);

    if (final_goal_reached_)
    {
      ROS_INFO_THROTTLE(1.0, "[gt_planner_node] Final goal reached.");
      PublishStop();
      return;
    }

    if (dist_to_goal < goal_tolerance_)
    {
      ROS_INFO("[gt_planner_node] Goal %zu reached at distance %.3f.",
               current_goal_idx_ + 1,
               dist_to_goal);

      AdvanceGoal();

      if (final_goal_reached_)
      {
        PublishStop();
        return;
      }
    }
    std::vector<AgentActions> agents;

    AgentActions robot_agent;
    robot_agent.name = turtlebot_name_;
    robot_agent.state = robot;
    robot_agent.actions = GenerateRobotActions(robot);
    agents.push_back(robot_agent);

    for (const std::string &actor_name : actor_names_)
    {
      State actor;

      if (!ExtractState(latest_msg_, actor_name, actor))
        continue;

      const double d = Dist2D(robot, actor);

      if (d > interaction_radius_)
        continue;

      AgentActions actor_agent;
      actor_agent.name = actor_name;
      actor_agent.state = actor;
      actor_agent.actions = GenerateActorPredictionActions(actor_name, actor);
      agents.push_back(actor_agent);
    }

    auto allocations = EnumerateAllocations(agents);
    auto costs = BuildCostTable(agents, allocations);

    auto nash = FindNashEquilibria(agents, allocations, costs);
    auto pareto = ParetoFilter(nash, costs);

    size_t selected_row = 0;
    bool valid_game_solution = false;

    if (!pareto.empty())
    {
      selected_row = SelectEquilibriumForRobot(pareto, costs);
      valid_game_solution = true;
    }
    else if (!nash.empty())
    {
      selected_row = SelectEquilibriumForRobot(nash, costs);
      valid_game_solution = true;
    }

    geometry_msgs::Twist cmd;

    if (valid_game_solution)
    {
      int robot_action_idx = allocations[selected_row][0];
      const Action &selected_action = agents[0].actions[robot_action_idx];

      cmd.linear.x = selected_action.v_cmd;
      cmd.angular.z = selected_action.w_cmd;

      if (enable_debug_)
      {
        ROS_INFO_THROTTLE(
          0.5,
          "[gt_planner_node] agents=%zu alloc=%zu nash=%zu pareto=%zu action=%s cmd=[%.2f %.2f] goal_dist=%.2f",
          agents.size(),
          allocations.size(),
          nash.size(),
          pareto.size(),
          selected_action.name.c_str(),
          cmd.linear.x,
          cmd.angular.z,
          dist_to_goal);
      }
    }
    else
    {
      ROS_WARN_THROTTLE(0.5, "[gt_planner_node] No Nash/Pareto solution found. Stopping.");
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    cmd_pub_.publish(cmd);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber model_sub_;
  ros::Publisher cmd_pub_;
  ros::Timer timer_;

  gazebo_msgs::ModelStates latest_msg_;
  bool have_model_states_{false};

  std::string turtlebot_name_;
  std::vector<std::string> actor_names_;

  double goal_x_{2.0};
  double goal_y_{0.0};

  std::vector<Goal> goals_;
  size_t current_goal_idx_{0};
  bool loop_goals_{true};
  bool final_goal_reached_{false};

  double interaction_radius_{5.0};
  double agent_radius_{0.375};
  double collision_margin_{0.15};

  double dt_{0.1};
  double planning_horizon_{2.0};
  double control_rate_{10.0};

  double robot_v_{0.16};
  double robot_w_{0.65};
  double actor_turn_rate_{0.25};

  double goal_weight_{2.0};
  double heading_weight_{0.2};
  double stop_cost_{20.0};

  double goal_tolerance_{0.25};

  double max_actor_speed_{1.2};
  double min_actor_speed_{0.05};

  bool enable_debug_{true};

  std::map<std::string, State> previous_actor_states_;
  std::map<std::string, ros::Time> previous_actor_times_;

  std::map<std::vector<int>, size_t> allocation_to_row_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gt_planner_node");

  GameTheoryPlannerNode node;

  ros::spin();

  return 0;
}
