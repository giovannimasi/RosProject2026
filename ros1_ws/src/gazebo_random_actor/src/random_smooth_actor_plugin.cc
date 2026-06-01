#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>

#include <ignition/math/Pose3.hh>

#include <random>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>

namespace gazebo
{

class RandomSmoothActorPlugin : public ModelPlugin
{
public:
  RandomSmoothActorPlugin() = default;

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->model = model;
    this->actor = boost::dynamic_pointer_cast<physics::Actor>(model);

    if (!this->actor)
    {
      gzerr << "[RandomSmoothActorPlugin] Model [" << model->GetName()
            << "] is not an actor.\n";
      return;
    }

    // Force Gazebo actor to use a custom trajectory controlled by this plugin.
    // This avoids the actor being reset by Gazebo's internal script logic.
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = "walking";
    this->trajectoryInfo->duration = 1.0;
    this->trajectoryInfo->startTime = 0.0;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);

    this->world = this->actor->GetWorld();

    // Workspace bounds
    this->minX = this->GetParam<double>(sdf, "min_x", -8.0);
    this->maxX = this->GetParam<double>(sdf, "max_x",  8.0);
    this->minY = this->GetParam<double>(sdf, "min_y", -8.0);
    this->maxY = this->GetParam<double>(sdf, "max_y",  8.0);

    // Speed
    this->speed = this->GetParam<double>(sdf, "speed", 0.65);
    this->minSpeed = this->GetParam<double>(sdf, "min_speed", this->speed);
    this->maxSpeed = this->GetParam<double>(sdf, "max_speed", this->speed);

    // Turning and natural walking
    this->maxTurnRate = this->GetParam<double>(sdf, "max_turn_rate", 0.35);
    this->minStraightTime = this->GetParam<double>(sdf, "min_straight_time", 4.0);
    this->maxStraightTime = this->GetParam<double>(sdf, "max_straight_time", 9.0);
    this->minTurnAngle = this->GetParam<double>(sdf, "min_turn_angle", -1.2);
    this->maxTurnAngle = this->GetParam<double>(sdf, "max_turn_angle",  1.2);

    this->wallMargin = this->GetParam<double>(sdf, "wall_margin", 1.0);
    this->lookAheadDistance = this->GetParam<double>(sdf, "look_ahead_distance", 1.5);

    // Small heading oscillation while walking straight.
    this->headingNoiseAmplitude = this->GetParam<double>(sdf, "heading_noise_amplitude", 0.025);
    this->headingNoiseFrequency = this->GetParam<double>(sdf, "heading_noise_frequency", 0.5);

    // Actor avoidance
    this->avoidanceRadius = this->GetParam<double>(sdf, "avoidance_radius", 1.4);
    this->hardAvoidanceRadius = this->GetParam<double>(sdf, "hard_avoidance_radius", 0.65);
    this->avoidanceGain = this->GetParam<double>(sdf, "avoidance_gain", 1.0);

    this->ReadAvoidModels(sdf);

    // Animation
    this->animationFactor = this->GetParam<double>(sdf, "animation_factor", 4.5);

    // Group / lateral offset.
    // Same seed + same center pose + opposite lateral_offset gives a pair walking side by side.
    this->lateralOffset = this->GetParam<double>(sdf, "lateral_offset", 0.0);

    this->useCenterPose = this->GetParam<bool>(sdf, "use_center_pose", false);
    this->centerX = this->GetParam<double>(sdf, "center_x", 0.0);
    this->centerY = this->GetParam<double>(sdf, "center_y", 0.0);
    this->centerYaw = this->GetParam<double>(sdf, "center_yaw", 0.0);

    // Mesh orientation correction.
    this->rollOffset = this->GetParam<double>(sdf, "roll_offset", 1.5708);
    this->pitchOffset = this->GetParam<double>(sdf, "pitch_offset", 0.0);
    this->yawOffset = this->GetParam<double>(sdf, "yaw_offset", 1.5780);
    this->zOffset = this->GetParam<double>(sdf, "z_offset", 1.0);

    this->seedValue = this->GetParam<unsigned int>(sdf, "seed", 1u);
    this->rng.seed(this->seedValue);

    this->distSpeed = std::uniform_real_distribution<double>(this->minSpeed, this->maxSpeed);
    this->distStraightTime = std::uniform_real_distribution<double>(
      this->minStraightTime, this->maxStraightTime);
    this->distTurnAngle = std::uniform_real_distribution<double>(
      this->minTurnAngle, this->maxTurnAngle);

    ignition::math::Pose3d pose = this->actor->WorldPose();

    if (this->useCenterPose)
    {
      this->x = this->centerX;
      this->y = this->centerY;
      this->yaw = this->centerYaw;
    }
    else
    {
      this->x = pose.Pos().X();
      this->y = pose.Pos().Y();
      this->yaw = pose.Rot().Yaw();
    }

    this->currentSpeed = this->distSpeed(this->rng);
    this->desiredYaw = this->yaw;

    this->motionState = MotionState::WALK_STRAIGHT;
    this->stateTimer = 0.0;
    this->stateDuration = this->distStraightTime(this->rng);

    this->lastUpdate = this->world->SimTime();

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RandomSmoothActorPlugin::OnUpdate, this));

    gzmsg << "[RandomSmoothActorPlugin] Loaded for actor ["
          << this->actor->GetName() << "]"
          << " bounds=[" << this->minX << "," << this->maxX
          << "] x [" << this->minY << "," << this->maxY << "]"
          << " speed=[" << this->minSpeed << "," << this->maxSpeed << "]"
          << " lateral_offset=" << this->lateralOffset
          << " avoid_models=" << this->avoidModels.size()
          << "\n";
  }

private:
  enum class MotionState
  {
    WALK_STRAIGHT,
    TURNING
  };

  template<typename T>
  T GetParam(sdf::ElementPtr sdf, const std::string &name, const T &defaultValue)
  {
    if (sdf->HasElement(name))
      return sdf->Get<T>(name);
    return defaultValue;
  }

  void ReadAvoidModels(sdf::ElementPtr sdf)
  {
    this->avoidModels.clear();

    if (!sdf->HasElement("avoid_model"))
      return;

    sdf::ElementPtr elem = sdf->GetElement("avoid_model");

    while (elem)
    {
      std::string modelName = elem->Get<std::string>();

      if (!modelName.empty() && modelName != this->actor->GetName())
        this->avoidModels.push_back(modelName);

      elem = elem->GetNextElement("avoid_model");
    }
  }

  static double WrapToPi(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  static double Clamp(double value, double minValue, double maxValue)
  {
    return std::max(minValue, std::min(value, maxValue));
  }

  bool NearBoundary(double x0, double y0, double yaw0) const
  {
    const double futureX = x0 + std::cos(yaw0) * this->lookAheadDistance;
    const double futureY = y0 + std::sin(yaw0) * this->lookAheadDistance;

    return futureX < this->minX + this->wallMargin ||
           futureX > this->maxX - this->wallMargin ||
           futureY < this->minY + this->wallMargin ||
           futureY > this->maxY - this->wallMargin;
  }

  double YawToWorkspaceCenter() const
  {
    const double cx = 0.5 * (this->minX + this->maxX);
    const double cy = 0.5 * (this->minY + this->maxY);

    return std::atan2(cy - this->y, cx - this->x);
  }

  void StartNewTurn(double newDesiredYaw)
  {
    this->desiredYaw = WrapToPi(newDesiredYaw);
    this->motionState = MotionState::TURNING;
    this->stateTimer = 0.0;
  }

  void StartStraightWalk()
  {
    this->motionState = MotionState::WALK_STRAIGHT;
    this->stateTimer = 0.0;
    this->stateDuration = this->distStraightTime(this->rng);
    this->currentSpeed = this->distSpeed(this->rng);
    this->desiredYaw = this->yaw;
  }

  bool ComputeAvoidanceYaw(double &avoidYaw, double &speedScale)
  {
    double repX = 0.0;
    double repY = 0.0;
    double minDist = 1e9;

    for (const std::string &name : this->avoidModels)
    {
      physics::ModelPtr other = this->world->ModelByName(name);

      if (!other)
        continue;

      ignition::math::Pose3d otherPose = other->WorldPose();

      const double dx = this->x - otherPose.Pos().X();
      const double dy = this->y - otherPose.Pos().Y();
      const double d = std::sqrt(dx * dx + dy * dy);

      if (d < 1e-4)
        continue;

      minDist = std::min(minDist, d);

      if (d < this->avoidanceRadius)
      {
        const double weight = (this->avoidanceRadius - d) / this->avoidanceRadius;
        const double weight2 = weight * weight;

        repX += weight2 * dx / d;
        repY += weight2 * dy / d;
      }
    }

    const double repNorm = std::sqrt(repX * repX + repY * repY);

    speedScale = 1.0;

    if (minDist < this->hardAvoidanceRadius)
      speedScale = 0.35;
    else if (minDist < this->avoidanceRadius)
      speedScale = 0.70;

    if (repNorm < 1e-5)
      return false;

    const double nominalX = std::cos(this->desiredYaw);
    const double nominalY = std::sin(this->desiredYaw);

    const double combinedX = nominalX + this->avoidanceGain * repX;
    const double combinedY = nominalY + this->avoidanceGain * repY;

    avoidYaw = std::atan2(combinedY, combinedX);
    return true;
  }

  void OnUpdate()
  {
    common::Time now = this->world->SimTime();
    double dt = (now - this->lastUpdate).Double();

    if (dt <= 0.0)
      return;

    // Avoid pathological jumps after pause/reset.
    if (dt > 0.2)
      dt = 0.2;

    this->lastUpdate = now;
    this->stateTimer += dt;

    double speedScale = 1.0;
    double avoidYaw = this->desiredYaw;
    const bool hasAvoidance = this->ComputeAvoidanceYaw(avoidYaw, speedScale);

    const bool nearBoundary = this->NearBoundary(this->x, this->y, this->yaw);

    if (hasAvoidance)
    {
      this->StartNewTurn(avoidYaw);
    }
    else if (nearBoundary)
    {
      this->StartNewTurn(this->YawToWorkspaceCenter());
    }
    else if (this->motionState == MotionState::WALK_STRAIGHT)
    {
      if (this->stateTimer >= this->stateDuration)
      {
        const double turnAngle = this->distTurnAngle(this->rng);
        this->StartNewTurn(this->yaw + turnAngle);
      }
    }
    else if (this->motionState == MotionState::TURNING)
    {
      const double yawError = WrapToPi(this->desiredYaw - this->yaw);

      if (std::abs(yawError) < 0.05)
      {
        this->StartStraightWalk();
      }
    }

    double w = 0.0;

    if (this->motionState == MotionState::TURNING)
    {
      const double yawError = WrapToPi(this->desiredYaw - this->yaw);
      const double maxYawStep = this->maxTurnRate * dt;
      const double yawStep = Clamp(yawError, -maxYawStep, maxYawStep);
      w = yawStep / dt;
    }
    else
    {
      // Small smooth variation while walking mostly straight.
      w = this->headingNoiseAmplitude *
          std::sin(this->headingNoiseFrequency * now.Double() +
                   static_cast<double>(this->seedValue));
    }

    this->yaw = WrapToPi(this->yaw + w * dt);

    // This is the speed of the virtual center of the actor/group.
    const double centerSpeed = Clamp(this->currentSpeed * speedScale, 0.15, this->maxSpeed);

    // The center follows the path. For a group, both actors must keep the same center path.
    this->x += centerSpeed * std::cos(this->yaw) * dt;
    this->y += centerSpeed * std::sin(this->yaw) * dt;

    // Keep inside bounds and force a turn inward if a clamp happens.
    bool clamped = false;

    if (this->x < this->minX)
    {
      this->x = this->minX;
      clamped = true;
    }
    else if (this->x > this->maxX)
    {
      this->x = this->maxX;
      clamped = true;
    }

    if (this->y < this->minY)
    {
      this->y = this->minY;
      clamped = true;
    }
    else if (this->y > this->maxY)
    {
      this->y = this->maxY;
      clamped = true;
    }

    if (clamped)
      this->StartNewTurn(this->YawToWorkspaceCenter());

    // Lateral offset for side-by-side groups.
    const double offsetX = -std::sin(this->yaw) * this->lateralOffset;
    const double offsetY =  std::cos(this->yaw) * this->lateralOffset;

    ignition::math::Pose3d pose(
      this->x + offsetX,
      this->y + offsetY,
      this->zOffset,
      this->rollOffset,
      this->pitchOffset,
      WrapToPi(this->yaw + this->yawOffset));

    this->actor->SetWorldPose(pose, false, false);

    // If this actor is part of a group, the visible actor at lateral offset has
    // a slightly different arc-length speed during turns.
    const double visibleSpeed = Clamp(centerSpeed - w * this->lateralOffset, 0.05, this->maxSpeed + 0.35);

    this->animationTime += std::abs(visibleSpeed) * dt * this->animationFactor;
    this->actor->SetScriptTime(this->animationTime);
  }

private:
  physics::ModelPtr model;
  physics::ActorPtr actor;
  physics::WorldPtr world;
  event::ConnectionPtr updateConnection;

  physics::TrajectoryInfoPtr trajectoryInfo;
  common::Time lastUpdate;

  double animationTime{0.0};

  double minX{-8.0};
  double maxX{8.0};
  double minY{-8.0};
  double maxY{8.0};

  double speed{0.65};
  double minSpeed{0.65};
  double maxSpeed{0.65};
  double currentSpeed{0.65};

  double maxTurnRate{0.35};

  double minStraightTime{4.0};
  double maxStraightTime{9.0};
  double minTurnAngle{-1.2};
  double maxTurnAngle{1.2};

  double wallMargin{1.0};
  double lookAheadDistance{1.5};

  double headingNoiseAmplitude{0.025};
  double headingNoiseFrequency{0.5};

  double avoidanceRadius{1.4};
  double hardAvoidanceRadius{0.65};
  double avoidanceGain{1.0};

  double animationFactor{4.5};

  double lateralOffset{0.0};

  bool useCenterPose{false};
  double centerX{0.0};
  double centerY{0.0};
  double centerYaw{0.0};

  double rollOffset{1.5708};
  double pitchOffset{0.0};
  double yawOffset{1.5780};
  double zOffset{1.0};

  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double desiredYaw{0.0};

  MotionState motionState{MotionState::WALK_STRAIGHT};
  double stateTimer{0.0};
  double stateDuration{5.0};

  unsigned int seedValue{1u};

  std::mt19937 rng;
  std::uniform_real_distribution<double> distSpeed;
  std::uniform_real_distribution<double> distStraightTime;
  std::uniform_real_distribution<double> distTurnAngle;

  std::vector<std::string> avoidModels;
};

GZ_REGISTER_MODEL_PLUGIN(RandomSmoothActorPlugin)

}
