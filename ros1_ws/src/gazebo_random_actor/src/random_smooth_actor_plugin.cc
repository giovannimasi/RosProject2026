#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Actor.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Angle.hh>

#include <random>
#include <string>
#include <algorithm>
#include <cmath>

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
    // Force actor to use a custom trajectory controlled by this plugin.
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = "walking";
    this->trajectoryInfo->duration = 1.0;
    this->trajectoryInfo->startTime = 0.0;
    this->actor->SetCustomTrajectory(this->trajectoryInfo);

    this->world = this->actor->GetWorld();

    this->minX = this->GetParam<double>(sdf, "min_x", -8.0);
    this->maxX = this->GetParam<double>(sdf, "max_x",  8.0);
    this->minY = this->GetParam<double>(sdf, "min_y", -8.0);
    this->maxY = this->GetParam<double>(sdf, "max_y",  8.0);

    this->speed = this->GetParam<double>(sdf, "speed", 0.65);
    this->minSpeed = this->GetParam<double>(sdf, "min_speed", this->speed);
    this->maxSpeed = this->GetParam<double>(sdf, "max_speed", this->speed);

    this->maxTurnRate = this->GetParam<double>(sdf, "max_turn_rate", 0.45);
    this->targetTolerance = this->GetParam<double>(sdf, "target_tolerance", 0.7);
    this->animationFactor = this->GetParam<double>(sdf, "animation_factor", 4.5);

    this->lateralOffset = this->GetParam<double>(sdf, "lateral_offset", 0.0);

    this->useCenterPose = this->GetParam<bool>(sdf, "use_center_pose", false);
    this->centerX = this->GetParam<double>(sdf, "center_x", 0.0);
    this->centerY = this->GetParam<double>(sdf, "center_y", 0.0);
    this->centerYaw = this->GetParam<double>(sdf, "center_yaw", 0.0);

    this->rollOffset = this->GetParam<double>(sdf, "roll_offset", 0.0);
    this->pitchOffset = this->GetParam<double>(sdf, "pitch_offset", 0.0);
    this->yawOffset = this->GetParam<double>(sdf, "yaw_offset", 0.0);

    this->zOffset = this->GetParam<double>(sdf, "z_offset", 1.0);

    unsigned int seed = this->GetParam<unsigned int>(sdf, "seed", 1u);
    this->rng.seed(seed);

    this->distX = std::uniform_real_distribution<double>(this->minX, this->maxX);
    this->distY = std::uniform_real_distribution<double>(this->minY, this->maxY);
    this->distSpeed = std::uniform_real_distribution<double>(this->minSpeed, this->maxSpeed);

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
    this->ChooseNewTarget();

    this->lastUpdate = this->world->SimTime();

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RandomSmoothActorPlugin::OnUpdate, this));

    gzmsg << "[RandomSmoothActorPlugin] Loaded for actor ["
          << this->actor->GetName() << "]"
          << " speed=" << this->speed
          << " bounds=[" << this->minX << "," << this->maxX
          << "] x [" << this->minY << "," << this->maxY << "]"
          << " lateral_offset=" << this->lateralOffset
          << "\n";
  }

private:
  template<typename T>
  T GetParam(sdf::ElementPtr sdf, const std::string &name, const T &defaultValue)
  {
    if (sdf->HasElement(name))
      return sdf->Get<T>(name);
    return defaultValue;
  }

  void ChooseNewTarget()
  {
    this->targetX = this->distX(this->rng);
    this->targetY = this->distY(this->rng);
    this->currentSpeed = this->distSpeed(this->rng);
  }

  static double WrapToPi(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  static double Clamp(double value, double minValue, double maxValue)
  {
    return std::max(minValue, std::min(value, maxValue));
  }

  void OnUpdate()
  {
    common::Time now = this->world->SimTime();
    double dt = (now - this->lastUpdate).Double();

    if (dt <= 0.0)
      return;

    this->lastUpdate = now;

    double dx = this->targetX - this->x;
    double dy = this->targetY - this->y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < this->targetTolerance)
    {
      this->ChooseNewTarget();
      dx = this->targetX - this->x;
      dy = this->targetY - this->y;
      distance = std::sqrt(dx * dx + dy * dy);
    }

    double desiredYaw = std::atan2(dy, dx);
    double yawError = WrapToPi(desiredYaw - this->yaw);

    double maxStep = this->maxTurnRate * dt;
    double yawStep = Clamp(yawError, -maxStep, maxStep);

    this->yaw = WrapToPi(this->yaw + yawStep);

    double vScale = std::max(0.0, std::cos(yawError));
    double v = this->currentSpeed * vScale;

    this->x += v * std::cos(this->yaw) * dt;
    this->y += v * std::sin(this->yaw) * dt;

    this->x = Clamp(this->x, this->minX, this->maxX);
    this->y = Clamp(this->y, this->minY, this->maxY);

    double offsetX = -std::sin(this->yaw) * this->lateralOffset;
    double offsetY =  std::cos(this->yaw) * this->lateralOffset;

    ignition::math::Pose3d pose(
      this->x + offsetX,
      this->y + offsetY,
      this->zOffset,
      this->rollOffset,
      this->pitchOffset,
      WrapToPi(this->yaw + this->yawOffset));

    this->actor->SetWorldPose(pose, false, false);

    double distanceWalked = std::abs(v) * dt;
    this->animationTime += distanceWalked * this->animationFactor;
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

  double maxTurnRate{0.45};
  double targetTolerance{0.7};
  double animationFactor{4.5};

  double lateralOffset{0.0};

  bool useCenterPose{false};
  double centerX{0.0};
  double centerY{0.0};
  double centerYaw{0.0};

  double rollOffset{0.0};
  double pitchOffset{0.0};
  double yawOffset{0.0};

  double zOffset{1.0};

  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  double targetX{0.0};
  double targetY{0.0};

  std::mt19937 rng;
  std::uniform_real_distribution<double> distX;
  std::uniform_real_distribution<double> distY;
  std::uniform_real_distribution<double> distSpeed;
};

GZ_REGISTER_MODEL_PLUGIN(RandomSmoothActorPlugin)
}
