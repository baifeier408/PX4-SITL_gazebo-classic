/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Anton Matosov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H

#include <string>
#include <random>
#include <fstream>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "Wind.pb.h"

namespace gazebo {
// Default values
static const std::string kDefaultNamespace = "";
static const std::string kDefaultFrameId = "world";

static constexpr double kDefaultWindVelocityMean = 0.0;
static constexpr double kDefaultWindVelocityMax = 100.0;
static constexpr double kDefaultWindVelocityVariance = 0.0;
static constexpr double kDefaultWindGustVelocityMean = 0.0;
static constexpr double kDefaultWindGustVelocityMax = 10.0;
static constexpr double kDefaultWindGustVelocityVariance = 0.0;

static constexpr double kDefaultWindGustStart = 10.0;
static constexpr double kDefaultWindGustDuration = 0.0;

static const ignition::math::Vector3d kDefaultWindDirectionMean = ignition::math::Vector3d(1, 0, 0);
static const ignition::math::Vector3d kDefaultWindGustDirectionMean = ignition::math::Vector3d(0, 1, 0);
static constexpr double kDefaultWindDirectionVariance = 0.0;
static constexpr double kDefaultWindGustDirectionVariance = 0.0;

/// \brief This gazebo plugin simulates wind acting on a model.
class GazeboWindPlugin : public WorldPlugin {
 public:
  GazeboWindPlugin()
      : WorldPlugin(),
        namespace_(kDefaultNamespace),
        wind_pub_topic_("world_wind"),
        wind_velocity_mean_(kDefaultWindVelocityMean),
        wind_velocity_max_(kDefaultWindVelocityMax),
        wind_velocity_variance_(kDefaultWindVelocityVariance),
        wind_gust_velocity_mean_(kDefaultWindGustVelocityMean),
        wind_gust_velocity_max_(kDefaultWindGustVelocityMax),
        wind_gust_velocity_variance_(kDefaultWindGustVelocityVariance),
        wind_direction_mean_(kDefaultWindDirectionMean),
        wind_direction_variance_(kDefaultWindDirectionVariance),
        wind_gust_direction_mean_(kDefaultWindGustDirectionMean),
        wind_gust_direction_variance_(kDefaultWindGustDirectionVariance),
        frame_id_(kDefaultFrameId),
        pub_interval_(0.5),
        node_handle_(NULL) {}

  virtual ~GazeboWindPlugin();

 protected:
  /// \brief Load the plugin.
  /// \param[in] _model Pointer to the model that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr world, sdf::ElementPtr sdf);

  /// \brief Called when the world is updated.
  /// \param[in] _info Update timing information.
  void OnUpdate(const common::UpdateInfo& /*_info*/);

  // Callback for runtime wind command.
  // The command is a wind velocity vector in Gazebo world frame, unit: m/s.
  void WindCmdCallback(ConstVector3dPtr& msg);

  // Callback for runtime wind mode command.
  // Temporary Vector3d format:
  //   x = mode, 0 step/hold, 1 ramp, 2 sine
  //   y = duration in seconds
  //   z = frequency in Hz for sine mode, reserved for other modes
  void WindModeCallback(ConstVector3dPtr& msg);

  // Callback for runtime sine wind amplitude.
  // Temporary Vector3d format:
  //   x = sine amplitude in wind_x, m/s
  //   y = sine amplitude in wind_y, m/s
  //   z = sine amplitude in wind_z, m/s
  void WindSineAmpCallback(ConstVector3dPtr& msg);

  // Callback for runtime random-step wind parameters.
  // Temporary Vector3d format:
  //   x = min random wind speed, m/s
  //   y = max random wind speed, m/s
  //   z = update period, s
  void WindRandomParamsCallback(ConstVector3dPtr& msg);

  // Callback for C2.5 three-axis random-step minimum wind vector.
  // Temporary Vector3d format:
  //   x = min wind_x, m/s
  //   y = min wind_y, m/s
  //   z = min wind_z, m/s
  void WindRandomMinCallback(ConstVector3dPtr& msg);

  // Callback for C2.5 three-axis random-step maximum wind vector.
  // Temporary Vector3d format:
  //   x = max wind_x, m/s
  //   y = max wind_y, m/s
  //   z = max wind_z, m/s
  void WindRandomMaxCallback(ConstVector3dPtr& msg);

 private:
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr update_connection_;

  physics::WorldPtr world_;

  std::string namespace_;

  std::string frame_id_;
  std::string wind_pub_topic_;

  double wind_velocity_mean_;
  double wind_velocity_max_;
  double wind_velocity_variance_;
  double wind_gust_velocity_mean_;
  double wind_gust_velocity_max_;
  double wind_gust_velocity_variance_;
  double pub_interval_;
  std::default_random_engine wind_velocity_generator_;
  std::normal_distribution<double> wind_velocity_distribution_;
  std::default_random_engine wind_gust_velocity_generator_;
  std::normal_distribution<double> wind_gust_velocity_distribution_;

  ignition::math::Vector3d wind_direction_mean_;
  ignition::math::Vector3d wind_gust_direction_mean_;
  double wind_direction_variance_;
  double wind_gust_direction_variance_;
  std::default_random_engine wind_direction_generator_;
  std::normal_distribution<double> wind_direction_distribution_X_;
  std::normal_distribution<double> wind_direction_distribution_Y_;
  std::normal_distribution<double> wind_direction_distribution_Z_;
  std::default_random_engine wind_gust_direction_generator_;
  std::normal_distribution<double> wind_gust_direction_distribution_X_;
  std::normal_distribution<double> wind_gust_direction_distribution_Y_;
  std::normal_distribution<double> wind_gust_direction_distribution_Z_;

  common::Time wind_gust_end_;
  common::Time wind_gust_start_;
  common::Time last_time_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr wind_pub_;

  // Runtime wind command subscriber.
  transport::SubscriberPtr wind_cmd_sub_;

  // C2.2 runtime wind mode subscriber.
  transport::SubscriberPtr wind_mode_sub_;

  // C2.3 runtime sine wind amplitude subscriber.
  transport::SubscriberPtr wind_sine_amp_sub_;

  // C2.4 runtime random-step wind parameter subscriber.
  transport::SubscriberPtr wind_random_params_sub_;

  // C2.5 runtime three-axis random-step wind min/max subscribers.
  transport::SubscriberPtr wind_random_min_sub_;
  transport::SubscriberPtr wind_random_max_sub_;

  // Latest runtime wind command in Gazebo world frame, unit: m/s.
  // Legacy C1 variable kept for compatibility with previous notes.
  ignition::math::Vector3d wind_cmd_;

  // C2.1 runtime wind command interpolation.
  // The external command sets a target wind. The plugin internally interpolates
  // from wind_cmd_current_ to wind_cmd_target_ using Gazebo simulation time.
  ignition::math::Vector3d wind_cmd_target_{0.0, 0.0, 0.0};
  ignition::math::Vector3d wind_cmd_current_{0.0, 0.0, 0.0};
  ignition::math::Vector3d wind_cmd_ramp_start_{0.0, 0.0, 0.0};
  double wind_cmd_ramp_start_time_{0.0};
  double wind_cmd_ramp_duration_{2.0};
  bool wind_cmd_ramp_active_{false};

  // C2.2/C2.3/C2.4/C2.5/C2.6 runtime wind mode.
  // 0 = step / hold
  // 1 = ramp
  // 2 = sine
  // 3 = random_step_fixed_direction
  // 4 = random_step_3axis
  // 5 = random_walk_3axis
  int wind_cmd_mode_{1};

  // C2.4 random-step wind parameters.
  ignition::math::Vector3d wind_random_direction_{0.0, 1.0, 0.0};
  double wind_random_speed_min_{0.0};
  double wind_random_speed_max_{5.0};
  double wind_random_update_period_{3.0};
  double wind_random_start_time_{0.0};
  double wind_random_next_update_time_{0.0};
  double wind_random_current_speed_{0.0};
  unsigned int wind_random_seed_{42};
  std::mt19937 wind_random_gen_{42};
  bool wind_random_active_{false};

  // C2.5 three-axis random-step wind ranges.
  // Used by mode=4 random_step_3axis.
  // Also used as clamp bounds by mode=5 random_walk_3axis.
  ignition::math::Vector3d wind_random_min_{-3.0, 2.0, -0.5};
  ignition::math::Vector3d wind_random_max_{ 3.0, 8.0,  0.5};

  // C2.6 three-axis random-walk wind parameters.
  // Used by mode=5 random_walk_3axis.
  double wind_random_walk_delta_xy_{0.4};
  double wind_random_walk_delta_z_{0.08};
  double wind_random_walk_duration_{30.0};
  ignition::math::Vector3d wind_random_walk_target_{0.0, 5.0, 0.0};

  // C2.3 sine wind parameters.
  ignition::math::Vector3d wind_sine_mean_{0.0, 0.0, 0.0};
  ignition::math::Vector3d wind_sine_amp_{0.0, 0.0, 0.0};
  double wind_sine_start_time_{0.0};
  double wind_sine_frequency_hz_{0.1};
  bool wind_sine_active_{false};

  // C2.1.1 actual published wind logging.
  // This records the final wind vector published by the plugin, regardless of
  // whether the wind is generated from SDF parameters, runtime wind_cmd, or
  // future mode-based wind generators.
  std::string wind_actual_log_path_{"/tmp/gazebo_wind_actual_log.csv"};
  std::ofstream wind_actual_log_;

  // If true, use runtime wind command instead of SDF/random/gust wind.
  bool use_external_wind_cmd_{false};

  physics_msgs::msgs::Wind wind_msg;
};
}

#endif // ROTORS_GAZEBO_PLUGINS_GAZEBO_WIND_PLUGIN_H
