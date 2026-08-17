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

#include "gazebo_wind_plugin.h"
#include "common.h"

namespace gazebo {

GazeboWindPlugin::~GazeboWindPlugin() {
  if (wind_actual_log_.is_open()) {
    wind_actual_log_.close();
  }
  update_connection_->~Connection();
}

void GazeboWindPlugin::Load(physics::WorldPtr world, sdf::ElementPtr sdf) {
  world_ = world;

  double wind_gust_start = kDefaultWindGustStart;
  double wind_gust_duration = kDefaultWindGustDuration;

  if (sdf->HasElement("robotNamespace")) {
    namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
  } else {
    gzerr << "[gazebo_wind_plugin] Please specify a robotNamespace.\n";
  }

  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);


  getSdfParam<std::string>(sdf, "windPubTopic", wind_pub_topic_, wind_pub_topic_);
  double pub_rate = 50.0;  // C2.1: higher default wind publish rate for smoother runtime wind interpolation
  getSdfParam<double>(sdf, "publishRate", pub_rate, pub_rate); //Wind topic publishing rates
  pub_interval_ = (pub_rate > 0.0) ? 1/pub_rate : 0.0;
  getSdfParam<std::string>(sdf, "frameId", frame_id_, frame_id_);
  // Get the wind params from SDF.
  getSdfParam<double>(sdf, "windVelocityMean", wind_velocity_mean_, wind_velocity_mean_);
  getSdfParam<double>(sdf, "windVelocityMax", wind_velocity_max_, wind_velocity_max_);
  getSdfParam<double>(sdf, "windVelocityVariance", wind_velocity_variance_, wind_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windDirectionMean", wind_direction_mean_, wind_direction_mean_);
  getSdfParam<double>(sdf, "windDirectionVariance", wind_direction_variance_, wind_direction_variance_);
  // Get the wind gust params from SDF.
  getSdfParam<double>(sdf, "windGustStart", wind_gust_start, wind_gust_start);
  getSdfParam<double>(sdf, "windGustDuration", wind_gust_duration, wind_gust_duration);
  getSdfParam<double>(sdf, "windGustVelocityMean", wind_gust_velocity_mean_, wind_gust_velocity_mean_);
  getSdfParam<double>(sdf, "windGustVelocityMax", wind_gust_velocity_max_, wind_gust_velocity_max_);
  getSdfParam<double>(sdf, "windGustVelocityVariance", wind_gust_velocity_variance_, wind_gust_velocity_variance_);
  getSdfParam<ignition::math::Vector3d>(sdf, "windGustDirectionMean", wind_gust_direction_mean_, wind_gust_direction_mean_);
  getSdfParam<double>(sdf, "windGustDirectionVariance", wind_gust_direction_variance_, wind_gust_direction_variance_);

  wind_direction_mean_.Normalize();
  wind_gust_direction_mean_.Normalize();
  wind_gust_start_ = common::Time(wind_gust_start);
  wind_gust_end_ = common::Time(wind_gust_start + wind_gust_duration);
  // Set random wind velocity mean and standard deviation
  wind_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_velocity_mean_, sqrt(wind_velocity_variance_)));
  // Set random wind direction mean and standard deviation
  wind_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.X(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Y(), sqrt(wind_direction_variance_)));
  wind_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_direction_mean_.Z(), sqrt(wind_direction_variance_)));
  // Set random wind gust velocity mean and standard deviation
  wind_gust_velocity_distribution_.param(std::normal_distribution<double>::param_type(wind_gust_velocity_mean_, sqrt(wind_gust_velocity_variance_)));
  // Set random wind gust direction mean and standard deviation
  wind_gust_direction_distribution_X_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.X(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Y_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Y(), sqrt(wind_gust_direction_variance_)));
  wind_gust_direction_distribution_Z_.param(std::normal_distribution<double>::param_type(wind_gust_direction_mean_.Z(), sqrt(wind_gust_direction_variance_)));

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboWindPlugin::OnUpdate, this, _1));

  wind_pub_ = node_handle_->Advertise<physics_msgs::msgs::Wind>("~/" + wind_pub_topic_, 10);

  // Subscribe to runtime wind command.
  // Topic name is scoped under the Gazebo namespace, usually:
  //   /gazebo/default/wind_cmd
  // Message type:
  //   gazebo::msgs::Vector3d
  // Unit:
  //   m/s in Gazebo world frame
  wind_cmd_sub_ = node_handle_->Subscribe("~/wind_cmd", &GazeboWindPlugin::WindCmdCallback, this);

  // C2.2: runtime wind mode command.
  // Topic name is usually:
  //   /gazebo/default/wind_mode
  // Temporary message type:
  //   gazebo::msgs::Vector3d
  // Format:
  //   x = mode, 0 step/hold, 1 ramp
  //   y = duration in seconds
  //   z = reserved
  wind_mode_sub_ = node_handle_->Subscribe("~/wind_mode", &GazeboWindPlugin::WindModeCallback, this);

  // C2.3: runtime sine wind amplitude command.
  // Topic name is usually:
  //   /gazebo/default/wind_sine_amp
  // Temporary message type:
  //   gazebo::msgs::Vector3d
  // Format:
  //   x = amplitude in wind_x, m/s
  //   y = amplitude in wind_y, m/s
  //   z = amplitude in wind_z, m/s
  wind_sine_amp_sub_ = node_handle_->Subscribe("~/wind_sine_amp", &GazeboWindPlugin::WindSineAmpCallback, this);

  // C2.4: runtime random-step wind parameters.
  // Topic name is usually:
  //   /gazebo/default/wind_random_params
  // Temporary message type:
  //   gazebo::msgs::Vector3d
  // Format:
  //   x = min random wind speed, m/s
  //   y = max random wind speed, m/s
  //   z = update period, s
  wind_random_params_sub_ = node_handle_->Subscribe("~/wind_random_params", &GazeboWindPlugin::WindRandomParamsCallback, this);

  // C2.5: runtime three-axis random-step wind min/max.
  // Topic names are usually:
  //   /gazebo/default/wind_random_min
  //   /gazebo/default/wind_random_max
  // Temporary message type:
  //   gazebo::msgs::Vector3d
  wind_random_min_sub_ = node_handle_->Subscribe("~/wind_random_min", &GazeboWindPlugin::WindRandomMinCallback, this);
  wind_random_max_sub_ = node_handle_->Subscribe("~/wind_random_max", &GazeboWindPlugin::WindRandomMaxCallback, this);

  // C2.1.1: actual published wind log.
  // This file is truncated each time the plugin is loaded.
  getSdfParam<std::string>(sdf, "actualWindLogPath", wind_actual_log_path_, wind_actual_log_path_);
  wind_actual_log_.open(wind_actual_log_path_.c_str(), std::ios::out | std::ios::trunc);

  if (wind_actual_log_.is_open()) {
    wind_actual_log_ << "sim_time_s,wind_x_mps,wind_y_mps,wind_z_mps,"
                     << "use_external_wind_cmd,ramp_active,ramp_alpha,"
                     << "wind_cmd_mode,ramp_duration_s,"
                     << "sine_active,sine_frequency_hz,"
                     << "sine_amp_x_mps,sine_amp_y_mps,sine_amp_z_mps,"
                     << "random_active,random_speed_min_mps,random_speed_max_mps,"
                     << "random_update_period_s,random_seed,random_current_speed_mps,"
                     << "random_dir_x,random_dir_y,random_dir_z,"
                     << "random_min_x_mps,random_min_y_mps,random_min_z_mps,"
                     << "random_max_x_mps,random_max_y_mps,random_max_z_mps,"
                     << "random_walk_delta_xy_mps,random_walk_delta_z_mps,"
                     << "random_walk_target_x_mps,random_walk_target_y_mps,random_walk_target_z_mps,"
                     << "target_wind_x_mps,target_wind_y_mps,target_wind_z_mps\n";
    wind_actual_log_.flush();
    gzmsg << "[gazebo_wind_plugin] Actual published wind log: "
          << wind_actual_log_path_ << "\n";
  } else {
    gzerr << "[gazebo_wind_plugin] Failed to open actual wind log: "
          << wind_actual_log_path_ << "\n";
  }

#if GAZEBO_MAJOR_VERSION >= 9
  last_time_ = world_->SimTime();
#else
  last_time_ = world_->GetSimTime();
#endif

}

// This gets called by the world update start event.
void GazeboWindPlugin::OnUpdate(const common::UpdateInfo& _info) {
  // Get the current simulation time.
#if GAZEBO_MAJOR_VERSION >= 9
  common::Time now = world_->SimTime();
#else
  common::Time now = world_->GetSimTime();
#endif
  if ((now - last_time_).Double() < pub_interval_ || pub_interval_ == 0.0) {
    return;
  }
  last_time_ = now;

  // Calculate the wind force.
  // Get normal distribution wind strength
  double wind_strength = std::abs(wind_velocity_distribution_(wind_velocity_generator_));
  wind_strength = (wind_strength > wind_velocity_max_) ? wind_velocity_max_ : wind_strength;
  // Get normal distribution wind direction
  ignition::math::Vector3d wind_direction;
  wind_direction.X() = wind_direction_distribution_X_(wind_direction_generator_);
  wind_direction.Y() = wind_direction_distribution_Y_(wind_direction_generator_);
  wind_direction.Z() = wind_direction_distribution_Z_(wind_direction_generator_);
  // Calculate total wind velocity
  ignition::math::Vector3d wind = wind_strength * wind_direction;

  ignition::math::Vector3d wind_gust(0, 0, 0);
  // Calculate the wind gust velocity.
  if (now >= wind_gust_start_ && now < wind_gust_end_) {
    // Get normal distribution wind gust strength
    double wind_gust_strength = std::abs(wind_gust_velocity_distribution_(wind_gust_velocity_generator_));
    wind_gust_strength = (wind_gust_strength > wind_gust_velocity_max_) ? wind_gust_velocity_max_ : wind_gust_strength;
    // Get normal distribution wind gust direction
    ignition::math::Vector3d wind_gust_direction;
    wind_gust_direction.X() = wind_gust_direction_distribution_X_(wind_gust_direction_generator_);
    wind_gust_direction.Y() = wind_gust_direction_distribution_Y_(wind_gust_direction_generator_);
    wind_gust_direction.Z() = wind_gust_direction_distribution_Z_(wind_gust_direction_generator_);
    wind_gust = wind_gust_strength * wind_gust_direction;
  }

  ignition::math::Vector3d final_wind(0.0, 0.0, 0.0);
  double wind_cmd_ramp_alpha_for_log = -1.0;

  // If an external runtime wind command has been received, override the
  // SDF/random/gust wind with the commanded wind velocity.
  if (use_external_wind_cmd_) {
    const double now_sec = now.Double();

    if (wind_cmd_mode_ == 2 && wind_sine_active_) {
      const double elapsed = now_sec - wind_sine_start_time_;
      const double omega = 2.0 * M_PI * wind_sine_frequency_hz_;
      const double s = sin(omega * elapsed);

      wind_cmd_current_.X() = wind_sine_mean_.X() + wind_sine_amp_.X() * s;
      wind_cmd_current_.Y() = wind_sine_mean_.Y() + wind_sine_amp_.Y() * s;
      wind_cmd_current_.Z() = wind_sine_mean_.Z() + wind_sine_amp_.Z() * s;

      wind_cmd_ramp_alpha_for_log = -1.0;

      if (wind_cmd_ramp_duration_ > 0.0 && elapsed >= wind_cmd_ramp_duration_) {
        // After finite-duration sine mode ends, hold the mean wind.
        wind_cmd_current_ = wind_sine_mean_;
        wind_sine_active_ = false;
      }
    } else if (wind_cmd_mode_ == 3 && wind_random_active_) {
      const double elapsed = now_sec - wind_random_start_time_;

      if (wind_cmd_ramp_duration_ > 0.0 && elapsed >= wind_cmd_ramp_duration_) {
        // After finite-duration random-step mode ends, hold the last sampled wind.
        wind_random_active_ = false;
      } else if (now_sec >= wind_random_next_update_time_) {
        double min_speed = wind_random_speed_min_;
        double max_speed = wind_random_speed_max_;

        if (max_speed < min_speed) {
          const double tmp = min_speed;
          min_speed = max_speed;
          max_speed = tmp;
        }

        std::uniform_real_distribution<double> dist(min_speed, max_speed);
        wind_random_current_speed_ = dist(wind_random_gen_);
        wind_cmd_current_ = wind_random_direction_ * wind_random_current_speed_;

        double period = wind_random_update_period_;
        if (period < 1e-3) {
          period = 1e-3;
        }

        wind_random_next_update_time_ = now_sec + period;
      }

      wind_cmd_ramp_alpha_for_log = -1.0;
    } else if (wind_cmd_mode_ == 4 && wind_random_active_) {
      const double elapsed = now_sec - wind_random_start_time_;

      if (wind_cmd_ramp_duration_ > 0.0 && elapsed >= wind_cmd_ramp_duration_) {
        // After finite-duration three-axis random-step mode ends,
        // hold the last sampled wind vector.
        wind_random_active_ = false;
      } else if (now_sec >= wind_random_next_update_time_) {
        double min_x = wind_random_min_.X();
        double min_y = wind_random_min_.Y();
        double min_z = wind_random_min_.Z();
        double max_x = wind_random_max_.X();
        double max_y = wind_random_max_.Y();
        double max_z = wind_random_max_.Z();

        if (max_x < min_x) {
          const double tmp = min_x;
          min_x = max_x;
          max_x = tmp;
        }
        if (max_y < min_y) {
          const double tmp = min_y;
          min_y = max_y;
          max_y = tmp;
        }
        if (max_z < min_z) {
          const double tmp = min_z;
          min_z = max_z;
          max_z = tmp;
        }

        std::uniform_real_distribution<double> dist_x(min_x, max_x);
        std::uniform_real_distribution<double> dist_y(min_y, max_y);
        std::uniform_real_distribution<double> dist_z(min_z, max_z);

        wind_cmd_current_.X() = dist_x(wind_random_gen_);
        wind_cmd_current_.Y() = dist_y(wind_random_gen_);
        wind_cmd_current_.Z() = dist_z(wind_random_gen_);
        wind_random_current_speed_ = wind_cmd_current_.Length();

        double period = wind_random_update_period_;
        if (period < 1e-3) {
          period = 1e-3;
        }

        wind_random_next_update_time_ = now_sec + period;
      }

      wind_cmd_ramp_alpha_for_log = -1.0;
    } else if (wind_cmd_mode_ == 5 && wind_random_active_) {
      const double elapsed = now_sec - wind_random_start_time_;

      if (wind_random_walk_duration_ > 0.0 && elapsed >= wind_random_walk_duration_) {
        wind_random_active_ = false;
        wind_cmd_ramp_active_ = false;
        wind_cmd_current_ = wind_random_walk_target_;
      } else {
        double period = wind_random_update_period_;
        if (period < 1e-3) {
          period = 1e-3;
        }

        if (now_sec >= wind_random_next_update_time_) {
          std::uniform_real_distribution<double> dist_xy(-wind_random_walk_delta_xy_,
                                                          wind_random_walk_delta_xy_);
          std::uniform_real_distribution<double> dist_z(-wind_random_walk_delta_z_,
                                                         wind_random_walk_delta_z_);

          ignition::math::Vector3d next_target = wind_random_walk_target_;
          next_target.X() += dist_xy(wind_random_gen_);
          next_target.Y() += dist_xy(wind_random_gen_);
          next_target.Z() += dist_z(wind_random_gen_);

          if (next_target.X() < wind_random_min_.X()) {
            next_target.X() = wind_random_min_.X();
          }
          if (next_target.X() > wind_random_max_.X()) {
            next_target.X() = wind_random_max_.X();
          }
          if (next_target.Y() < wind_random_min_.Y()) {
            next_target.Y() = wind_random_min_.Y();
          }
          if (next_target.Y() > wind_random_max_.Y()) {
            next_target.Y() = wind_random_max_.Y();
          }
          if (next_target.Z() < wind_random_min_.Z()) {
            next_target.Z() = wind_random_min_.Z();
          }
          if (next_target.Z() > wind_random_max_.Z()) {
            next_target.Z() = wind_random_max_.Z();
          }

          wind_random_walk_target_ = next_target;
          wind_cmd_target_ = next_target;
          wind_cmd_ramp_start_ = wind_cmd_current_;
          wind_cmd_ramp_start_time_ = now_sec;
          wind_cmd_ramp_duration_ = period;
          wind_cmd_ramp_active_ = true;
          wind_random_next_update_time_ = now_sec + period;
        }

        if (wind_cmd_ramp_active_) {
          double alpha = 1.0;

          if (wind_cmd_ramp_duration_ > 1e-6) {
            alpha = (now_sec - wind_cmd_ramp_start_time_) / wind_cmd_ramp_duration_;
          }

          if (alpha >= 1.0) {
            alpha = 1.0;
            wind_cmd_ramp_active_ = false;
          } else if (alpha < 0.0) {
            alpha = 0.0;
          }

          wind_cmd_current_ = wind_cmd_ramp_start_
                              + alpha * (wind_cmd_target_ - wind_cmd_ramp_start_);
          wind_random_current_speed_ = wind_cmd_current_.Length();
          wind_cmd_ramp_alpha_for_log = alpha;
        } else {
          wind_cmd_current_ = wind_random_walk_target_;
          wind_random_current_speed_ = wind_cmd_current_.Length();
          wind_cmd_ramp_alpha_for_log = -1.0;
        }
      }
    } else if (wind_cmd_ramp_active_) {
      double alpha = 1.0;

      if (wind_cmd_ramp_duration_ > 1e-6) {
        alpha = (now_sec - wind_cmd_ramp_start_time_) / wind_cmd_ramp_duration_;
      }

      if (alpha >= 1.0) {
        alpha = 1.0;
        wind_cmd_ramp_active_ = false;
      } else if (alpha < 0.0) {
        alpha = 0.0;
      }

      wind_cmd_ramp_alpha_for_log = alpha;

      wind_cmd_current_ = wind_cmd_ramp_start_
                          + alpha * (wind_cmd_target_ - wind_cmd_ramp_start_);
    }

    final_wind = wind_cmd_current_;
  }

  gazebo::msgs::Vector3d* wind_v = new gazebo::msgs::Vector3d();
  wind_v->set_x(final_wind.X());
  wind_v->set_y(final_wind.Y());
  wind_v->set_z(final_wind.Z());

  wind_msg.set_frame_id(frame_id_);
  wind_msg.set_time_usec(now.Double() * 1e6);
  wind_msg.set_allocated_velocity(wind_v);

  // C2.1.1: log the actual final wind published by the plugin.
  if (wind_actual_log_.is_open()) {
    wind_actual_log_ << now.Double() << ","
                     << final_wind.X() << ","
                     << final_wind.Y() << ","
                     << final_wind.Z() << ","
                     << static_cast<int>(use_external_wind_cmd_) << ","
                     << static_cast<int>(wind_cmd_ramp_active_) << ","
                     << wind_cmd_ramp_alpha_for_log << ","
                     << wind_cmd_mode_ << ","
                     << wind_cmd_ramp_duration_ << ","
                     << static_cast<int>(wind_sine_active_) << ","
                     << wind_sine_frequency_hz_ << ","
                     << wind_sine_amp_.X() << ","
                     << wind_sine_amp_.Y() << ","
                     << wind_sine_amp_.Z() << ","
                     << static_cast<int>(wind_random_active_) << ","
                     << wind_random_speed_min_ << ","
                     << wind_random_speed_max_ << ","
                     << wind_random_update_period_ << ","
                     << wind_random_seed_ << ","
                     << wind_random_current_speed_ << ","
                     << wind_random_direction_.X() << ","
                     << wind_random_direction_.Y() << ","
                     << wind_random_direction_.Z() << ","
                     << wind_random_min_.X() << ","
                     << wind_random_min_.Y() << ","
                     << wind_random_min_.Z() << ","
                     << wind_random_max_.X() << ","
                     << wind_random_max_.Y() << ","
                     << wind_random_max_.Z() << ","
                     << wind_random_walk_delta_xy_ << ","
                     << wind_random_walk_delta_z_ << ","
                     << wind_random_walk_target_.X() << ","
                     << wind_random_walk_target_.Y() << ","
                     << wind_random_walk_target_.Z() << ","
                     << wind_cmd_target_.X() << ","
                     << wind_cmd_target_.Y() << ","
                     << wind_cmd_target_.Z() << "\n";
    wind_actual_log_.flush();
  }

  wind_pub_->Publish(wind_msg);
}

void GazeboWindPlugin::WindCmdCallback(ConstVector3dPtr& msg) {
#if GAZEBO_MAJOR_VERSION >= 9
  const double now_sec = world_->SimTime().Double();
#else
  const double now_sec = world_->GetSimTime().Double();
#endif

  wind_cmd_target_.X() = msg->x();
  wind_cmd_target_.Y() = msg->y();
  wind_cmd_target_.Z() = msg->z();

  use_external_wind_cmd_ = true;

  if (wind_cmd_mode_ == 0) {
    // C2.2 mode 0: step / hold.
    wind_sine_active_ = false;
    wind_random_active_ = false;
    wind_cmd_current_ = wind_cmd_target_;
    wind_cmd_ramp_start_ = wind_cmd_target_;
    wind_cmd_ramp_start_time_ = now_sec;
    wind_cmd_ramp_active_ = false;

    gzmsg << "[gazebo_wind_plugin] Received runtime wind target step: "
          << wind_cmd_target_.X() << ", "
          << wind_cmd_target_.Y() << ", "
          << wind_cmd_target_.Z() << " m/s\n";
  } else if (wind_cmd_mode_ == 2) {
    // C2.3 mode 2: sine wind.
    wind_cmd_ramp_active_ = false;
    wind_sine_mean_ = wind_cmd_target_;
    wind_sine_start_time_ = now_sec;
    wind_sine_active_ = true;

    gzmsg << "[gazebo_wind_plugin] Received runtime wind target sine mean: "
          << wind_sine_mean_.X() << ", "
          << wind_sine_mean_.Y() << ", "
          << wind_sine_mean_.Z() << " m/s, amp = "
          << wind_sine_amp_.X() << ", "
          << wind_sine_amp_.Y() << ", "
          << wind_sine_amp_.Z() << " m/s, freq = "
          << wind_sine_frequency_hz_ << " Hz, duration = "
          << wind_cmd_ramp_duration_ << " s\n";
  } else if (wind_cmd_mode_ == 3) {
    // C2.4 mode 3: random-step fixed-direction wind.
    wind_sine_active_ = false;
    wind_cmd_ramp_active_ = false;

    double norm = wind_cmd_target_.Length();
    if (norm < 1e-6) {
      wind_random_direction_.X() = 0.0;
      wind_random_direction_.Y() = 1.0;
      wind_random_direction_.Z() = 0.0;
    } else {
      wind_random_direction_ = wind_cmd_target_ / norm;
    }

    wind_random_start_time_ = now_sec;
    wind_random_next_update_time_ = now_sec;
    wind_random_active_ = true;

    gzmsg << "[gazebo_wind_plugin] Received runtime random-step direction: "
          << wind_random_direction_.X() << ", "
          << wind_random_direction_.Y() << ", "
          << wind_random_direction_.Z() << ", speed range = ["
          << wind_random_speed_min_ << ", " << wind_random_speed_max_
          << "] m/s, update period = "
          << wind_random_update_period_ << " s, duration = "
          << wind_cmd_ramp_duration_ << " s, seed = "
          << wind_random_seed_ << "\n";
  } else if (wind_cmd_mode_ == 4) {
    // C2.5 mode 4: three-axis random-step.
    // wind_cmd is not required for this mode; min/max vectors and mode command
    // define the random wind. Receiving wind_cmd while mode=4 only confirms
    // that runtime external wind control is active.
    wind_sine_active_ = false;
    wind_cmd_ramp_active_ = false;
    use_external_wind_cmd_ = true;

    gzmsg << "[gazebo_wind_plugin] Runtime wind_cmd received during mode=4. "
          << "Ignoring target vector and using random_min/random_max ranges.\n";
  } else if (wind_cmd_mode_ == 5) {
    // C2.6 mode 5: three-axis random-walk.
#if GAZEBO_MAJOR_VERSION >= 9
    const double now_sec = world_->SimTime().Double();
#else
    const double now_sec = world_->GetSimTime().Double();
#endif

    ignition::math::Vector3d initial_target(msg->x(), msg->y(), msg->z());

    if (initial_target.X() < wind_random_min_.X()) {
      initial_target.X() = wind_random_min_.X();
    }
    if (initial_target.X() > wind_random_max_.X()) {
      initial_target.X() = wind_random_max_.X();
    }
    if (initial_target.Y() < wind_random_min_.Y()) {
      initial_target.Y() = wind_random_min_.Y();
    }
    if (initial_target.Y() > wind_random_max_.Y()) {
      initial_target.Y() = wind_random_max_.Y();
    }
    if (initial_target.Z() < wind_random_min_.Z()) {
      initial_target.Z() = wind_random_min_.Z();
    }
    if (initial_target.Z() > wind_random_max_.Z()) {
      initial_target.Z() = wind_random_max_.Z();
    }

    double period = wind_random_update_period_;
    if (period < 1e-3) {
      period = 1e-3;
    }

    wind_sine_active_ = false;
    wind_random_active_ = true;
    // Do not update wind_random_walk_duration_ here.
    // The total random-walk duration is defined only by wind_mode.y.
    wind_random_walk_target_ = initial_target;
    wind_cmd_target_ = initial_target;
    wind_cmd_ramp_start_ = wind_cmd_current_;
    wind_cmd_ramp_start_time_ = now_sec;
    wind_cmd_ramp_duration_ = period;
    wind_cmd_ramp_active_ = true;
    wind_random_start_time_ = now_sec;
    wind_random_next_update_time_ = now_sec + period;
    wind_random_current_speed_ = wind_cmd_current_.Length();
    use_external_wind_cmd_ = true;

    gzmsg << "[gazebo_wind_plugin] Received random-walk initial wind target: "
          << initial_target.X() << ", "
          << initial_target.Y() << ", "
          << initial_target.Z() << " m/s, update period = "
          << period << " s\n";
  } else {
    // C2.2 mode 1: ramp.
    wind_sine_active_ = false;
    wind_random_active_ = false;
    wind_cmd_ramp_start_ = wind_cmd_current_;
    wind_cmd_ramp_start_time_ = now_sec;
    wind_cmd_ramp_active_ = true;

    gzmsg << "[gazebo_wind_plugin] Received runtime wind target ramp: "
          << wind_cmd_target_.X() << ", "
          << wind_cmd_target_.Y() << ", "
          << wind_cmd_target_.Z() << " m/s, duration = "
          << wind_cmd_ramp_duration_ << " s\n";
  }
}

void GazeboWindPlugin::WindModeCallback(ConstVector3dPtr& msg) {
  int requested_mode = static_cast<int>(msg->x());
  double requested_duration = msg->y();
  double requested_frequency_hz = msg->z();

  if (requested_mode < 0) {
    requested_mode = 0;
  }

  if (requested_mode > 5) {
    gzerr << "[gazebo_wind_plugin] Unsupported wind mode "
          << requested_mode << ". Falling back to ramp mode 1.\n";
    requested_mode = 1;
  }

  if (requested_duration < 0.0) {
    requested_duration = 0.0;
  }

  if (requested_frequency_hz < 0.0) {
    requested_frequency_hz = 0.0;
  }

  wind_cmd_mode_ = requested_mode;

  if (wind_cmd_mode_ == 0) {
    wind_cmd_ramp_duration_ = 0.0;
  } else {
    wind_cmd_ramp_duration_ = requested_duration;
  }

  if (wind_cmd_mode_ == 2) {
    wind_sine_frequency_hz_ = requested_frequency_hz;
  }

  if (wind_cmd_mode_ == 3 || wind_cmd_mode_ == 4 || wind_cmd_mode_ == 5) {
    wind_random_seed_ = static_cast<unsigned int>(requested_frequency_hz);
    wind_random_gen_.seed(wind_random_seed_);
  }

  if (wind_cmd_mode_ == 4) {
#if GAZEBO_MAJOR_VERSION >= 9
    const double now_sec = world_->SimTime().Double();
#else
    const double now_sec = world_->GetSimTime().Double();
#endif

    wind_sine_active_ = false;
    wind_cmd_ramp_active_ = false;
    wind_random_start_time_ = now_sec;
    wind_random_next_update_time_ = now_sec;
    wind_random_current_speed_ = wind_cmd_current_.Length();
    wind_random_active_ = true;
    use_external_wind_cmd_ = true;
  }

  if (wind_cmd_mode_ == 5) {
#if GAZEBO_MAJOR_VERSION >= 9
    const double now_sec = world_->SimTime().Double();
#else
    const double now_sec = world_->GetSimTime().Double();
#endif

    wind_sine_active_ = false;
    wind_cmd_ramp_active_ = false;

    // C2.6: store total random-walk duration from wind_mode.y.
    // Do not start random_active here. The random-walk sequence starts only
    // after the following wind_cmd provides the initial wind target.
    wind_random_walk_duration_ = wind_cmd_ramp_duration_;
    wind_random_start_time_ = now_sec;

    double period = wind_random_update_period_;
    if (period < 1e-3) {
      period = 1e-3;
    }

    wind_random_next_update_time_ = now_sec + period;
    wind_random_walk_target_ = wind_cmd_current_;
    wind_random_current_speed_ = wind_cmd_current_.Length();
    wind_random_active_ = false;
    use_external_wind_cmd_ = true;
  }

  gzmsg << "[gazebo_wind_plugin] Received runtime wind mode: mode="
        << wind_cmd_mode_ << ", duration="
        << wind_cmd_ramp_duration_ << " s, frequency="
        << wind_sine_frequency_hz_ << " Hz, random_seed="
        << wind_random_seed_ << "\n";
}

void GazeboWindPlugin::WindSineAmpCallback(ConstVector3dPtr& msg) {
  wind_sine_amp_.X() = msg->x();
  wind_sine_amp_.Y() = msg->y();
  wind_sine_amp_.Z() = msg->z();

  gzmsg << "[gazebo_wind_plugin] Received runtime sine wind amplitude: "
        << wind_sine_amp_.X() << ", "
        << wind_sine_amp_.Y() << ", "
        << wind_sine_amp_.Z() << " m/s\n";
}

void GazeboWindPlugin::WindRandomMinCallback(ConstVector3dPtr& msg) {
  wind_random_min_.X() = msg->x();
  wind_random_min_.Y() = msg->y();
  wind_random_min_.Z() = msg->z();

  gzmsg << "[gazebo_wind_plugin] Received runtime random-step 3-axis min: "
        << wind_random_min_.X() << ", "
        << wind_random_min_.Y() << ", "
        << wind_random_min_.Z() << " m/s\n";
}

void GazeboWindPlugin::WindRandomMaxCallback(ConstVector3dPtr& msg) {
  wind_random_max_.X() = msg->x();
  wind_random_max_.Y() = msg->y();
  wind_random_max_.Z() = msg->z();

  gzmsg << "[gazebo_wind_plugin] Received runtime random-step 3-axis max: "
        << wind_random_max_.X() << ", "
        << wind_random_max_.Y() << ", "
        << wind_random_max_.Z() << " m/s\n";
}

void GazeboWindPlugin::WindRandomParamsCallback(ConstVector3dPtr& msg) {
  wind_random_speed_min_ = msg->x();
  wind_random_speed_max_ = msg->y();
  wind_random_update_period_ = msg->z();

  // C2.6 mode=5 random_walk_3axis interpretation:
  //   x = max absolute x/y increment per update
  //   y = max absolute z increment per update
  //   z = update period
  wind_random_walk_delta_xy_ = msg->x();
  if (wind_random_walk_delta_xy_ < 0.0) {
    wind_random_walk_delta_xy_ = -wind_random_walk_delta_xy_;
  }

  wind_random_walk_delta_z_ = msg->y();
  if (wind_random_walk_delta_z_ < 0.0) {
    wind_random_walk_delta_z_ = -wind_random_walk_delta_z_;
  }

  if (wind_random_update_period_ < 1e-3) {
    wind_random_update_period_ = 1e-3;
  }

  gzmsg << "[gazebo_wind_plugin] Received runtime random-step parameters: "
        << "speed_min=" << wind_random_speed_min_
        << " m/s, speed_max=" << wind_random_speed_max_
        << " m/s, update_period=" << wind_random_update_period_
        << " s\n";
}

GZ_REGISTER_WORLD_PLUGIN(GazeboWindPlugin);
}
