/* Author: David Akre
 * Date: 4/26/25
 * Description: Control for quadrotor
 */

#include <eigen/Eigen/Dense>

#include "Copter.h"

void ModeBPRL::run() {
  // (0) Forward declarations
  const int n_states = 12;
  const int n_inputs = 4;

  auto clamp_input = [](const float& input, float lb = -1.0) -> float {
    return std::min(std::max(lb, input), 1.0);
  };

  auto ned_to_body_velocity = [](Eigen::Vector3f& v, const float& roll,
                                 const float& pitch, const float& yaw) {
    Eigen::Matrix3f R_bf;
    const auto cr = cos(roll), sr = sin(roll);
    const auto cp = cos(pitch), sp = sin(pitch);
    const auto cy = cos(yaw), sy = sin(yaw);
    R_bf << cp * cy, cp * sy, -sp, sr * sp * cy - cr * sy,
        sr * sp * sy + cr * cy, sr * cp, cr * sp * cy + sr * sy,
        cr * sp * sy - sr * cy, cr * cp;
    v = R_bf * v;
  };

  // (1) Acquire desired states
  Eigen::VectorXf U(n_inputs);
  Eigen::VectorXf X(n_states);
  Eigen::MatrixXf K(4, 12);

  Vector3f vel_ned, pos_ned, gyro;
  bool rc = motors->armed();
  rc &= ahrs.get_velocity_NED(vel_ned);
  rc &= ahrs.get_relative_position_NED_origin(pos_ned);
  if (!rc) {
    // gcs().send_text(MAV_SEVERITY_CRITICAL, "Motors not armed!");
    return;
  }

  float roll = ahrs.get_roll();
  float pitch = ahrs.get_pitch();
  float yaw = ahrs.get_yaw();

  Eigen::Vector3f vel;
  vel << vel_ned.x, vel_ned.y, vel_ned.z;
  ned_to_body_velocity(vel, roll, pitch, yaw);

  X << pos_ned.x, vel(0), pos_ned.y, vel(1), pos_ned.z, vel(2), roll, gyro.x,
      pitch, gyro.y, yaw, gyro.z;

  if (nullptr == pos_control || nullptr == attitude_control) {
    gcs().send_text(MAV_SEVERITY_CRITICAL,
                    "Pos and Attiude control ptrs have not been initialized");
    return;
  }

  const Vector3d& des_pos_neu = pos_control->get_pos_target_cm();
  const Vector3f& des_angs = attitude_control->get_att_target_euler_rad();
  const Vector3f& des_vel_neu = pos_control->get_vel_target_cms();
  const Vector3f& des_rates = attitude_control->rate_bf_targets();

  // (2) Apply LQR control with static K
  // (dakre) implicit transform from NEU to NED below for relative velocity
  Eigen::VectorXf X_ref(n_states);
  vel << des_vel_neu.x, des_vel_neu.y, -des_vel_neu.z;
  vel /= 100;
  ned_to_body_velocity(vel, des_angs.x, des_angs.y, des_angs.z);

  X_ref << des_pos_neu.x / 100, vel(0), des_pos_neu.y / 100, vel(1),
      -des_pos_neu.z / 100, vel(2), des_angs.x, des_rates.x, des_angs.y,
      des_rates.y, des_angs.z, des_rates.z;

  // gcs().send_text(MAV_SEVERITY_INFO,
  //                 "x1: %.2f x3: %.2f x5: %.2f x7: %.2f x9: %.2f x11: %.2f",
  //                 X_ref(1), X_ref(3), X_ref(5), X_ref(7), X_ref(9),
  //                 X_ref(11));

  // gcs().send_text(MAV_SEVERITY_INFO,
  //                 "x1: %.2f x3: %.2f x5: %.2f x7: %.2f x9: %.2f x11: %.2f",
  //                 X(1), X(3), X(5), X(7), X(9), X(11));

  const auto X_delta = X - X_ref;

  gcs().send_text(MAV_SEVERITY_INFO,
                  "x0: %.2f x1: %.2f x2: %.2f x3: %.2f x4: %.2f x5: %.2f x6: "
                  "%.2f x7: %.2f x8: %.2f x9: %.2f x10: %.2f x11: %.2f",
                  X_delta(0), X_delta(1), X_delta(2), X_delta(3), X_delta(4),
                  X_delta(5), X_delta(6), X_delta(7), X_delta(8), X_delta(9),
                  X_delta(10), X_delta(11));

  Eigen::VectorXf U_ref(n_inputs);
  U_ref << motors->get_throttle_hover(), 0, 0, 0;

  K << -4.370000e-01, 3.380000e-01, -3.070000e-01, 1.610000e-01, 1.725000e+00,
      4.900000e-02, -9.880000e-01, -5.060000e-01, -2.515000e+00, 1.790000e+00,
      3.370000e-01, 3.031000e+00, -1.900000e+00, -1.043000e+00, 1.537000e+00,
      -3.700000e-01, -1.037000e+00, -5.000000e-03, -1.000000e-02, 9.000000e-03,
      -4.987000e+00, 2.885000e+00, 2.450000e-01, 1.421000e+00, 1.860000e-01,
      -4.674000e+00, 1.377000e+00, -1.854000e+00, -4.000000e-03, -1.566000e+00,
      -3.100000e-02, 2.410000e-01, 3.569000e+00, -6.910000e-01, 4.940000e+00,
      -7.880000e-01, -3.640000e-01, -3.800000e-01, 2.196000e+00, 8.400000e-02,
      1.954000e+00, -1.700000e-01, 5.830000e-01, 3.980000e-01, 4.700000e-02,
      1.689000e+00, 9.900000e-02, 4.483000e+00;

  // \Delta(U) = -K * (X - X_ref) \to U = -K dot (X - X_ref) + U_ref
  U = -K * (X_delta) + U_ref;
  gcs().send_text(MAV_SEVERITY_INFO, "u0: %.2f u1: %.2f u2: %.2f u3: %.2f",
                  U(0), U(1), U(2), U(3));

  // (4) Apply normalize inputs and send to motors
  auto u_throttle = clamp_input(U(0), 0.0);
  auto u_roll = clamp_input(U(1));
  auto u_pitch = clamp_input(U(2));
  auto u_yaw = clamp_input(U(3));

  motors->set_desired_spool_state(
      AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

  motors->set_throttle(u_throttle);
  motors->set_roll(u_roll);
  motors->set_pitch(u_pitch);
  motors->set_yaw(u_yaw);

  // gcs().send_text(MAV_SEVERITY_INFO, "u0: %.2f u1: %.2f u2: %.2f u3: %.2f",
  // u_throttle, u_roll, u_pitch, u_yaw);

  motors->output();
}
