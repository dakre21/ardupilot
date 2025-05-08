/* Author: David Akre
 * Date: 4/26/25
 * Description: Control for quadrotor
 */

#include "BPRL_Control/BPRL_Control.h"
#include "Copter.h"

#define OUTER_LOOP 40

bool ModeBPRL::init(bool ignore_checks) {
  (void)ignore_checks;
  float dt = 1.0 / 400.0;
  std::array<bprl_control::PID, 3> att_pids{
      bprl_control::PID(0.8, 2e-2, 1e-3, OUTER_LOOP * dt, 1, 1, 1),
      bprl_control::PID(0.8, 2e-2, 1e-3, OUTER_LOOP * dt, 1, 1, 1),
      bprl_control::PID(0, 0, 0, OUTER_LOOP * dt, 1, 1, 1)};
  att_ctrl_ = std::shared_ptr<bprl_control::AttitudeControl>(
      new bprl_control::AttitudeControl(std::move(att_pids)));

  std::array<bprl_control::PID, 3> rate_pids{
      bprl_control::PID(0.2, 1e-2, 4e-3, dt, 1.0, 1, 1),
      bprl_control::PID(0.2, 1e-2, 4e-3, dt, 1.0, 1, 1),
      bprl_control::PID(0, 0, 0, dt, 1.0, 1, 1)};
  rate_ctrl_ = std::shared_ptr<bprl_control::RateControl>(
      new bprl_control::RateControl(std::move(rate_pids)));

  return true;
}

void ModeBPRL::run() {
  static size_t count = OUTER_LOOP + 1;
  static Eigen::Vector3f ref_rates(0, 0, 0);

  // (0) Check motors
  if (!motors->armed()) {
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    return;
  }
  motors->set_desired_spool_state(
      AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if 0
  Vector3f ang_vel;
  attitude_control->input_quaternion(quat_, ang_vel);
  attitude_control->set_throttle_out(thrust_, true, g.throttle_filt);
#else
  float r, p, y;
  if (count > OUTER_LOOP) {
    quat_.to_euler(r, p, y);
    Eigen::Vector3f ref;
    ref << r, p, y;
    gcs().send_text(MAV_SEVERITY_INFO, "r0 %f r1 %f r3 %f", r, p, y);

    Quaternion veh_att;
    if (!ahrs.get_quaternion(veh_att)) {
      gcs().send_text(MAV_SEVERITY_ERROR, "Failed to acquire measured quat");
      return;
    }
    veh_att.to_euler(r, p, y);

    Eigen::Vector3f meas;
    meas << r, p, y;

    gcs().send_text(MAV_SEVERITY_INFO, "m0 %f m1 %f m3 %f", r, p, y);
    att_ctrl_->step(ref, meas, ref_rates);
    gcs().send_text(MAV_SEVERITY_INFO, "u0 %f u1 %f u3 %f", ref_rates(0),
                    ref_rates(1), ref_rates(2));
    count = 0;
  }
  const auto gyro = ahrs.get_gyro_latest();
  Eigen::Vector3f rates;
  rates << gyro.x, gyro.y, gyro.z;
  r = ahrs.get_roll();
  p = ahrs.get_pitch();
  y = ahrs.get_yaw();
  const auto meas = bprl_control::RatesToPQR(rates, r, p, y);
  // gcs().send_text(MAV_SEVERITY_INFO, "mm0 %f mm1 %f mm3 %f", meas(0),
  // meas(1), meas(2));
  Eigen::Vector3f u_angles;
  rate_ctrl_->step(ref_rates, meas, u_angles);
  // gcs().send_text(MAV_SEVERITY_INFO, "uu0 %f uu1 %f uu3 %f", u_angles(0),
  // u_angles(1), u_angles(2));
  motors->set_roll(u_angles(0));
  motors->set_pitch(u_angles(1));
  motors->set_yaw(u_angles(2));
  motors->set_throttle(thrust_);
  motors->output();
#endif
  count++;
}
