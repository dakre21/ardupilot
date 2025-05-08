/* Author: David Akre
 * Date: 5/8/25
 * Description: Custom control for BPRL Demo(s)
 *
 */

#include <bits/stdc++.h>

#include <array>
#include <eigen/Eigen/Dense>

// (dakre) TODO clean this up later -> cpp / templated / etc.
// but the notion is 3-axis control using PID control ATM which is
// 0-indexed = 1-axis and so on in the base class construction
// PID follows standard convention with anti-windup for the integral
// term and low-pass filtering the derivative term with setpoint
// references of beta on the proportional term

namespace bprl_control {

inline float ClampInput(const float& input, float low = -1.0,
                        float high = 1.0) {
  return std::min(std::max(low, input), high);
}

inline Eigen::Vector3f VelXYZToUVW(const Eigen::Vector3f& v, const float& roll,
                                   const float& pitch, const float& yaw) {
  Eigen::Matrix3f R_bf;
  const auto cr = cos(roll), sr = sin(roll);
  const auto cp = cos(pitch), sp = sin(pitch);
  const auto cy = cos(yaw), sy = sin(yaw);
  R_bf << cp * cy, cp * sy, -sp, sr * sp * cy - cr * sy, sr * sp * sy + cr * cy,
      sr * cp, cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp;
  return R_bf * v;
}

inline Eigen::Vector3f RatesToPQR(const Eigen::Vector3f& rates,
                                  const float& roll, const float& pitch,
                                  const float& yaw) {
  Eigen::Matrix3f R;
  const auto cr = cos(roll), sr = sin(roll);
  const auto cp = cos(pitch), sp = sin(pitch);

  R << 1, 0, -sp, 0, cr, sr * cp, 0, -sr, cr * cp;
  return R * rates;
}

class PID {
 public:
  PID(float&& kp, float&& ki, float&& kd, const float& dt, float&& beta,
      float&& Tf, float&& Ti)
      : kp_(kp), ki_(ki), kd_(kd), dt_(dt), beta_(beta), Tf_(Tf), Ti_(Ti) {
    assert(dt_ > 0);
    assert(Ti_ > 0);
    assert(Tf_ > 0);
    Reset();
  }

  void Reset() {
    D_ = 0;
    I_ = 0;
    y_prev_ = 0;
  }

  float Update(const float& r, const float& y) {
    const auto P = kp_ * (beta_ * r - y);
    D_ = (Tf_ / (Tf_ + dt_)) * D_ - (kd_ / (Tf_ + dt_)) * (y - y_prev_);
    const auto v = P + I_ + D_;
    const auto u = ClampInput(v, -1.0, 1.0);
    I_ = I_ + ki_ * dt_ * (r - y) + dt_ / Ti_ * (u - v);
    y_prev_ = y;
    return u;
  }

 private:
  float D_, I_, y_prev_;
  float kp_, ki_, kd_, kt_;
  float dt_;
  float beta_;
  float Tf_, Ti_;
};

class ThreeAxisBaseControl {
 public:
  ThreeAxisBaseControl(std::array<PID, 3>&& pids) : pids_(pids) {
    assert(pids.size() == 3);
  }

  virtual ~ThreeAxisBaseControl() {}

  virtual void step(const Eigen::Vector3f& r, const Eigen::Vector3f& y,
                    Eigen::Vector3f& u) {
    UpdatePIDs(r, y, u);
  }

 protected:
  virtual void UpdatePIDs(const Eigen::Vector3f& r, const Eigen::Vector3f& y,
                          Eigen::Vector3f& u) {
    for (size_t i = 0; i < pids_.size(); ++i) {
      u(i) = pids_[i].Update(r(i), y(i));
    }
  }

  std::array<PID, 3> pids_;
};

class AttitudeControl : public ThreeAxisBaseControl {
 public:
  AttitudeControl(std::array<PID, 3>&& pids)
      : ThreeAxisBaseControl(std::move(pids)) {}
};

class RateControl : public ThreeAxisBaseControl {
 public:
  RateControl(std::array<PID, 3>&& pids)
      : ThreeAxisBaseControl(std::move(pids)) {}
};

}  // namespace bprl_control
