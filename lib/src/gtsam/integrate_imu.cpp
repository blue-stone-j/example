#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
// #include <gtsam/navigation/PreintegratedImuMeasurements.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/geometry/Rot3.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>

struct ImuSample
{
  double dt;
  Eigen::Vector3d accel; // m/s^2 (body)
  Eigen::Vector3d gyro;  // rad/s (body)
};

int main()
{
  const double g = 9.80665;

  // ENU gravity in GTSAM uses NED by default; we can pass gravity magnitude only.
  auto p = gtsam::PreintegrationParams::MakeSharedU(g); // uses gravity along -Z in local frame

  // Minimal noise (non-zero). Values don't affect deterministic prediction readout.
  p->accelerometerCovariance = 1e-6 * gtsam::I_3x3;
  p->gyroscopeCovariance     = 1e-6 * gtsam::I_3x3;
  p->integrationCovariance   = 1e-8 * gtsam::I_3x3;

  gtsam::imuBias::ConstantBias biasZero; // all zeros

  // Initial static alignment:
  Eigen::Vector3d aavg_b(0, 0, -g); // m/s², measurement of imu when static
  // Align body -Z with gravity; yaw = 0
  Eigen::Vector3d zb      = -aavg_b.normalized();
  Eigen::Vector3d z_minus = -Eigen::Vector3d::UnitZ();
  Eigen::Vector3d v       = zb.cross(z_minus);
  double w                = std::sqrt(zb.squaredNorm() * z_minus.squaredNorm()) + zb.dot(z_minus);
  Eigen::Quaterniond q_align(w, v.x(), v.y(), v.z());
  q_align.normalize();

  gtsam::Rot3 R0 = gtsam::Rot3::Quaternion(q_align.w(), q_align.x(), q_align.y(), q_align.z());
  gtsam::Point3 p0(0, 0, 0);
  gtsam::Vector3 v0(0, 0, 0);
  gtsam::NavState state0(R0, p0, v0);

  // Build preintegrator and stream samples
  gtsam::PreintegratedImuMeasurements pim(p, biasZero);

  std::vector<ImuSample> stream; // = /* fill from sensor */;
  gtsam::NavState state = state0;

  for (const auto &s : stream)
  {
    // Feed one IMU sample
    gtsam::Vector3 acc(s.accel.x(), s.accel.y(), s.accel.z());
    gtsam::Vector3 gyr(s.gyro.x(), s.gyro.y(), s.gyro.z());
    pim.integrateMeasurement(acc, gyr, s.dt);

    // Option A: predict at each step
    state = pim.predict(state, biasZero);

    // Optionally read out pose/vel:
    gtsam::Pose3 T_wb  = state.pose();
    gtsam::Vector3 v_w = state.v();
    // Reset the preintegrator if you want step-by-step deltas:
    pim.resetIntegrationAndSetBias(biasZero);
  }

  // Final pose/velocity:
  std::cout << "p_w: " << state.pose().translation().transpose() << std::endl;
  gtsam::Rot3 R             = state.pose().rotation();
  gtsam::Quaternion q_gtsam = R.toQuaternion();
  Eigen::Quaterniond q_eig(q_gtsam.w(), q_gtsam.x(), q_gtsam.y(), q_gtsam.z());
  std::cout << "v_w: " << state.v().transpose() << std::endl;
  return 0;
}
