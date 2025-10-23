/**
 * @file .cpp
 * @brief Compute endpoint velocities from GTSAM PreintegratedImuMeasurements and two poses.
 * @author bs
 * @date 2025-10-28
 * @details
 *  Given Pose3 at times ti and tj, raw IMU between them, gravity, and bias estimates,
 *  this computes world-frame velocities v_i and v_j using GTSAM preintegration.
 */

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Vector.h>

#include <vector>
#include <utility>
#include <iostream>

struct ImuData
{
  gtsam::Vector3 acc;  // measured specific force (m/s^2)
  gtsam::Vector3 gyro; // measured angular rate (rad/s)
  double dt;           // seconds
};

struct EndpointVelResult
{
  gtsam::Vector3 v_i_world;
  gtsam::Vector3 v_j_world;
};

EndpointVelResult computeEndpointVelocities(
    const gtsam::Pose3 &T_wi,                     // pose at t_i (body->world)
    const gtsam::Pose3 &T_wj,                     // pose at t_j (body->world)
    const std::vector<ImuData> &imu,              // imu samples over [t_i, t_j)
    const gtsam::imuBias::ConstantBias &bias_hat, // bias used for preintegration
    double gravity_mag,                           // e.g. 9.81
    const gtsam::Matrix33 &acc_noise_cov,         // continuous-time accel noise covariance (sigma^2)
    const gtsam::Matrix33 &gyro_noise_cov,        // continuous-time gyro noise covariance (sigma^2)
    const gtsam::Matrix33 &integ_noise_cov        // integration noise covariance (usually small diag)
)
{
  // 1) Preintegration parameter setup (gravity points along negative Z by MakeSharedU)
  auto params                     = gtsam::PreintegrationParams::MakeSharedU(gravity_mag);
  params->accelerometerCovariance = acc_noise_cov;
  params->gyroscopeCovariance     = gyro_noise_cov;
  params->integrationCovariance   = integ_noise_cov;

  // 2) Create preintegrator with current bias estimates
  gtsam::PreintegratedImuMeasurements pim(params, bias_hat);

  // 3) Feed IMU samples
  double sum_dt = 0.0;
  for (const auto &s : imu)
  {
    pim.integrateMeasurement(s.acc, s.gyro, s.dt);
    sum_dt += s.dt;
  }

  // 4) Extract preintegrated terms
  const double dt         = pim.deltaTij(); // should equal sum_dt
  const gtsam::Vector3 dP = pim.deltaPij(); // Δp_ij (in body_i frame)
  const gtsam::Vector3 dV = pim.deltaVij(); // Δv_ij (in body_i frame)
  // const gtsam::Rot3 dR = pim.deltaRij();          // if you need it

  // 5) Pull out world-frame poses
  const gtsam::Rot3 &R_wi   = T_wi.rotation(); // body_i -> world
  const gtsam::Point3 &p_wi = T_wi.translation();
  const gtsam::Point3 &p_wj = T_wj.translation();

  // 6) World gravity vector (MakeSharedU sets g_world = [0,0,-|g|]^T)
  const gtsam::Vector3 g_world(0.0, 0.0, -gravity_mag);

  // 7) Solve for v_i (position kinematics) and then v_j (velocity kinematics)
  gtsam::Vector3 v_i_world =
      (p_wj.vector() - p_wi.vector()
       - 0.5 * g_world * (dt * dt)
       - R_wi.matrix() * dP)
      / dt;

  gtsam::Vector3 v_j_world =
      v_i_world + g_world * dt + R_wi.matrix() * dV;

  return EndpointVelResult{v_i_world, v_j_world};
}

// ------------------------- Example usage -------------------------
int main()
{
  // Example: identity poses with a small translation, fake IMU
  gtsam::Pose3 T_wi = gtsam::Pose3::identity();
  gtsam::Pose3 T_wj(gtsam::Rot3::identity(), gtsam::Point3(1.0, 0.0, 0.0)); // moved 1 m in x

  std::vector<ImuData> imu;
  // Provide your real IMU samples here. For demo, pretend a single 0.5 s step with zero readings:
  imu.push_back(ImuData{gtsam::Vector3(0, 0, 0), gtsam::Vector3(0, 0, 0), 0.5});
  imu.push_back(ImuData{gtsam::Vector3(0, 0, 0), gtsam::Vector3(0, 0, 0), 0.5});

  // Bias estimates (set to zero for this toy example)
  gtsam::imuBias::ConstantBias bias_hat(gtsam::Vector3::Zero(), gtsam::Vector3::Zero());

  // Noise (choose realistic values for your sensor; here: tiny)
  gtsam::Matrix33 acc_Q = 1e-4 * gtsam::I_3x3;
  gtsam::Matrix33 gyr_Q = 1e-6 * gtsam::I_3x3;
  gtsam::Matrix33 int_Q = 1e-8 * gtsam::I_3x3;

  double g = 9.81;

  EndpointVelResult r = computeEndpointVelocities(T_wi, T_wj, imu, bias_hat, g, acc_Q, gyr_Q, int_Q);

  std::cout << "v_i^w = " << r.v_i_world.transpose() << std::endl;
  std::cout << "v_j^w = " << r.v_j_world.transpose() << std::endl;
  return 0;
}
