/**
 * @file CameraRuntimeOwned.cpp
 * @brief Long-lived, runtime-selectable GTSAM calibration with camera variant
 * @author bs
 * @LastEditTime 2025-09-30
 */

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

#include <variant>
#include <memory>
#include <stdexcept>

using CamDS2        = gtsam::PinholeCamera<gtsam::Cal3DS2>;
using CamFisheye    = gtsam::PinholeCamera<gtsam::Cal3Fisheye>;
using CameraVariant = std::variant<CamDS2, CamFisheye>;

class CameraBundle
{
 public:
  // Prefer one smart-pointer flavor consistently (GTSAM often uses boost::shared_ptr).
  using CalBasePtr = std::shared_ptr<gtsam::Cal3>; // replace with boost::shared_ptr if needed

  explicit CameraBundle(const gtsam::Pose3 &pose,
                        CalBasePtr K) :
    pose_(pose),
    K_(std::move(K))
  {
    rebuildCamera();
  }

  // --- Accessors you can use anywhere else in your code ---
  const CalBasePtr &calibration() const { return K_; } // share/use K_ elsewhere
  const gtsam::Pose3 &pose() const { return pose_; }

  // Update pose or calibration at runtime; camera is kept in sync.
  void setPose(const gtsam::Pose3 &pose)
  {
    pose_ = pose;
    rebuildCamera();
  }

  void setCalibration(CalBasePtr K)
  {
    K_ = std::move(K);
    rebuildCamera();
  }

  // Example uniform operations over the variant
  gtsam::Point2 project(const gtsam::Point3 &Pw) const
  {
    return std::visit([&](const auto &c) { return c.project(Pw); }, cam_);
  }

  const CameraVariant &camera() const { return cam_; } // if you need direct access

 private:
  void rebuildCamera()
  {
    // Try DS2
    if (auto k = std::dynamic_pointer_cast<gtsam::Cal3DS2>(K_))
    {
      cam_ = CamDS2(pose_, *k);
      return;
    }
    // Try Fisheye
    if (auto k = std::dynamic_pointer_cast<gtsam::Cal3Fisheye>(K_))
    {
      cam_ = CamFisheye(pose_, *k);
      return;
    }
    throw std::runtime_error("Unsupported calibration type held in K_");
  }

 private:
  gtsam::Pose3 pose_;
  CalBasePtr K_;      // <-- long-lived, shared elsewhere
  CameraVariant cam_; // <-- rebuilt whenever K_ or pose_ changes
};

// ------------------ Example usage ------------------
int main()
{
  // Pose
  gtsam::Pose3 Twc(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0, 0, 0));

  // Decide calibration at runtime (e.g., from config/CLI)
  bool useFisheye = false;

  std::shared_ptr<gtsam::Cal3> K; // or boost::shared_ptr<gtsam::Cal3>
  if (!useFisheye)
  {
    K = std::make_shared<gtsam::Cal3DS2>(
        500.0, 500.0, 0.0, 320.0, 240.0, // fx, fy, s, u0, v0
        0.01, -0.001, 0.0001, -0.0001    // k1, k2, p1, p2
    );
  }
  else
  {
    K = std::make_shared<gtsam::Cal3Fisheye>(
        500.0, 500.0, 0.0, 320.0, 240.0,
        0.01, -0.001, 0.0001, -0.0001);
  }

  CameraBundle bundle(Twc, K);

  // Use K_ elsewhere (same pointer, long-lived)
  auto sharedK = bundle.calibration(); // pass to other subsystems as needed

  // Use camera uniformly
  gtsam::Point2 uv = bundle.project(gtsam::Point3(0, 0, 5));
  (void)uv;

  // Later, switch to another model at runtime:
  bundle.setCalibration(std::make_shared<gtsam::Cal3Fisheye>(
      520.0, 520.0, 0.0, 320.0, 240.0, 0.0, 0.0, 0.0, 0.0));
  // Camera variant automatically rebuilt.

  return 0;
}
