#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace vis
{
// Normalize a quaternion (safe for non-unit inputs)
inline Eigen::Quaternionf NormalizeQuat(const Eigen::Quaternionf &q)
{
  Eigen::Quaternionf qq = q;
  qq.normalize();
  return qq;
}

// Build a homogeneous transform from translation and quaternion
inline Eigen::Matrix4f MakeSE3(const Eigen::Vector3f &t, const Eigen::Quaternionf &q_raw)
{
  Eigen::Quaternionf q = NormalizeQuat(q_raw);
  Eigen::Matrix4f T    = Eigen::Matrix4f::Identity();
  T.block<3, 3>(0, 0)  = q.toRotationMatrix();
  T.block<3, 1>(0, 3)  = t;
  return T;
}

// Helper: transform a 3D point by 4x4
inline Eigen::Vector3f TransformPoint(const Eigen::Matrix4f &T, const Eigen::Vector3f &p)
{
  Eigen::Vector4f ph;
  ph << p, 1.0f;
  Eigen::Vector4f po = T * ph;
  return po.head<3>();
}

// Draw XYZ axes for a frame T_w_f with given scale and id prefix
inline void AddAxes(pcl::visualization::PCLVisualizer::Ptr &viewer,
                    const Eigen::Matrix4f &T_w_f,
                    float scale,
                    const std::string &id_prefix)
{
  const Eigen::Vector3f Ow = T_w_f.block<3, 1>(0, 3);
  const Eigen::Vector3f Xw = TransformPoint(T_w_f, Eigen::Vector3f(scale, 0.f, 0.f));
  const Eigen::Vector3f Yw = TransformPoint(T_w_f, Eigen::Vector3f(0.f, scale, 0.f));
  const Eigen::Vector3f Zw = TransformPoint(T_w_f, Eigen::Vector3f(0.f, 0.f, scale));

  viewer->addLine<pcl::PointXYZ>(
      pcl::PointXYZ(Ow.x(), Ow.y(), Ow.z()),
      pcl::PointXYZ(Xw.x(), Xw.y(), Xw.z()),
      1.0, 0.0, 0.0, id_prefix + "_x"); // red

  viewer->addLine<pcl::PointXYZ>(
      pcl::PointXYZ(Ow.x(), Ow.y(), Ow.z()),
      pcl::PointXYZ(Yw.x(), Yw.y(), Yw.z()),
      0.0, 1.0, 0.0, id_prefix + "_y"); // green

  viewer->addLine<pcl::PointXYZ>(
      pcl::PointXYZ(Ow.x(), Ow.y(), Ow.z()),
      pcl::PointXYZ(Zw.x(), Zw.y(), Zw.z()),
      0.0, 0.0, 1.0, id_prefix + "_z"); // blue
}

// Draw a simple pinhole frustum for the camera at T_w_c
// fx, fy, cx, cy: intrinsics; w,h: image size; depth_z: displayed depth in camera meters
inline void AddCameraFrustum(pcl::visualization::PCLVisualizer::Ptr &viewer,
                             const Eigen::Matrix4f &T_w_c,
                             float fx, float fy, float cx, float cy,
                             int w, int h, float depth_z,
                             const std::string &id_prefix)
{
  // Corner pixels on an image plane at Z = depth_z in the camera frame
  auto pix2cam = [&](float u, float v) -> Eigen::Vector3f {
    float X = (u - cx) * depth_z / fx;
    float Y = (v - cy) * depth_z / fy;
    float Z = depth_z;
    return Eigen::Vector3f(X, Y, Z);
  };

  Eigen::Vector3f Cc(0.f, 0.f, 0.f); // camera center in camera frame
  Eigen::Vector3f Cw = TransformPoint(T_w_c, Cc);

  Eigen::Vector3f tl = TransformPoint(T_w_c, pix2cam(0, 0));
  Eigen::Vector3f tr = TransformPoint(T_w_c, pix2cam(w - 1, 0));
  Eigen::Vector3f br = TransformPoint(T_w_c, pix2cam(w - 1, h - 1));
  Eigen::Vector3f bl = TransformPoint(T_w_c, pix2cam(0, h - 1));

  // Pyramid edges
  auto addEdge = [&](const Eigen::Vector3f &a, const Eigen::Vector3f &b, double r, double g, double bcol, const std::string &id) {
    viewer->addLine<pcl::PointXYZ>(
        pcl::PointXYZ(a.x(), a.y(), a.z()),
        pcl::PointXYZ(b.x(), b.y(), b.z()),
        r, g, bcol, id);
  };

  addEdge(Cw, tl, 1.0, 1.0, 0.0, id_prefix + "_ctl"); // yellow
  addEdge(Cw, tr, 1.0, 1.0, 0.0, id_prefix + "_ctr");
  addEdge(Cw, br, 1.0, 1.0, 0.0, id_prefix + "_cbr");
  addEdge(Cw, bl, 1.0, 1.0, 0.0, id_prefix + "_cbl");

  // Image-plane rectangle
  addEdge(tl, tr, 0.8, 0.8, 0.8, id_prefix + "_top");
  addEdge(tr, br, 0.8, 0.8, 0.8, id_prefix + "_right");
  addEdge(br, bl, 0.8, 0.8, 0.8, id_prefix + "_bottom");
  addEdge(bl, tl, 0.8, 0.8, 0.8, id_prefix + "_left");
}

// Draw gravity vector (e.g., [0, 0, -g] in IMU frame) transformed to world and rendered as an arrow
inline void AddImuGravityArrow(pcl::visualization::PCLVisualizer::Ptr &viewer,
                               const Eigen::Matrix4f &T_w_i,
                               const Eigen::Vector3f &g_imu, // gravity direction in IMU frame (e.g., {0,0,-1})
                               float length,
                               const std::string &id)
{
  Eigen::Vector3f O_w   = T_w_i.block<3, 1>(0, 3);
  Eigen::Vector3f tip_i = g_imu.normalized() * length; // direction in IMU
  Eigen::Vector3f tip_w = TransformPoint(T_w_i, tip_i);

  viewer->addArrow(
      pcl::PointXYZ(tip_w.x(), tip_w.y(), tip_w.z()),
      pcl::PointXYZ(O_w.x(), O_w.y(), O_w.z()),
      0.2, 0.6, 1.0, false, id);
}

} // namespace vis

int main()
{
  // ===========================
  // 1) Define your extrinsics
  // World (W) is your chosen global frame.
  // Replace the following with your calibrated results.

  // Example: Camera pose in world (translation in meters, quaternion w,x,y,z)
  Eigen::Vector3f t_wc_left(-0.024f, -0.075f, -0.063f);
  Eigen::Quaternionf q_wc_left(-0.57075f, -0.57716f, -0.40885f, 0.41093f);
  Eigen::Matrix4f T_w_c_left = vis::MakeSE3(t_wc_left, q_wc_left);

  Eigen::Vector3f t_wc_right(-0.024f, 0.029f, -0.063f);
  Eigen::Quaternionf q_wc_right(0.58058f, -0.57667f, 0.39957f, 0.41317f);
  Eigen::Matrix4f T_w_c_right = vis::MakeSE3(t_wc_right, q_wc_right);

  // LiDAR pose in world
  Eigen::Vector3f t_wl(-0.011f, -0.02329f, 0.04412f);
  Eigen::Quaternionf q_wl(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::Matrix4f T_w_l = vis::MakeSE3(t_wl, q_wl);

  // IMU pose in world
  Eigen::Vector3f t_wi(0.0f, 0.0f, 0.0f);
  Eigen::Quaternionf q_wi(1.0f, 0.0f, 0.0f, 0.0f);
  Eigen::Matrix4f T_w_i = vis::MakeSE3(t_wi, q_wi);

  // ===========================
  // 2) (Optional) Camera intrinsics for frustum drawing
  float fx = 800.0f, fy = 800.0f, cx = 2000.0f, cy = 1500.0f;
  int img_w = 4000, img_h = 3000;
  float frustum_depth = 0.01f; // meters

  // ===========================
  // 3) Create viewer
  auto viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Sensor Frames"));
  viewer->setBackgroundColor(0.05, 0.05, 0.07);
  // viewer->addCoordinateSystem(0.5, "W_world"); // world axes at origin

  // ===========================
  // 4) Draw sensor frames
  vis::AddAxes(viewer, T_w_c_left, 0.05f, "C_cam_left");
  vis::AddAxes(viewer, T_w_c_right, 0.05f, "C_cam_right");
  vis::AddAxes(viewer, T_w_l, 0.05f, "L_lidar");
  vis::AddAxes(viewer, T_w_i, 0.05f, "I_imu");

  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "C_cam_left_x");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "C_cam_left_y");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "C_cam_left_z");

  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "C_cam_right_x");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "C_cam_right_y");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "C_cam_right_z");

  // LiDAR axes thicker
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "L_lidar_x");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "L_lidar_y");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "L_lidar_z");

  // IMU axes thicker
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "I_imu_x");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "I_imu_y");
  viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, "I_imu_z");

  // Distinguish with text labels
  {
    auto label = [&](const Eigen::Matrix4f &T, const std::string &s, double r, double g, double b) {
      Eigen::Vector3f p = T.block<3, 1>(0, 3);
      viewer->addText3D(s, pcl::PointXYZ(p.x(), p.y(), p.z() + 0.01f), 0.005, r, g, b, s + "_label");
    };
    label(T_w_c_left, "Left Camera", 1.0, 0.5, 0.0);
    label(T_w_c_right, "Right Camera", 1.0, 0.5, 0.0);
    label(T_w_l, "LiDAR", 0.0, 1.0, 0.5);
    label(T_w_i, "IMU", 0.4, 0.8, 1.0);

    // label(T_w_c_left, "左相机", 1.0, 0.5, 0.0);
    // label(T_w_c_right, "右相机", 1.0, 0.5, 0.0);
    // label(T_w_l, "激光雷达", 0.0, 1.0, 0.5);
    // label(T_w_i, "imu", 0.4, 0.8, 1.0);
  }

  // ===========================
  // 5) Draw camera frustum and IMU gravity vector
  // vis::AddCameraFrustum(viewer, T_w_c_left, fx, fy, cx, cy, img_w, img_h, frustum_depth, "C_frustum_left");
  // vis::AddCameraFrustum(viewer, T_w_c_right, fx, fy, cx, cy, img_w, img_h, frustum_depth, "C_frustum_right");

  // IMU gravity direction (assume gravity points along IMU -Z; adjust as needed)
  // Eigen::Vector3f g_imu(0.f, 0.f, -0.1f);
  // vis::AddImuGravityArrow(viewer, T_w_i, g_imu, 0.1f, "I_gravity");

  // ===========================
  // 6) (Optional) Add a tiny point cloud or ground plane grid for context
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr grid(new pcl::PointCloud<pcl::PointXYZ>());
    const float s    = 0.3f;  // half size
    const float step = 0.01f; // spacing
    for (float x = -s; x <= s; x += step)
    {
      for (float y = -s; y <= s; y += step)
      {
        grid->push_back(pcl::PointXYZ(x, y, 0.0f));
      }
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(grid, 120, 120, 120);
    viewer->addPointCloud<pcl::PointXYZ>(grid, color, "grid_xy");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "grid_xy");
  }

  // ===========================
  // 7) Camera view & screenshot (optional)
  viewer->setCameraPosition(
      0.3, 0.3, 0.2,   // camera position
      0.0, 0.0, 0.0,   // view target
      0.0, 0.0, -0.1); // up

  // Save a screenshot once at start (you can also bind a key to capture later)
  viewer->saveScreenshot("sensor_frames.png");
  std::cout << "Saved screenshot: sensor_frames.png\n";

  // ===========================
  // 8) Spin
  while (!viewer->wasStopped())
  {
    viewer->spinOnce(16);
  }
  return 0;
}
