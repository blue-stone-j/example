/**
 * @file make_bag_from_images_imu.cpp
 * @brief Read timestamped images and IMU CSV, write ROS1 bag.
 * @author bs
 * @date 2025-09-25
 * @details
 *   Inputs:
 *     1) images.txt: "<stamp_sec> <path>"
 *     2) imu.csv    : header with fields timestamp,ax,ay,az,gx,gy,gz
 *   Output:
 *     bag file with topics:
 *       /camera/image_raw (sensor_msgs/Image, mono or color)
 *       /imu               (sensor_msgs/Imu)
 *   Notes:
 *     - Timestamps are interpreted as seconds since epoch (double).
 *     - If your timestamps are in nanoseconds, divide by 1e9 where indicated.
 *     - IMU orientation is left unset (NaN) and covariances provided as examples.
 *     - Requires: ROS Noetic, OpenCV, cv_bridge, sensor_msgs.
 * @LastEditTime 2025-09-25
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <iomanip>
#include <limits>

struct ImageItem
{
  double t; // seconds
  std::string path;
};

struct ImuItem
{
  double t;          // seconds
  double ax, ay, az; // m/s^2
  double gx, gy, gz; // rad/s
};

static bool parseImagesTxt(const std::string &path, std::vector<ImageItem> &out)
{
  std::ifstream ifs(path);
  if (!ifs.is_open())
  {
    std::cerr << "Failed to open images list: " << path << std::endl;
    return false;
  }
  std::string line;
  size_t ln = 0;
  while (std::getline(ifs, line))
  {
    ++ln;
    if (line.empty() || line[0] == '#') continue;
    std::istringstream iss(line);
    double t;
    std::string p;
    if (!(iss >> t >> p))
    {
      std::cerr << "Bad images.txt line " << ln << ": " << line << std::endl;
      return false;
    }
    out.push_back({t, p});
  }
  return true;
}

static std::vector<std::string> splitCSV(const std::string &s)
{
  // Simple CSV split (no quoted commas). Adequate for common IMU logs.
  std::vector<std::string> out;
  std::string cur;
  std::istringstream ss(s);
  while (std::getline(ss, cur, ',')) out.push_back(cur);
  return out;
}

static bool parseImuCsv(const std::string &path, std::vector<ImuItem> &out)
{
  std::ifstream ifs(path);
  if (!ifs.is_open())
  {
    std::cerr << "Failed to open imu csv: " << path << std::endl;
    return false;
  }
  std::string header;
  if (!std::getline(ifs, header))
  {
    std::cerr << "Empty imu csv: " << path << std::endl;
    return false;
  }
  auto cols = splitCSV(header);
  std::unordered_map<std::string, int> idx;
  for (int i = 0; i < (int)cols.size(); ++i)
  {
    idx[cols[i]] = i;
  }
  auto need = {"timestamp", "ax", "ay", "az", "gx", "gy", "gz"};
  for (auto k : need)
  {
    if (!idx.count(k))
    {
      std::cerr << "IMU CSV header missing column: " << k << std::endl;
      return false;
    }
  }
  std::string line;
  size_t ln = 1;
  while (std::getline(ifs, line))
  {
    ++ln;
    if (line.empty()) continue;
    auto v = splitCSV(line);
    if (v.size() < cols.size())
    {
      std::cerr << "Bad imu.csv line " << ln << ": " << line << std::endl;
      return false;
    }
    ImuItem it{};
    it.t  = std::stod(v[idx["timestamp"]]);
    it.ax = std::stod(v[idx["ax"]]);
    it.ay = std::stod(v[idx["ay"]]);
    it.az = std::stod(v[idx["az"]]);
    it.gx = std::stod(v[idx["gx"]]);
    it.gy = std::stod(v[idx["gy"]]);
    it.gz = std::stod(v[idx["gz"]]);
    out.push_back(it);
  }
  return true;
}

static ros::Time toRosTime(double t_sec)
{
  // If your timestamps are nanoseconds: return ros::Time().fromNSec((uint64_t)t_ns);
  return ros::Time(t_sec);
}

int main(int argc, char **argv)
{
  if (argc < 6)
  {
    std::cerr << "Usage:\n"
              << "  " << argv[0]
              << " <images.txt> <imu.csv> <output.bag> <camera_frame_id> <imu_frame_id>\n\n"
              << "Example:\n  " << argv[0]
              << " images.txt imu.csv out.bag camera_link imu_link\n";
    return 1;
  }

  const std::string images_txt = argv[1];
  const std::string imu_csv    = argv[2];
  const std::string bag_path   = argv[3];
  const std::string cam_frame  = argv[4];
  const std::string imu_frame  = argv[5];

  std::vector<ImageItem> images;
  std::vector<ImuItem> imus;

  if (!parseImagesTxt(images_txt, images)) return 2;
  if (!parseImuCsv(imu_csv, imus)) return 3;

  ros::Time::init(); // Safe even outside roscpp nodehandle
  rosbag::Bag bag;
  try
  {
    bag.open(bag_path, rosbag::bagmode::Write);
  }
  catch (const rosbag::BagException &e)
  {
    std::cerr << "Failed to open bag for writing: " << e.what() << std::endl;
    return 4;
  }

  const std::string img_topic = "/camera/image_raw";
  const std::string imu_topic = "/imu";

  // Write IMU messages
  for (const auto &m : imus)
  {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp    = toRosTime(m.t);
    imu_msg.header.frame_id = imu_frame;

    // Angular velocity (rad/s)
    imu_msg.angular_velocity.x = m.gx;
    imu_msg.angular_velocity.y = m.gy;
    imu_msg.angular_velocity.z = m.gz;

    // Linear acceleration (m/s^2)
    imu_msg.linear_acceleration.x = m.ax;
    imu_msg.linear_acceleration.y = m.ay;
    imu_msg.linear_acceleration.z = m.az;

    // Orientation unknown: set to NaN (ROS tools treat NaN as unknown)
    imu_msg.orientation.x = std::numeric_limits<double>::quiet_NaN();
    imu_msg.orientation.y = std::numeric_limits<double>::quiet_NaN();
    imu_msg.orientation.z = std::numeric_limits<double>::quiet_NaN();
    imu_msg.orientation.w = std::numeric_limits<double>::quiet_NaN();

    // Example covariances (diagonal). Adjust to your sensor specs.
    // Use -1 to indicate "not provided" where appropriate.
    imu_msg.orientation_covariance[0]         = -1; // unknown orientation
    imu_msg.angular_velocity_covariance[0]    = 1e-6;
    imu_msg.angular_velocity_covariance[4]    = 1e-6;
    imu_msg.angular_velocity_covariance[8]    = 1e-6;
    imu_msg.linear_acceleration_covariance[0] = 1e-4;
    imu_msg.linear_acceleration_covariance[4] = 1e-4;
    imu_msg.linear_acceleration_covariance[8] = 1e-4;

    bag.write(imu_topic, imu_msg.header.stamp, imu_msg);
  }

  // Write Image messages
  // Images may be grayscale or color; we will load with OpenCV and encode accordingly.
  for (const auto &it : images)
  {
    cv::Mat img = cv::imread(it.path, cv::IMREAD_UNCHANGED);
    if (img.empty())
    {
      std::cerr << "Warning: failed to read image: " << it.path << " — skipping.\n";
      continue;
    }

    std_msgs::Header header;
    header.stamp    = toRosTime(it.t);
    header.frame_id = cam_frame;

    sensor_msgs::ImagePtr msg;
    // Determine encoding:
    if (img.type() == CV_8UC1)
    {
      msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
    }
    else if (img.type() == CV_16UC1)
    {
      msg = cv_bridge::CvImage(header, "mono16", img).toImageMsg();
    }
    else if (img.type() == CV_8UC3)
    {
      // OpenCV loads BGR by default; ROS "bgr8" is valid.
      msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
    }
    else if (img.type() == CV_16UC3)
    {
      // Rare; convert to 8-bit if needed, or publish as rgb16/bgr16
      cv::Mat tmp;
      img.convertTo(tmp, CV_8UC3, 1.0 / 256.0);
      msg = cv_bridge::CvImage(header, "bgr8", tmp).toImageMsg();
    }
    else
    {
      // Fallback: convert to bgr8
      cv::Mat bgr;
      if (img.channels() == 4)
      {
        cv::cvtColor(img, bgr, cv::COLOR_BGRA2BGR);
      }
      else if (img.channels() == 3)
      {
        bgr = img;
      }
      else if (img.channels() == 1)
      {
        cv::cvtColor(img, bgr, cv::COLOR_GRAY2BGR);
      }
      else
      {
        std::cerr << "Unsupported image type (depth=" << img.depth()
                  << ", ch=" << img.channels() << ") for " << it.path << " — converting to bgr8.\n";
        bgr = img;
      }
      msg = cv_bridge::CvImage(header, "bgr8", bgr).toImageMsg();
    }

    bag.write("/camera/image_raw", header.stamp, *msg);
  }

  bag.close();
  std::cout << "Wrote bag: " << bag_path << "\n";
  return 0;
}
