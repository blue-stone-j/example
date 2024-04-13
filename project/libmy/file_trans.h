#ifndef FILE_TRANS
#define FILE_TRANS

#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include "lidaru.h"

#include "param_server.h"

class FileTrans : public ParamServer
{
public:
  FileTrans()
  {
    // xy2cloud();
    if (mode == 1)
    {
      cloud2ll();
      ROS_INFO_STREAM("\033[32m mode: cloud2ll\033[0m");
    }
    else if (mode == 2)
    {
      ll2cloud();
      ROS_INFO_STREAM("\033[32m mode: ll2cloud\033[0m");
    }
  }
  std::string pcd_name, csv_name;
  void cloud2ll()
  {
    pcl::PointCloud<PointType> cloud;
    pcl::io::loadPCDFile(pcd_name, cloud);

    std::fstream map_lidar;
    map_lidar.open(csv_name, std::ios::out);
    geographic_msgs::GeoPoint geo_point;
    geodesy::UTMPoint utm;
    utm.band = mine_orig.band;
    utm.zone = mine_orig.zone;
    for (auto pt : cloud)
    {
      utm.easting = pt.x + mine_orig.easting;
      utm.northing = pt.y + mine_orig.northing;
      utm.altitude = pt.z + mine_orig.altitude;
      geo_point = geodesy::toMsg(utm);
      map_lidar << std::setprecision(13) << geo_point.latitude << " " << geo_point.longitude << std::endl;
    }
  }

  void ll2cloud()
  {
    pcl::PointCloud<PointType> cloud;
    std::ifstream fs;

    fs.open(csv_name);
    if (!fs.is_open())
    {
      std::cout << "can't open " << csv_name << std::endl;
    }
    if (!fs)
    {
      std::cout << "error: " << csv_name << std::endl;
    }
    std::string line;
    std::vector<double> poseV(4);
    while (getline(fs, line))
    {
      std::stringstream ss;
      ss << line;
      ss >> poseV[0] >> poseV[1] >> poseV[2] >> poseV[3];
      geographic_msgs::GeoPoint geo_point;
      geodesy::UTMPoint utm;
      geo_point.longitude = poseV[0];
      geo_point.latitude = poseV[1];
      geo_point.altitude = poseV[2];
      geodesy::fromMsg(geo_point, utm);

      PointType pt;
      pt.x = utm.easting + offset.easting - mine_orig.easting;
      pt.y = utm.northing + offset.northing - mine_orig.northing;
      pt.z = utm.altitude + offset.altitude - mine_orig.altitude;
      cloud.push_back(pt);
    }
    fs.close();

    pcl::io::savePCDFile(pcd_name, cloud);
  }
  void xy2cloud()
  {
    pcl::PointCloud<PointType> cloud;
    std::ifstream fs;
    std::string csvname = "data.csv", pcdname = "data.pcd";
    fs.open(csvname);
    if (!fs.is_open())
    {
      std::cout << "can't open " << csvname << std::endl;
    }
    if (!fs)
    {
      std::cout << "error: " << csvname << std::endl;
    }
    std::string line;
    std::vector<double> poseV(4);
    while (getline(fs, line))
    {
      std::stringstream ss;
      ss << line;
      ss >> poseV[0] >> poseV[1];

      PointType pt;
      pt.x = poseV[0];
      pt.y = poseV[1];
      pt.z = 33;
      cloud.push_back(pt);
    }
    fs.close();

    pcl::io::savePCDFile(pcdname, cloud);
    std::cout << "data: " << cloud.size() << std::endl;
  }
};
#endif
