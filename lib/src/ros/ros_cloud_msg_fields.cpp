
#include <iostream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

void printPointCloud2Fields(const sensor_msgs::PointCloud2 &cloud)
{
  std::cout << "PointCloud2 has " << cloud.fields.size() << " fields:" << std::endl;
  for (const auto &field : cloud.fields)
  {
    std::cout << "  Name: " << field.name
              << ", Offset: " << field.offset
              << ", Datatype: " << static_cast<int>(field.datatype)
              << ", Count: " << field.count << std::endl;
  }

  /*
  field.datatype: one of the constants in sensor_msgs/PointField:
  uint8 INT8    = 1;  // signed 8-bit integer
  uint8 UINT8   = 2;  // unsigned 8-bit integer
  uint8 INT16   = 3;  // signed 16-bit integer
  uint8 UINT16  = 4;  // unsigned 16-bit integer
  uint8 INT32   = 5;  // signed 32-bit integer
  uint8 UINT32  = 6;  // unsigned 32-bit integer
  uint8 FLOAT32 = 7;  // 32-bit floating point (IEEE 754)
  uint8 FLOAT64 = 8;  // 64-bit floating point (double)

  sensor_msgs::PointField::FLOAT32
*/
}


int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <bag_file> <cloud_topic>" << std::endl;
    return 1;
  }

  std::string bag_file    = argv[1];
  std::string cloud_topic = argv[2];

  rosbag::Bag bag;
  try
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException &e)
  {
    std::cerr << "Error opening bag file: " << e.what() << std::endl;
    return 1;
  }

  rosbag::View view(bag, rosbag::TopicQuery(cloud_topic));

  for (const rosbag::MessageInstance &message : view)
  {
    if (message.getTopic() != cloud_topic)
    { continue; }
    auto cloud_message = message.instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_message)
    {
      continue;
    }

    printPointCloud2Fields(*cloud_message);
    break; // only print the first one
  }
}