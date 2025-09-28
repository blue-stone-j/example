// Extract images from a ROS bag file

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

#include <iostream>
#include <iomanip>
#include <sstream>

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    std::cerr << "Usage: " << argv[0] << " <bag_file> <image_topic>" << std::endl;
    return 1;
  }

  std::string bag_file    = argv[1];
  std::string image_topic = argv[2];
  std::string out_folder  = "extracted_images";

  std::filesystem::create_directory(out_folder);

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

  rosbag::View view(bag, rosbag::TopicQuery(image_topic));
  std::size_t count = 0;

  for (const rosbag::MessageInstance &message : view)
  {
    if (message.getTopic() != image_topic)
    { continue; }
    auto image_message = message.instantiate<sensor_msgs::Image>();
    if (!image_message)
    {
      continue;
    }

    try
    {
      cv::Mat image = cv_bridge::toCvCopy(image_message, image_message->encoding)->image;
      std::ostringstream filename;
      filename << out_folder << "/" << std::fixed << std::setprecision(9)
               << image_message->header.stamp.toSec() << ".png";

      cv::imwrite(filename.str(), image);
      ++count;
    }
    catch (const cv_bridge::Exception &e)
    {
      std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    }
  }

  std::cout << "Saved " << count << " images to: " << out_folder << std::endl;
  bag.close();
  return 0;
}
