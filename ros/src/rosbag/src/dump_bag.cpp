#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/compression.h>
#include <boost/foreach.hpp>

int main()
{
  const std::string input_bag_path  = "input.bag";
  const std::string output_bag_path = "output_compressed.bag";

  // Open input bag
  rosbag::Bag in_bag;
  in_bag.open(input_bag_path, rosbag::bagmode::Read);

  // Read all messages
  rosbag::View view(in_bag);

  // Open output bag with LZ4 compression
  rosbag::Bag out_bag;
  out_bag.open(output_bag_path, rosbag::bagmode::Write, rosbag::CompressionType::LZ4); // or BZ2

  // Copy each message
  for (const rosbag::MessageInstance &m : view)
  {
    out_bag.write(m.getTopic(), m.getTime(), m);
  }

  in_bag.close();
  out_bag.close();

  return 0;
}
