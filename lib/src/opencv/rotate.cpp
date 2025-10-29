
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
  cv::Mat src = cv::imread("image.png");
  cv::Mat rotated;

  // 90 degrees counterclockwise
  cv::rotate(src, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
  cv::rotate(src, rotated, cv::ROTATE_90_CLOCKWISE);

  // Alternatively, manual method for 90 degrees clockwise rotation
  // Step 1: transpose the image (swap rows and columns)
  cv::transpose(src, rotated);
  // Step 2: flip horizontally to achieve clockwise rotation
  cv::flip(rotated, rotated, 1); // flip around y-axis

  // Alternatively, manual method for 90 degrees counterclockwise rotation
  cv::transpose(src, rotated);
  cv::flip(rotated, rotated, 0); // flip vertically

  return 0;
}