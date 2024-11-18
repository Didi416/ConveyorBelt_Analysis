#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class ColourMap : public rclcpp::Node {
  public:
    ColourMap()  : Node("conveyor_analysis"){
      depthSub_ = this->create_subscription<sensor_msgs::msg::Image>("/depth/image_raw", rclcpp::SensorDataQoS(),
                      std::bind(&ColourMap::depthImageCallback, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "Depth Image Visualizer Node started.");
    };
  
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
      try{
          // Convert ROS image message to OpenCV image
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

          // Normalize depth image for better visualization
          cv::Mat normalized_depth;
          cv::normalize(cv_ptr->image, normalized_depth, 0, 255, cv::NORM_MINMAX);

          // Invert depth values so that bottom is low (blue) and top is high (red)
          cv::Mat inverted_depth = 255 - normalized_depth;

          // Convert the inverted depth image to an 8-bit image
          inverted_depth.convertTo(inverted_depth, CV_8U);

          // Apply a colormap (e.g., JET)
          cv::Mat color_mapped_depth;
          cv::applyColorMap(inverted_depth, color_mapped_depth, cv::COLORMAP_JET);

          // Display the colorized depth image
          cv::imshow("Depth Image (Colorized)", color_mapped_depth);
          cv::waitKey(1);
        }
        catch (const cv_bridge::Exception &e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<ColourMap>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};
