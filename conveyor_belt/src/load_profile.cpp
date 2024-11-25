// #include <rclcpp/rclcpp.hpp>
// #include <image_transport/image_transport.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>
// #include <geometry_msgs/msg/point.hpp>
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// struct CameraInfo{
//   double fx; // Focal length in x
//   double fy; // Focal length in y
//   double cx; // Principal point x
//   double cy; // Principal point y
// };

// class ColourMap : public rclcpp::Node {
//   public:
//     ColourMap()  : Node("conveyor_analysis"){
//       depthSub_ = this->create_subscription<sensor_msgs::msg::Image>("/depth/image_raw", 10, std::bind(&ColourMap::depthImageCallback, this, std::placeholders::_1));
//       infoSub_ = this->create_subscription<sensor_msgs::msg::Image>("/depth/camera_info", 10, std::bind(&ColourMap::infoCallback, this, std::placeholders::_1));

//       RCLCPP_INFO(this->get_logger(), "Depth Image Visualizer Node started.");
//     };

//     void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
//       // extract camera info for global transforms
//       CameraInfo info;
//       info.fx = msg->k[0];
//       info.fy = msg->k[4];
//       info.cx = msg->k[2];
//       info.cy = msg->k[5];
//       RCLCPP_INFO(this->get_logger(), "Camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", info.fx, info.fy, info.cx, info.cy);
//     }
  
//     void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
//       try{
//           // Convert ROS image message to OpenCV image
//           cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
//           cv::Mat depth_image = cv_ptr->image;

//           // Reproject depth image to point cloud (3D points)
//           cv::Mat point_cloud(depth_image.rows, depth_image.cols, CV_32FC3);  // Stores (x, y, z) points
//           cv::reprojectImageTo3D(depth_image, point_cloud, cv::Mat::eye(3, 3, CV_64F), true);

//           std::vector<double> transformed_z_values;
//           for (int i = 0; i < point_cloud.rows; i++) {
//             for (int j = 0; j < point_cloud.cols; j++) {
//               cv::Vec3f& point = point_cloud.at<cv::Vec3f>(i, j);
                
//               geometry_msgs::msg::Point camera_point;
//               camera_point.x = point[0];
//               camera_point.y = point[1];
//               camera_point.z = point[2];

//               // Transform the point from camera frame to global frame
//               geometry_msgs::msg::Point global_point = transformPoint(camera_point);
//               transformed_z_values.push_back(global_point.z); // Store the z-value
//             }
//           }

//           // Convert z-values to a grayscale image
//           cv::Mat z_image(depth_image.rows, depth_image.cols, CV_32F);
//           for (int i = 0; i < transformed_z_values.size(); i++) {
//               z_image.at<float>(i / depth_image.cols, i % depth_image.cols) = transformed_z_values[i];
//           }

//           // Normalize z-values to the range [0, 255] for color mapping
//           cv::Mat z_image_normalized;
//           cv::normalize(z_image, z_image_normalized, 0, 255, cv::NORM_MINMAX);

//           // Convert to 8-bit format for applyColorMap
//           cv::Mat z_image_8bit;
//           z_image_normalized.convertTo(z_image_8bit, CV_8U);

//           // Apply the color map (using a "JET" colormap, or you can choose another)
//           cv::Mat color_mapped_image;
//           cv::applyColorMap(z_image_8bit, color_mapped_image, cv::COLORMAP_JET);

//           // Step 7: Display the color-mapped image
//           cv::imshow("Color Mapped Image", color_mapped_image);
//           cv::waitKey(1);
//         }
//         catch (const cv_bridge::Exception &e){
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//         }
//     }

//     geometry_msgs::msg::Point transformPoint(geometry_msgs::msg::Point point){
//         geometry_msgs::msg::Point transformedPoint;
//         //transfer position and orientation data to tf2 data type format
//         tf2::Vector3 globalPoint(0,0,0); //
//         tf2::Quaternion q(0,0,0,1);

//         tf2::Transform transform(q,globalPoint); //calculate transform of car pose in global frame
//         tf2::Vector3 vecPoint(point.x, point.y, point.z); //create tf2 data type point to be transformed
//         tf2::Vector3 transformedVecPoint = transform * vecPoint; //apply transform to point
//         //assign values of x,y,z to geometry_msgs/msg/Point
//         transformedPoint.x = transformedVecPoint.x();
//         transformedPoint.y = transformedVecPoint.y();
//         transformedPoint.z = transformedVecPoint.z();
//         // RCLCPP_INFO(this->get_logger(), "GOAL PUBLISHED: %.2f, %.2f", transformedPoint.x, transformedPoint.y);

//         return transformedPoint; //return transformed point
//     }
//   private:
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr infoSub_;
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);

//   auto node = std::make_shared<ColourMap>();

//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// };
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
      depthSub_ = this->create_subscription<sensor_msgs::msg::Image>("/infra1/image_raw", 10,
                      std::bind(&ColourMap::depthImageCallback, this, std::placeholders::_1));

      imagePub_ = this->create_publisher<sensor_msgs::msg::Image>("/global_heat_map", 1);

      RCLCPP_INFO(this->get_logger(), "Depth Image Visualizer Node started.");
    };
  
    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
      try{
          // Convert ROS image message to OpenCV image
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

          // Normalize depth image for better visualization
          cv::Mat normalized_depth;
          cv::normalize(cv_ptr->image, normalized_depth, 0, 255, cv::NORM_MINMAX, CV_8UC1);

          // Invert depth values so that bottom is low (blue) and top is high (red)
          // cv::Mat inverted_depth = 255 - normalized_depth;

          // // Convert the inverted depth image to an 8-bit image
          // inverted_depth.convertTo(inverted_depth, CV_8U);

          // Apply a colormap
          cv::Mat color_mapped_depth;
          cv::applyColorMap(normalized_depth, color_mapped_depth, cv::COLORMAP_JET);

          // Display the colorized depth image
          // cv::imshow("Depth Image (Colorized)", color_mapped_depth);
          // cv::waitKey(1);
          // Convert the OpenCV image (cv::Mat) to a ROS Image message
          std_msgs::msg::Header header;  // Create an empty header
          header.stamp = this->get_clock()->now();  // Use the current time
          header.frame_id = "camera_frame";  // Set the appropriate frame ID

          // Convert cv::Mat to ROS Image message
          auto color_image_msg = cv_bridge::CvImage(header, "bgr8", color_mapped_depth).toImageMsg();

          // Publish the message
          imagePub_->publish(*color_image_msg);

        }
        catch (const cv_bridge::Exception &e){
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthSub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<ColourMap>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
};