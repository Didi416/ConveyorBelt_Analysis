// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
// #include <sensor_msgs/point_cloud2_iterator.hpp>
// // #include <pcl_ros/point_cloud.hpp>
// // #include <pcl/point_types.h>
// // #include <pcl/filters/boost.h>
// // #include <pcl/filters/crop_box.h>
// // #include <pcl_conversions/pcl_conversions.h>

// class Filter : public rclcpp::Node {
//   public:
//     Filter() : Node("filter_node"){
//       tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//       tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      
//       depthSub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/depth/color/points", 10,
//                       std::bind(&Filter::depthCallback, this, std::placeholders::_1));

//       filterPub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 1);

//       RCLCPP_INFO(this->get_logger(), "Depth Image Visualizer Node started.");
//     };
  
//     void depthCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
//       try {
//             // Lookup the transformation from depth optical frame to base frame
//             RCLCPP_INFO(this->get_logger(), "Test01");
//             // geometry_msgs::msg::TransformStamped transform_stamped =
//             //       tf_buffer_->lookupTransform("base", "camera_depth_optical_frame", rclcpp::Time(0), std::chrono::seconds(1));

//             // // Transform the point cloud
//             // sensor_msgs::msg::PointCloud2::SharedPtr transformed_cloud;
//             // RCLCPP_INFO(this->get_logger(), "Test02");
//             // tf2::doTransform(*msg, *transformed_cloud, transform_stamped);
//             // RCLCPP_INFO(this->get_logger(), "Test0");
//             mapCoordinates(msg);

//             // RCLCPP_INFO(this->get_logger(), "Point cloud successfully transformed to base frame.");
//         } catch (const tf2::TransformException &e) {
//             RCLCPP_WARN(this->get_logger(), "Could not transform: %s", e.what());
//         }
//     }

//     void mapCoordinates(sensor_msgs::msg::PointCloud2::SharedPtr transformedCloud){
//       RCLCPP_INFO(this->get_logger(), "Test11");
//       auto filteredCloud = filterCoordinates(transformedCloud);
      

//     }
//     // sensor_msgs::msg::PointCloud2::SharedPtr filterCoordinates(sensor_msgs::msg::PointCloud2::SharedPtr transformedCloud){
//     //   // Convert the sensor_msgs::msg::PointCloud2 to pcl::PointCloud
//     //   pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
//     //   pcl::fromROSMsg(*transformedCloud, *pclCloud);
//     //   // Define the region of interest (ROI) as a bounding box
//     //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
//     //   pcl::CropBox<pcl::PointXYZ> crop_box_filter;
//     //   // Set the ROI's min and max values (bounding box coordinates)
//     //   crop_box_filter.setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0));  // Min x, y, z values
//     //   crop_box_filter.setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0));    // Max x, y, z values
//     //   // Apply the crop box filter
//     //   crop_box_filter.setInputCloud(pclCloud);
//     //   crop_box_filter.filter(*cloud_filtered);
//     //   // Convert back to sensor_msgs::PointCloud2 for publishing
//     //   sensor_msgs::msg::PointCloud2::SharedPtr filteredCloud;
//     //   pcl::toROSMsg(*cloud_filtered, *filteredCloud);
//     //   // Set the header for the output point cloud message
//     //   // filteredCloud.header = transformedCloud.header;
//     //   // Publish the filtered point cloud
//     //   filterPub_->publish(*filteredCloud);
//     //   return filteredCloud;
//     // }

//     sensor_msgs::msg::PointCloud2 filterCoordinates(sensor_msgs::msg::PointCloud2::SharedPtr transformedCloud){
//       RCLCPP_INFO(this->get_logger(), "Test12");
//       // Create an iterator for the PointCloud2 message
//       sensor_msgs::PointCloud2Iterator<float> iter_x(*transformedCloud, "x");
//       sensor_msgs::PointCloud2Iterator<float> iter_y(*transformedCloud, "y");
//       sensor_msgs::PointCloud2Iterator<float> iter_z(*transformedCloud, "z");
//       RCLCPP_INFO(this->get_logger(), "Test13");
//       // Vector to store filtered points
//       std::vector<float> filtered_x;
//       std::vector<float> filtered_y;
//       std::vector<float> filtered_z;

//       // Iterate over the points in the point cloud
//       RCLCPP_INFO(this->get_logger(), "Test1");
//       for (size_t i = 0; i < transformedCloud->width * transformedCloud->height; ++i) {
//           // Access current point's coordinates
//           float x = *iter_x;
//           float y = *iter_y;
//           float z = *iter_z;
//           RCLCPP_INFO(this->get_logger(), "Test2");
//           // Check if the point is within the specified ranges
//           if (x >= x_min_ && x <= x_max_ && 
//               y >= y_min_ && y <= y_max_ && 
//               z >= z_min_ && z <= z_max_) {
//               // If within range, store the point
//               filtered_x.push_back(x);
//               filtered_y.push_back(y);
//               filtered_z.push_back(z);
//           }

//           // Increment iterators for next point
//           ++iter_x;
//           ++iter_y;
//           ++iter_z;
//       }
//       RCLCPP_INFO(this->get_logger(), "Test3");

//       // Now filtered_x, filtered_y, filtered_z contain the coordinates of the filtered points.
//       // You can use these to create a new PointCloud2 message or process them further.

//       sensor_msgs::msg::PointCloud2 filtered_cloud;
//       filtered_cloud.header = transformedCloud->header;
//       filtered_cloud.width = filtered_x.size();
//       filtered_cloud.height = 1; // for simplicity, treating as a single row
//       filtered_cloud.fields = transformedCloud->fields;
//       filtered_cloud.is_bigendian = transformedCloud->is_bigendian;
//       filtered_cloud.point_step = transformedCloud->point_step;
//       filtered_cloud.row_step = transformedCloud->point_step * filtered_x.size();
//       filtered_cloud.is_dense = transformedCloud->is_dense;

//       // Allocate space for the filtered data
//       filtered_cloud.data.resize(filtered_x.size() * transformedCloud->point_step);
//       RCLCPP_INFO(this->get_logger(), "Test4");
//       // Copy filtered points into the filtered cloud data
//       for (size_t i = 0; i < filtered_x.size(); ++i) {
//           *reinterpret_cast<float*>(&filtered_cloud.data[i * transformedCloud->point_step]) = filtered_x[i];
//           *reinterpret_cast<float*>(&filtered_cloud.data[i * transformedCloud->point_step + 4]) = filtered_y[i];
//           *reinterpret_cast<float*>(&filtered_cloud.data[i * transformedCloud->point_step + 8]) = filtered_z[i];
//       }

//       // You now have a filtered PointCloud2 message ready for further use, like publishing  
//       filterPub_->publish(filtered_cloud);

//       return filtered_cloud;
//     }

//   private:
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depthSub_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filterPub_;

//     std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//     std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//     const float x_min_ = -1.5;  // Start depth (forward distance)
//     const float x_max_ = 4.0;  // End depth
//     const float y_min_ = 0;       // Slice start (side limit)
//     const float y_max_ = 0.05;    // Slice end (side limit)
//     const float z_min_ = 0.5;     // Min height
//     const float z_max_ = 2;       // Max height
// };

// int main(int argc, char** argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<Filter>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// };

