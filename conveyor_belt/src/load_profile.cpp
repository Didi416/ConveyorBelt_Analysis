#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/opencv.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"



// INITIALIZE ConveyorLoadProfile NODE

// SET depth_subscriber TO subscribe to "/camera/depth/image_rect_raw"

// DEFINE CALLBACK FUNCTION depth_callback(depth_image_msg)
//     CONVERT depth_image_msg TO depth_image in OpenCV format
    
//     DEFINE height_profile as empty list
    
//     FOR each section in the conveyor's width
//         INITIALIZE section_height TO 0
//         INITIALIZE valid_points TO 0
        
//         FOR each pixel in section
//             GET depth_value FROM depth_image at pixel location
            
//             IF depth_value > 0
//                 section_height += depth_value
//                 valid_points += 1
//             END IF
//         END FOR
        
//         IF valid_points > 0
//             AVERAGE section_height by dividing by valid_points
//         END IF
        
//         APPEND section_height TO height_profile
//     END FOR

//     CALL plot_load_profile(height_profile)
// END CALLBACK

// DEFINE FUNCTION plot_load_profile(height_profile)
//     INITIALIZE empty image profile_image FOR visualization
    
//     FOR each section in height_profile
//         MAP section height TO y-axis position
//         DRAW vertical line on profile_image at section position
//     END FOR
    
//     DISPLAY profile_image
// END FUNCTION

// LOOP rclcpp::spin UNTIL shutdown
