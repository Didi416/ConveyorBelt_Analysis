#include <chrono>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ConveyorDespawner : public rclcpp::Node {
public:
    ConveyorDespawner() : Node("conveyor_spawner") {
        // Create clients for spawn and delete services
        // spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
        model_states = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 10, std::bind(&ConveyorDespawner::statesCallback, this, std::placeholders::_1));
        // Timer to spawn objects periodically
        // spawn_timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&ConveyorSpawner::spawn_object, this));

        // Random generator setup
        // random_engine_ = std::default_random_engine(std::random_device{}());
        // shape_distribution_ = std::uniform_int_distribution<int>(0, 2); // Cube, Sphere, Cylinder (currently)
    }

private:
    // rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states;
    // rclcpp::TimerBase::SharedPtr spawn_timer_;

    // int object_counter_;
    // std::vector<std::string> spawned_objects_;

    // std::default_random_engine random_engine_;
    // std::uniform_int_distribution<int> shape_distribution_;

    // std::string get_random_shape() {
    //     switch (shape_distribution_(random_engine_)) {
    //         case 0: return "box";
    //         case 1: return "sphere";
    //         case 2: return "cylinder";
    //     }
    //     return "box"; // Default shape
    // }

    // void spawn_object() {
    //     if (!spawn_client_->wait_for_service(1s)) {
    //         RCLCPP_ERROR(this->get_logger(), "Spawn service not available.");
    //         return;
    //     }

    //     // Locate the package and load the urdf file
    //     std::string package_path = ament_index_cpp::get_package_share_directory("conveyor_belt");
    //     std::string sdf_path = package_path + "/urdf/" + get_random_shape() + ".urdf";

    //     std::ifstream sdf_file(sdf_path);
    //     if (!sdf_file.is_open()) {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to open SDF file: %s", sdf_path.c_str());
    //         return;
    //     }

    //     std::stringstream sdf_stream;
    //     sdf_stream << sdf_file.rdbuf();
    //     std::string sdf_content = sdf_stream.str();

    //     // Create and send spawn request
    //     auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    //     request->name = "object_" + std::to_string(object_counter_++);
    //     request->xml = sdf_content;
    //     request->robot_namespace = "/";
    //     request->initial_pose.position.x = 0.0; // Adjust position
    //     request->initial_pose.position.y = -1.5;
    //     request->initial_pose.position.z = 1.0;

    //     spawned_objects_.push_back(request->name);

    //     auto future = spawn_client_->async_send_request(request);

    //     // RCLCPP_INFO(this->get_logger(), "Spawned Test1");
    //     // try {
    //     // } catch (const std::exception& e) {
    //     //     RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    //     // }
    // }
    // void serviceCallback(rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture future){
    //     RCLCPP_INFO(this->get_logger(), "Spawned Test2");
    //     auto response = future.get();
    //     RCLCPP_INFO(this->get_logger(), "Spawned Test3");
    //     if (response->success) {
    //         RCLCPP_INFO(this->get_logger(), "Spawned Successfully");
    //         // spawned_objects_.push_back(name);
    //     } else {
    //         RCLCPP_ERROR(this->get_logger(), "Failed to spawn");
    //     }
    // }

    void statesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(), "Size: %2ld", msg->name.size());
        for (long unsigned int i=0; i<msg->name.size(); i++){
            // RCLCPP_INFO(this->get_logger(), "I: %2ld", i);
            if(msg->name.at(i) == "ground_plane" || msg->name.at(i) == "conveyor_belt" || msg->name.at(i) == "realsense_camera_0"){
                continue;
            }
            // RCLCPP_INFO(this->get_logger(), "Pose: %2f", msg->pose.at(i).position.z);
            if (msg->pose.at(i).position.z < 0.5){
                
                // RCLCPP_INFO(this->get_logger(), "Object below 0.5");
                delete_object(msg->name.at(i));
            }
        }
    }
    void delete_object(const std::string& name) {
        // if (!delete_client_->wait_for_service(std::chrono::seconds(1))) {
        //     RCLCPP_ERROR(this->get_logger(), "Delete service not available.");
        //     return;
        // }

        // auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        // request->name = name;

        // auto future = delete_client_->async_send_request(request);
        
        // // Wait for the future to complete
        // auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        
        // if (result == rclcpp::FutureReturnCode::SUCCESS) {
        //     auto response = future.get();
        //     if (response->success) {
        //         RCLCPP_INFO(this->get_logger(), "Successfully deleted: %s", name.c_str());
        //     } else {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to delete: %s, reason: %s", 
        //                     name.c_str(), response->status_message.c_str());
        //     }
        // } else if (result == rclcpp::FutureReturnCode::TIMEOUT) {
        //     RCLCPP_ERROR(this->get_logger(), "Delete service timed out for: %s", name.c_str());
        // } else if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
        //     RCLCPP_ERROR(this->get_logger(), "Delete service interrupted for: %s", name.c_str());
        // }

        if (!delete_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Service /delete_entity not available");
            return;
        }

        auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = name;

        delete_client_->async_send_request(request, [name](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(rclcpp::get_logger("DeleteEntity"), "Successfully deleted: %s", name.c_str());
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger("DeleteEntity"), "Failed to delete: %s, reason: %s",
                                name.c_str(), response->status_message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(rclcpp::get_logger("DeleteEntity"), "Service call failed: %s", e.what());
            }
        });
    }

    // void delete_object(const std::string& name) {
    //     if (!delete_client_->wait_for_service(1s)) {
    //         RCLCPP_ERROR(this->get_logger(), "Delete service not available.");
    //         return;
    //     }

    //     auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    //     request->name = name;

    //     auto future = delete_client_->async_send_request(request);
    //     RCLCPP_INFO(this->get_logger(), "Deleted object.");
    //     try {
    //         auto response = future.get();
    //         if (response->success) {
    //             RCLCPP_INFO(this->get_logger(), "Deleted: %s", name.c_str());
    //         } else {
    //             RCLCPP_ERROR(this->get_logger(), "Failed to delete: %s", name.c_str());
    //         }
    //     } catch (const std::exception& e) {
    //         RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    //     }
    // }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConveyorDespawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
