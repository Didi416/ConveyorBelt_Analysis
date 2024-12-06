#include "gazebo_msgs/srv/delete_entity.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ConveyorDespawner : public rclcpp::Node {
public:
    ConveyorDespawner() : Node("conveyor_despawner") {
        // Create clients for delete services
        delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
        model_states = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 10, std::bind(&ConveyorDespawner::statesCallback, this, std::placeholders::_1));
    }

private:
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr model_states;

    void statesCallback(const gazebo_msgs::msg::ModelStates::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(), "Size: %2ld", msg->name.size());
        for (long unsigned int i=0; i<msg->name.size(); i++){
            // RCLCPP_INFO(this->get_logger(), "I: %2ld", i);
            if(msg->name.at(i) == "ground_plane" || msg->name.at(i) == "conveyor_belt" || msg->name.at(i) == "camera_robot"){
                continue;
            }
            // RCLCPP_INFO(this->get_logger(), "Pose: %2f", msg->pose.at(i).position.z);
            if (msg->pose.at(i).position.y > 9){
                
                // RCLCPP_INFO(this->get_logger(), "Object below 0.5");
                delete_object(msg->name.at(i));
            }
        }
    }
    void delete_object(const std::string& name) {
        if (!delete_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Service /delete_entity not available");
            return;
        }

        auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        request->name = name;

        delete_client_->async_send_request(request, [name](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture future) {
            // try {
                auto response = future.get();
            //     if (!response->success) {
            //         RCLCPP_INFO(rclcpp::get_logger("DeleteEntity"), "Successfully deleted: %s", name.c_str());
            //     } else {
            //         RCLCPP_ERROR(rclcpp::get_logger("DeleteEntity"), "Failed to delete: %s, reason: %s",
            //                     name.c_str(), response->status_message.c_str());
            //     }
            // } catch (const std::exception& e) {
            //     RCLCPP_ERROR(rclcpp::get_logger("DeleteEntity"), "Service call failed: %s", e.what());
            // }
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
