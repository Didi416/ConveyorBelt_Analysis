#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/delete_entity.hpp>
#include "conveyor_belt_msgs/srv/batch_delete_entities.hpp" //Custom Service

using namespace std::chrono_literals;

class BatchDeleteService : public rclcpp::Node {
public:
    BatchDeleteService() : Node("batch_delete_service") {
        // Create the batch delete service
        service_ = this->create_service<conveyor_belt_msgs::srv::BatchDeleteEntities>(
            "batch_delete_entities",
            std::bind(&BatchDeleteService::handleBatchDelete, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Create the client for Gazebo's delete_entity service
        delete_client_ = this->create_client<gazebo_msgs::srv::DeleteEntity>("/gazebo/delete_entity");
        RCLCPP_INFO(this->get_logger(), "Batch Delete Service is ready.");
    }

private:
    rclcpp::Service<conveyor_belt_msgs::srv::BatchDeleteEntities>::SharedPtr service_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client_;

    void handleBatchDelete(
        const std::shared_ptr<conveyor_belt_msgs::srv::BatchDeleteEntities::Request> request,
        std::shared_ptr<conveyor_belt_msgs::srv::BatchDeleteEntities::Response> response
    ) {
        bool success = true;
        std::ostringstream message;

        for (const auto &model_name : request->names) {
            // Prepare a delete request
            auto delete_request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
            delete_request->name = model_name;

            // Call the delete service
            auto future = delete_client_->async_send_request(delete_request);

            // Wait for the service to respond
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
                rclcpp::FutureReturnCode::SUCCESS) {
                auto result = future.get();
                if (result->success) {
                    RCLCPP_INFO(this->get_logger(), "Successfully deleted: %s", model_name.c_str());
                } else {
                    success = false;
                    message << "Failed to delete " << model_name << ": " << result->status_message << "\n";
                    RCLCPP_ERROR(this->get_logger(), "Failed to delete: %s", model_name.c_str());
                }
            } else {
                success = false;
                message << "Failed to communicate with Gazebo for model: " << model_name << "\n";
                RCLCPP_ERROR(this->get_logger(), "Service call failed for: %s", model_name.c_str());
            }
        }

        // Set the response
        response->success = success;
        response->message = message.str();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatchDeleteService>());
    rclcpp::shutdown();
    return 0;
}
