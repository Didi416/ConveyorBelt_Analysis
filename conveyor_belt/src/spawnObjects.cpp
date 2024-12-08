#include <chrono>
#include <random>
#include <fstream>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ConveyorSpawner : public rclcpp::Node {
public:
    ConveyorSpawner()
        : Node("conveyor_spawner"), object_counter_(0) {
        // Create clients for spawn services
        spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        // Timer to spawn objects periodically
        spawn_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ConveyorSpawner::spawn_object, this));

        // Random generator setup
        random_engine_ = std::default_random_engine(std::random_device{}());
        shape_distribution_ = std::uniform_int_distribution<int>(0, 2); // Cube, Sphere, Cylinder (currently)
    }

private:
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::TimerBase::SharedPtr spawn_timer_;

    int object_counter_;

    std::default_random_engine random_engine_;
    std::uniform_int_distribution<int> shape_distribution_;

    std::string get_random_shape() {
        switch (shape_distribution_(random_engine_)) {
            case 0: return "box";
            case 1: return "sphere";
            case 2: return "cylinder";
        }
        return "box"; // Default shape
    }

    void spawn_object() {
        if (!spawn_client_->wait_for_service(1s)) {
            RCLCPP_ERROR(this->get_logger(), "Spawn service not available.");
            return;
        }

        // Locate the package and load the urdf file
        std::string package_path = ament_index_cpp::get_package_share_directory("conveyor_belt");
        std::string sdf_path = package_path + "/urdf/" + get_random_shape() + ".urdf";

        std::ifstream sdf_file(sdf_path);
        if (!sdf_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open SDF file: %s", sdf_path.c_str());
            return;
        }

        std::stringstream sdf_stream;
        sdf_stream << sdf_file.rdbuf();
        std::string sdf_content = sdf_stream.str();

        // Create and send spawn request
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "object_" + std::to_string(object_counter_++);
        request->xml = sdf_content;
        request->robot_namespace = "/";
        request->initial_pose.position.x = 0.0; // Adjust position
        request->initial_pose.position.y = -9;
        request->initial_pose.position.z = 2.0;

        auto future = spawn_client_->async_send_request(request);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConveyorSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
