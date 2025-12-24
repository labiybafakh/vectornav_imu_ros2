#include "vectornav_imu_publisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VectorNavImuPublisher>();

    // Initialize the sensor connection
    if (!node->initialize()) {
        RCLCPP_ERROR(rclcpp::get_logger("vectornav_imu_node"),
                     "Failed to initialize VectorNav sensor. Exiting...");
        rclcpp::shutdown();
        return -1;
    }

    RCLCPP_INFO(rclcpp::get_logger("vectornav_imu_node"),
                "VectorNav IMU node started successfully");

    try {
        node->run();
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("vectornav_imu_node"),
                     "Exception in main loop: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}