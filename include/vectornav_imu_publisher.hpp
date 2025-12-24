#ifndef VECTORNAV_IMU_PUBLISHER_HPP_
#define VECTORNAV_IMU_PUBLISHER_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "vectornav/Interface/Sensor.hpp"
#include "vectornav/TemplateLibrary/Vector.hpp"

class VectorNavImuPublisher : public rclcpp::Node
{
public:
    VectorNavImuPublisher();
    ~VectorNavImuPublisher();

    bool initialize();
    void run();

private:
    void publishImu();
    bool connectToSensor();
    void configureSensor();

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // VectorNav sensor
    VN::Sensor sensor_;

    // Parameters
    std::string port_;
    int baud_rate_;
    std::string frame_id_;
    double publish_rate_;

    // Sensor configuration
    bool sensor_connected_;
    bool sensor_configured_;

    // Utility functions
    geometry_msgs::msg::Vector3 toVector3(const VN::Vec3f& vec);
    geometry_msgs::msg::Vector3 toVector3(const VN::Vec3d& vec);
    geometry_msgs::msg::Quaternion toQuaternion(const VN::Quat& quat);
};

#endif  // VECTORNAV_IMU_PUBLISHER_HPP_