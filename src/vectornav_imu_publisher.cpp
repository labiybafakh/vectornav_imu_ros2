#include "vectornav_imu_publisher.hpp"
#include <chrono>

using namespace std::chrono_literals;

VectorNavImuPublisher::VectorNavImuPublisher()
    : Node("vectornav_imu_publisher"), sensor_connected_(false), sensor_configured_(false)
{
    // Declare parameters
    declare_parameter<std::string>("port", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 115200);
    declare_parameter<std::string>("frame_id", "imu_link");
    declare_parameter<double>("publish_rate", 200.0);

    // Get parameters
    port_ = get_parameter("port").as_string();
    baud_rate_ = get_parameter("baud_rate").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();

    // Create publisher
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    // Create timer for publishing
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
    timer_ = create_wall_timer(timer_period, std::bind(&VectorNavImuPublisher::publishImu, this));

    RCLCPP_INFO(get_logger(), "VectorNav IMU Publisher initialized");
    RCLCPP_INFO(get_logger(), "Port: %s, Baud: %d, Frame: %s, Rate: %.1f Hz",
                port_.c_str(), baud_rate_, frame_id_.c_str(), publish_rate_);
}

VectorNavImuPublisher::~VectorNavImuPublisher()
{
    if (sensor_connected_) {
        sensor_.disconnect();
        RCLCPP_INFO(get_logger(), "Disconnected from VectorNav sensor");
    }
}

bool VectorNavImuPublisher::initialize()
{
    if (!connectToSensor()) {
        return false;
    }

    configureSensor();
    return true;
}

bool VectorNavImuPublisher::connectToSensor()
{
    RCLCPP_INFO(get_logger(), "Connecting to VectorNav sensor on %s...", port_.c_str());

    try {
        VN::Error error = sensor_.autoConnect(port_);
        if (error != VN::Error::None) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to sensor: %s", VN::errorCodeToString(error));
            return false;
        }

        // Change baud rate if needed
        if (baud_rate_ != static_cast<int>(sensor_.connectedBaudRate().value())) {
            error = sensor_.changeBaudRate(static_cast<VN::Sensor::BaudRate>(baud_rate_));
            if (error != VN::Error::None) {
                RCLCPP_WARN(get_logger(), "Failed to change baud rate: %s", VN::errorCodeToString(error));
            }
        }

        sensor_connected_ = true;
        RCLCPP_INFO(get_logger(), "Successfully connected to VectorNav sensor");

        // Read sensor info
        VN::Registers::System::Model modelRegister;
        sensor_.readRegister(&modelRegister);
        std::string modelNumber = modelRegister.model;

        VN::Registers::System::FwVer firmwareRegister;
        sensor_.readRegister(&firmwareRegister);
        std::string firmwareVersion = firmwareRegister.fwVer;

        VN::Registers::System::Serial serialRegister;
        sensor_.readRegister(&serialRegister);
        uint32_t serialNumber = serialRegister.serialNum;

        RCLCPP_INFO(get_logger(), "Model: %s, Firmware: %s, Serial: %u",
                    modelNumber.c_str(), firmwareVersion.c_str(), serialNumber);

        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception while connecting: %s", e.what());
        return false;
    }
}

void VectorNavImuPublisher::configureSensor()
{
    if (!sensor_connected_) {
        return;
    }

    try {
        using namespace VN::Registers::System;

        // First, disable ASCII async output
        AsyncOutputType asyncOutputType;
        asyncOutputType.ador = AsyncOutputType::Ador::OFF;
        asyncOutputType.serialPort = AsyncOutputType::SerialPort::Serial1;
        VN::Error error = sensor_.writeRegister(&asyncOutputType);
        if (error != VN::Error::None) {
            RCLCPP_WARN(get_logger(), "Failed to disable ASCII output: %s", VN::errorCodeToString(error));
        } else {
            RCLCPP_INFO(get_logger(), "Disabled ASCII async output");
        }

        // Configure binary output for IMU data (using new SDK API)
        BinaryOutput1 binaryOutput;
        binaryOutput.asyncMode.emplace();  // Initialize the optional asyncMode
        binaryOutput.asyncMode->serial1 = true;  // Enable output on serial port 1
        binaryOutput.rateDivisor = static_cast<uint16_t>(800 / publish_rate_); // 800Hz base rate

        // Enable specific measurements using boolean flags
        binaryOutput.imu.accel = true;           // Linear acceleration
        binaryOutput.imu.angularRate = true;     // Angular velocity (gyro)
        binaryOutput.attitude.quaternion = true; // Orientation quaternion

        error = sensor_.writeRegister(&binaryOutput);
        if (error != VN::Error::None) {
            RCLCPP_WARN(get_logger(), "Failed to configure binary output: %s", VN::errorCodeToString(error));
        } else {
            sensor_configured_ = true;
            RCLCPP_INFO(get_logger(), "Sensor configured for binary IMU data output");
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception during configuration: %s", e.what());
    }
}

void VectorNavImuPublisher::publishImu()
{
    if (!sensor_connected_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Sensor not connected");
        return;
    }

    try {
        auto cd = sensor_.getNextMeasurement();
        if (!cd) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No measurement available");
            return;
        }

        // Skip ASCII packets, only process binary
        if (std::holds_alternative<VN::AsciiHeader>(cd->header())) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received ASCII packet (skipping)");
            return;
        }

        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = now();
        imu_msg.header.frame_id = frame_id_;

        // Angular velocity (gyroscope)
        if (cd->imu.angularRate.has_value()) {
            imu_msg.angular_velocity = toVector3(cd->imu.angularRate.value());
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No angular rate data");
        }

        // Linear acceleration
        if (cd->imu.accel.has_value()) {
            imu_msg.linear_acceleration = toVector3(cd->imu.accel.value());
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No acceleration data");
        }

        // Orientation (quaternion)
        if (cd->attitude.quaternion.has_value()) {
            imu_msg.orientation = toQuaternion(cd->attitude.quaternion.value());
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No quaternion data");
        }

        // Set covariance matrices (replace with actual values if known)
        for (int i = 0; i < 9; ++i) {
            imu_msg.orientation_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
            imu_msg.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
            imu_msg.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.01 : 0.0;
        }

        imu_publisher_->publish(imu_msg);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception in publishImu: %s", e.what());
    }
}

void VectorNavImuPublisher::run()
{
    rclcpp::spin(shared_from_this());
}

geometry_msgs::msg::Vector3 VectorNavImuPublisher::toVector3(const VN::Vec3f& vec)
{
    geometry_msgs::msg::Vector3 result;
    result.x = vec[0];
    result.y = vec[1];
    result.z = vec[2];
    return result;
}

geometry_msgs::msg::Vector3 VectorNavImuPublisher::toVector3(const VN::Vec3d& vec)
{
    geometry_msgs::msg::Vector3 result;
    result.x = vec[0];
    result.y = vec[1];
    result.z = vec[2];
    return result;
}

geometry_msgs::msg::Quaternion VectorNavImuPublisher::toQuaternion(const VN::Quat& quat)
{
    geometry_msgs::msg::Quaternion result;
    result.x = quat.vector[0];
    result.y = quat.vector[1];
    result.z = quat.vector[2];
    result.w = quat.scalar;
    return result;
}