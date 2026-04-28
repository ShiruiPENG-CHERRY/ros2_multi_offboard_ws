#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class MultiOffboardControl : public rclcpp::Node
{
public:
    MultiOffboardControl() : Node("multi_offboard_control")
    {
        // === UAV0 /fmu/ ===
        offboard_control_mode_publisher_uav0_ = this->create_publisher<OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_uav0_ = this->create_publisher<TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_uav0_ = this->create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        // === UAV1 /px4_1/fmu/ ===
        offboard_control_mode_publisher_uav1_ = this->create_publisher<OffboardControlMode>(
            "/px4_1/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_uav1_ = this->create_publisher<TrajectorySetpoint>(
            "/px4_1/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_uav1_ = this->create_publisher<VehicleCommand>(
            "/px4_1/fmu/in/vehicle_command", 10);

        offboard_setpoint_counter_ = 0;
        time_start_ = this->get_clock()->now();

        // 100ms timer
        auto timer_callback = [this]() -> void {
            // 第10次循环时切换到 Offboard 模式并解锁，之后每50次重发一次保持模式
            if (offboard_setpoint_counter_ == 10 || offboard_setpoint_counter_ % 50 == 0) {
                // switch to Offboard mode: param1=1(self), param2=6(Offboard)
                this->publish_vehicle_command_uav0(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm_uav0();
                this->publish_vehicle_command_uav1(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm_uav1();
            }

            // control_mode & trajectory
            publish_offboard_control_mode_uav0();
            publish_trajectory_setpoint_uav0();
            publish_offboard_control_mode_uav1();
            publish_trajectory_setpoint_uav1();

            if (offboard_setpoint_counter_ < 11) {
                offboard_setpoint_counter_++;
            }
        };
        timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm_uav0() { publish_vehicle_command_uav0(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }
    void disarm_uav0() { publish_vehicle_command_uav0(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); }
    void arm_uav1() { publish_vehicle_command_uav1(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0); }
    void disarm_uav1() { publish_vehicle_command_uav1(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0); }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    // UAV0
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_uav0_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_uav0_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_uav0_;

    // UAV1
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_uav1_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_uav1_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_uav1_;

    uint64_t offboard_setpoint_counter_;
    rclcpp::Time time_start_;

    // Leader 位置，供 Follower 跟随使用
    float leader_position_[3] = {0.0, 0.0, -5.0};

    // ===================== UAV0 (Leader) =====================

    void publish_offboard_control_mode_uav0()
    {
        OffboardControlMode msg{};
        msg.position = true;       // only position control
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_uav0_->publish(msg);
    }

    void publish_trajectory_setpoint_uav0()
    {
        auto now = this->get_clock()->now();
        double t = (now - time_start_).seconds();

        // 圆形轨迹参数：半径5m，周期20s，高度-5m (NED坐标系，负值为上方)
        const double radius = 5.0;
        const double period = 20.0;
        double omega = 2.0 * M_PI / period;

        TrajectorySetpoint msg{};
        msg.position = {
            static_cast<float>(radius * std::cos(omega * t)),
            static_cast<float>(radius * std::sin(omega * t)),
            -5.0f
        };
        // 偏航角沿切线方向
        msg.yaw = -omega * t;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_uav0_->publish(msg);

        // 更新 leader 位置供 follower 使用
        leader_position_[0] = msg.position[0];
        leader_position_[1] = msg.position[1];
        leader_position_[2] = msg.position[2];
        RCLCPP_INFO(this->get_logger(), "uav0 position: x=%f, y=%f, z=%f",
                    leader_position_[0], leader_position_[1], leader_position_[2]);
    }

    void publish_vehicle_command_uav0(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;      // 0号实例 -> target_system = px4_instance + 1 = 1
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;    // 外部控制模式必须为 true
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        RCLCPP_INFO(this->get_logger(), "uav0 command: %u, param1=%f, param2=%f, target_system=%u",
                    command, param1, param2, msg.target_system);
        vehicle_command_publisher_uav0_->publish(msg);
    }

    // ===================== UAV1 (Follower) =====================

    void publish_offboard_control_mode_uav1()
    {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_uav1_->publish(msg);
    }

    void publish_trajectory_setpoint_uav1()
    {
        // Follower 在 Leader 后方2m、右侧2m 跟随
        const float offset_x = 2.0;
        const float offset_y = 2.0;
        const float offset_z = 0.0;

        TrajectorySetpoint msg{};
        msg.position = {
            leader_position_[0] + offset_x,
            leader_position_[1] + offset_y,
            leader_position_[2] + offset_z
        };
        msg.yaw = -std::atan2(msg.position[1], msg.position[0]);
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        RCLCPP_INFO(this->get_logger(), "uav1 setpoint: x=%f, y=%f, z=%f",
                    msg.position[0], msg.position[1], msg.position[2]);
        trajectory_setpoint_publisher_uav1_->publish(msg);
    }

    void publish_vehicle_command_uav1(uint16_t command, float param1 = 0.0, float param2 = 0.0)
    {
        VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 2;      // 1号实例 -> target_system = px4_instance + 1 = 2
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        RCLCPP_INFO(this->get_logger(), "uav1 command: %u, param1=%f, param2=%f, target_system=%u",
                    command, param1, param2, msg.target_system);
        vehicle_command_publisher_uav1_->publish(msg);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting multi offboard control node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiOffboardControl>());
    rclcpp::shutdown();
    return 0;
}