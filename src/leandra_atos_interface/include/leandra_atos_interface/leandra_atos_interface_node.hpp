#ifndef LEANDRA_ATOS_INTERFACE_NODE_HPP
#define LEANDRA_ATOS_INTERFACE_NODE_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <leandra_dbw_msgs/msg/vehicle_trigger.hpp>
#include <leandra_dbw_msgs/msg/state_machine_ros.hpp>
#include <leandra_dbw_msgs/srv/request_state.hpp>
#include <localization_utils_msgs/srv/set_lrm_utm.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace LeandraAtosInterface
{
    // forward declaration
    class LeandraAtosInterface;

    class LeandraAtosInterfaceNode : public rclcpp::Node
    {

    public:
        explicit LeandraAtosInterfaceNode(const rclcpp::NodeOptions &options);
        virtual ~LeandraAtosInterfaceNode() = default; // Add virtual destructor
        bool request_state(uint8_t state);
        bool set_lrm_coordinates(
            const double &x,
            const double &y,
            //const double &z,
            const int &zone,
            const bool &northp);

    private:
        // Services
        std::shared_ptr<rclcpp::Node> m_service_node;
        rclcpp::Client<leandra_dbw_msgs::srv::RequestState>::SharedPtr m_engage_client;
        rclcpp::Client<localization_utils_msgs::srv::SetLrmUtm>::SharedPtr m_lrm_client;
        // Callbacks
        void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg);
        void output_callback();
        void novatel_callback(const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg);

        // Subscriber
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber_odom;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_subscriber_novatel_fix;

        // Timer
        rclcpp::TimerBase::SharedPtr m_timer_output;

        // Publisher
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_publisher_path;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_velocity_profile_publisher;
        rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr m_drive_mode_publisher;
        rclcpp::Publisher<leandra_dbw_msgs::msg::VehicleTrigger>::SharedPtr m_publisher_vehicle_trigger;

        // Member vars
        std::shared_ptr<nav_msgs::msg::Odometry> m_last_odom;
        std::string m_ip_address;
        std::int64_t m_output_rate_ms;
        std::unique_ptr<LeandraAtosInterface> m_test_object;
        std::shared_ptr<sensor_msgs::msg::NavSatFix> m_novatel_fix;

        // Helper methods
        double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &orientation);
    };

}
#endif // LEANDRA_ATOS_INTERFACE_NODE_HPP