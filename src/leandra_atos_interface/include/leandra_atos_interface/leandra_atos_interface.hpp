#ifndef LEANDRA_ATOS_INTERFACE_HPP
#define LEANDRA_ATOS_INTERFACE_HPP

#include <memory>

#include "iso22133object.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <GeographicLib/UTMUPS.hpp>

namespace LeandraAtosInterface
{
    struct LocalRefMarker
    {
        bool valid = false;
        double x;    // UTM x position
        double y;    // UTM y position
        double z;    // UTM y position
        int zone;    // UTM zone
        bool northp; // hemisphere (true means north, false means south)
    };

    // forward declaration
    class LeandraAtosInterfaceNode;

    class LeandraAtosInterface : public ISO22133::TestObject
    {
    public:
        explicit LeandraAtosInterface(const std::string &ip_address, const std::string &name, rclcpp::Node &node);
        ~LeandraAtosInterface();
        void handleAbort() override;
        void onSTRT(StartMessageType &strt) override;
        void onOSEM(ObjectSettingsType &osem) override;
        void onTRAJ() override;
        void onStateChange() override;
        void setMonr(const geometry_msgs::msg::Point &position, const geometry_msgs::msg::Vector3 &linear, const double &heading);
        void update_curr_location(const geometry_msgs::msg::Point &position);
        double calculate_distance(const double &p1, const double &p2);
        nav_msgs::msg::Path create_path();
        std::vector<double> create_velocity_profile();
        tf2::Quaternion heading_to_quaternion(const CartesianPosition &pos);
        std::vector<uint32_t> create_drive_modes(const std::uint32_t &drive_mode);
        void set_LeanDRA_running(const bool);
        bool is_LeanDRA_running();
        rclcpp::Logger get_interface_logger() const;
        // overriden prearming state creation
        ISO22133::PreArming *createPreArming() const override;

    private:
        // vars
        rclcpp::Node &m_node;
        CartesianPosition pos;
        SpeedType spd;
        nav_msgs::msg::Odometry curr_loc;
        std::vector<TrajectoryWaypointType> trajectory;
        std::thread m_thread_leandra_status;
        std::atomic<bool> m_leandra_thread_running{true};
        std::atomic<bool> m_leandra_ready{false};
        std::mutex osemMutex;
        std::mutex trajMutex;
        // methods
        void check_leandra_status();
        bool leanDRA_running = false;
        void default_monr();
    };

}

#endif // LEANDRA_ATOS_INTERFACE_HPP