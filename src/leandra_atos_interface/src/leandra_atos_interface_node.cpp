#include <leandra_atos_interface/leandra_atos_interface_node.hpp>
#include <leandra_atos_interface/leandra_atos_interface.hpp>

#include "rclcpp_components/register_node_macro.hpp"

using namespace LeandraAtosInterface;

LeandraAtosInterfaceNode::LeandraAtosInterfaceNode(const rclcpp::NodeOptions &options) : Node("leandra_atos_interface_node", options)
{
    try
    {
        // First declare and get the parameter
        m_ip_address = this->declare_parameter<std::string>("ip_address", "127.0.0.1");
        m_output_rate_ms = this->declare_parameter<std::int64_t>("output_rate_ms", 1000);
        std::cout << "IP Adress Parameter: " << m_ip_address << std::endl;

        // Subscriber
         m_subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>("/localization/odom", rclcpp::QoS(rclcpp::KeepLast(1)),
                                                                               std::bind(&LeandraAtosInterfaceNode::odom_callback, this, std::placeholders::_1));
        m_subscriber_novatel_fix = this->create_subscription<sensor_msgs::msg::NavSatFix>("/sensor/novatel/fix", rclcpp::QoS(rclcpp::KeepLast(1)),
                                                                    std::bind(&LeandraAtosInterfaceNode::novatel_callback, this, std::placeholders::_1));
        // Create TestObject with proper exception handling
        try
        {
            m_test_object = std::make_unique<LeandraAtosInterface>(m_ip_address, "Leandra", *this);
        }
        catch (const boost::system::system_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize TestObject: %s", e.what());
            throw;
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Node initialization failed: %s", e.what());
        throw;
    }

    // Services
    m_engage_client = this->create_client<leandra_dbw_msgs::srv::RequestState>("/system/state_machine/state_request");
    m_service_node = rclcpp::Node::make_shared("set_lrm_client");
    m_lrm_client = this->create_client<localization_utils_msgs::srv::SetLrmUtm>("/localization/set_lrm");

    // Timer callback
    m_timer_output = this->create_wall_timer(
        std::chrono::milliseconds(m_output_rate_ms),
        std::bind(&LeandraAtosInterfaceNode::output_callback, this));

    // Publisher
    m_publisher_path = this->create_publisher<nav_msgs::msg::Path>("/map/path", rclcpp::QoS(rclcpp::KeepLast(1)));
    m_velocity_profile_publisher = this->create_publisher<std_msgs::msg::Float64MultiArray>("/map/velocity_profile", rclcpp::QoS(rclcpp::KeepLast(1)));
    m_drive_mode_publisher = this->create_publisher<std_msgs::msg::UInt32MultiArray>("/map/drive_mode", rclcpp::QoS(rclcpp::KeepLast(1)));
    m_publisher_vehicle_trigger = this->create_publisher<leandra_dbw_msgs::msg::VehicleTrigger>("/control/command/vehicle_trigger_cmd", rclcpp::QoS(rclcpp::KeepLast(1)));
}

bool LeandraAtosInterfaceNode::request_state(uint8_t state)
{
    if (!m_engage_client->wait_for_service(std::chrono::milliseconds(1000)))
    {
        RCLCPP_ERROR(this->get_logger(), "Service is not available.");
        return false;
    }

    auto request = std::make_shared<leandra_dbw_msgs::srv::RequestState::Request>();
    request->state = state;

    auto future_response = m_engage_client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_response);
    if (result == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future_response.get();
        return response->acknowledge;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service or response took too long.");
        return false;
    }
}

bool LeandraAtosInterfaceNode::set_lrm_coordinates(
    const double &x,
    const double &y,
    // const double &z,
    const int &zone,
    const bool &northp)
{

    if (!m_novatel_fix) {
    RCLCPP_ERROR(this->get_logger(), "Noch keine GPS-Daten empfangen!");
    return false;
}
    RCLCPP_INFO(this->get_logger(), "Waiting for service");
    if (!m_lrm_client->wait_for_service(std::chrono::seconds(5)))
    {
        RCLCPP_ERROR(this->get_logger(), "LRM service not available");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "LRM service waiting done -> available");
    auto request = std::make_shared<localization_utils_msgs::srv::SetLrmUtm::Request>();
    request->x = x;
    request->y = y;
    request->z = this->m_novatel_fix->altitude;
    request->zone = zone;
    request->northp = northp;

    auto future_response = m_lrm_client->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(m_service_node, future_response);
    if (result == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future_response.get();
        return response->status;
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call lrm service or response took too long.");
        return false;
    }
}

void LeandraAtosInterfaceNode::odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
{
    // reception of odom msgs equals LeanDRA is running
    this->m_test_object->set_LeanDRA_running(true);
    auto heading = quaternion_to_yaw(msg->pose.pose.orientation);
    this->m_test_object->setMonr(msg->pose.pose.position, msg->twist.twist.linear, heading);
    this->m_test_object->update_curr_location(msg->pose.pose.position);
}

void LeandraAtosInterfaceNode::output_callback()
{
    bool log_output = false;
    if (log_output)
        RCLCPP_INFO(this->get_logger(), "Davor");
    auto state = this->m_test_object.get()->getCurrentStateID();
    auto state_name = this->m_test_object.get()->getCurrentStateName();
    if (log_output)
        RCLCPP_INFO(this->get_logger(), "Danach");
    if (state == ISO_OBJECT_STATE_ARMED || state == ISO_OBJECT_STATE_RUNNING || state == ISO_OBJECT_STATE_PRE_RUNNING || state == ISO_OBJECT_STATE_DISARMED)
    {
        if (log_output)
            RCLCPP_INFO(this->get_logger(), "In IF schleife");
        // Publish velocity profile
        std_msgs::msg::Float64MultiArray vel_msg;
        if (log_output)
            RCLCPP_INFO(this->get_logger(), "Erstelle Velocityprofile data");
        vel_msg.data = this->m_test_object->create_velocity_profile();
        if (log_output)
            RCLCPP_INFO(this->get_logger(), "Kurz vor publish velocityprofile");
        m_velocity_profile_publisher->publish(vel_msg);
        if (log_output)
            RCLCPP_INFO(this->get_logger(), "Velocityprofile sollte gepublsiht sein");

        // Publish path
        if (log_output)
            RCLCPP_INFO(this->get_logger(), "Kurz vor path erzeugung/ publishen");
        nav_msgs::msg::Path path_msg;

        m_publisher_path->publish(this->m_test_object->create_path());
        if (log_output)
            RCLCPP_INFO(this->get_logger(), "Current Testobject StateID: %d"
                                            "\nPublishing path",
                        state);

        // Publish drivemode
        std_msgs::msg::UInt32MultiArray drive_msg;
        drive_msg.data = this->m_test_object->create_drive_modes(0x03);
        m_drive_mode_publisher->publish(drive_msg);

        // Publish vehicle triggers
        if (!this->m_test_object->getTrajectory().empty())
        {
            leandra_dbw_msgs::msg::VehicleTrigger veh_trigger_msg;
            veh_trigger_msg.set__hazard_lights(false);
            veh_trigger_msg.set__high_beam(false);
            veh_trigger_msg.set__horn(false);
            veh_trigger_msg.set__indicator_left(false);
            veh_trigger_msg.set__indicator_right(false);
            m_publisher_vehicle_trigger->publish(veh_trigger_msg);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "veh_trigger(): trajectory is empty");
        }
    }
    else
    {
        if (log_output)
            RCLCPP_INFO(this->get_logger(), "Current StateID: %d Current StateNAME: %s", state, state_name.c_str());
    }
}

void LeandraAtosInterfaceNode::novatel_callback(const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg)
{
    this->m_novatel_fix = msg;
    RCLCPP_INFO(this->get_logger(), "Novatel_fix HEIGHT is: %f " ,this->m_novatel_fix->altitude);
}

double LeandraAtosInterfaceNode::quaternion_to_yaw(const geometry_msgs::msg::Quaternion &orientation)
{
    double w = orientation.w;
    double x = orientation.x;
    double y = orientation.y;
    double z = orientation.z;
    return atan2(2 * (x * y + z * w), w * w + x * x - y * y - z * z);
}

RCLCPP_COMPONENTS_REGISTER_NODE(LeandraAtosInterface::LeandraAtosInterfaceNode)