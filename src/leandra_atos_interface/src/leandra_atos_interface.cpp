#include "leandra_atos_interface/leandra_atos_interface.hpp"
#include "leandra_atos_interface/leandra_atos_interface_node.hpp"

LeandraAtosInterface::LeandraAtosInterface::LeandraAtosInterface(const std::string &ip_address, const std::string &name, rclcpp::Node &node) : ISO22133::TestObject(ip_address, name), m_node(node)
{
    RCLCPP_INFO(m_node.get_logger(), "TestObject (%s) created with IP: %s", this->getName().c_str(), ip_address.c_str());

    m_thread_leandra_status = std::thread(&LeandraAtosInterface::check_leandra_status, this);
}

LeandraAtosInterface::LeandraAtosInterface::~LeandraAtosInterface()
{
    this->m_leandra_thread_running = false;
    if (m_thread_leandra_status.joinable())
    {
        m_thread_leandra_status.join();
    }
}

void LeandraAtosInterface::LeandraAtosInterface::handleAbort()
{
    bool success = false;
    if (this->getCurrentStateID() == ISO_OBJECT_STATE_ABORTING)
    {
        RCLCPP_INFO(m_node.get_logger(), "TestObjectState changed to EMERGENCY_STOP, requesting ABORT state");
        success = ((LeandraAtosInterfaceNode &)m_node).request_state(leandra_dbw_msgs::msg::StateMachineRos::STATE_TRANSITION_ABORT);
        if (!success)
        {
            RCLCPP_ERROR(m_node.get_logger(), "Failed to request ABORT state");
        }
    }
}

void LeandraAtosInterface::LeandraAtosInterface::onSTRT(StartMessageType &strt)
{
    std::string obj_name = this->getName();
    RCLCPP_INFO(m_node.get_logger(), "TestobjectStarting: ----- %s ------", obj_name.c_str());
    RCLCPP_INFO(m_node.get_logger(), "TimestampEval: %s", strt.isTimestampValid ? "true" : "false");
}

void LeandraAtosInterface::LeandraAtosInterface::onOSEM(ObjectSettingsType &osem)
{
    // Copy the data to avoid lifetime issues
    ObjectSettingsType osemCopy = osem;
    std::thread([this, osemCopy]()
                {
        std::lock_guard<std::mutex> lock(osemMutex);

        // logging frequency (ms)
    uint16_t frequency = 7500;

    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "----- OSEM RECEIVED -----");
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, ">> OSEM ID_STRUCT <<");
    std::cout << "Object Settings Received" << std::endl;
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: ControlCenterID: %d", osemCopy.desiredID.controlCentre);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: TransmitterID: %d", osemCopy.desiredID.transmitter);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: SubtransmitterID: %d", osemCopy.desiredID.subTransmitter);

    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, ">> OSEM ORIGIN_STRUCT <<");
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: CoordinateSystemType:  %d", osemCopy.coordinateSystemType); // see positioning.h
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Origin_latitude: %f IsValid: %s", osemCopy.coordinateSystemOrigin.latitude_deg, osemCopy.coordinateSystemOrigin.isLatitudeValid ? "true" : "false");
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Origin_longitude: %f IsValid: %s", osemCopy.coordinateSystemOrigin.longitude_deg, osemCopy.coordinateSystemOrigin.isLongitudeValid ? "true" : "false");
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Origin_altitude: %f IsValid: %s", osemCopy.coordinateSystemOrigin.altitude_m, osemCopy.coordinateSystemOrigin.isAltitudeValid ? "true" : "false");
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Origin_rotation: %f", osemCopy.coordinateSystemRotation_rad);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, ">> DATETIME_STRUCT <<");

    // RCLCPP_INFO(m_node.get_logger(), "OSEM: Current_date (ISO8601 conform): %ld", osem.currentTime);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: seconds %ld u_seconds: %ld", osemCopy.currentTime.tv_sec, osemCopy.currentTime.tv_usec);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, ">> ACCURACY_REQUIREMENTS_STRUCT << ");
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Max_Longitudinal_Deviation: %f", osemCopy.maxDeviation.position_m);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Max_Lateral_Deviation: %f", osemCopy.maxDeviation.lateral_m);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Max_Yaw_Deviation: %f", osemCopy.maxDeviation.yaw_rad);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Max_Pos_Error: %f", osemCopy.minRequiredPositioningAccuracy_m);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Communication_Timeout (HEAB): %ld (sec) %ld (usec) ", osemCopy.heabTimeout.tv_sec, osemCopy.heabTimeout.tv_usec);

    // -----> Lambda to extract which testmode is selected
    auto testModeToString = [](TestModeType mode) -> std::string
    {
        switch (mode)
        {
        case TestModeType::TEST_MODE_PREPLANNED:
            return "preplanned";
        case TestModeType::TEST_MODE_ONLINE:
            return "online";
        case TestModeType::TEST_MODE_SCENARIO:
            return "scenario";
        case TestModeType::TEST_MODE_UNAVAILABLE:
            return "unavailable";
        default:
            return "unknown";
        }
    };
    // <----- End of Lambda
    std::string testModeStr = testModeToString(osemCopy.testMode);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Testmode %s", testModeStr.c_str());
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: MONR_Rate: %f (configured)", osemCopy.rate.monr);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: MONR2_Rate: %f (configured)", osemCopy.rate.monr2);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: HEAB_Rate: %f (configured)", osemCopy.rate.heab);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, ">> TIMESERVER_STRUCT <<");
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Timeserver_IP: %d", osemCopy.timeServer.ip);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "OSEM: Timeserver_PORT: %d", osemCopy.timeServer.port);
    RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "----- END OF OSEM MESSAGE -----");

    // Get origin values, convert to UTM, trigger ServiceCall to set them as LocalReferenceMarker
    auto geoPos = osemCopy.coordinateSystemOrigin;
    auto lrm = std::make_unique<LocalRefMarker>();
    if (!geoPos.isLatitudeValid || !geoPos.isLongitudeValid || !geoPos.isAltitudeValid)
    {
        RCLCPP_ERROR(m_node.get_logger(), "Geographic coordinates INVALID");
        lrm->valid = false;
    }

    try
    {
        double meridianConvergence = 0.0;
        double k = 0.0;
        // here happens conversion
        GeographicLib::UTMUPS::Forward(
            geoPos.latitude_deg,
            geoPos.longitude_deg,
            lrm->zone,
            lrm->northp,
            lrm->x,
            lrm->y,
            meridianConvergence,
            k);
        lrm->z = geoPos.altitude_m;
        lrm->valid = true;

        // Log converted UTM values
        RCLCPP_INFO(m_node.get_logger(),
                    "In onOSEM: Resulting UTM-Coordinates: x: %lf, y: %lf, z: %lf, Zone: %d",
                    lrm->x, lrm->y, lrm->z, lrm->zone);
    }
    catch (const GeographicLib::GeographicErr &e)
    {
        RCLCPP_ERROR(m_node.get_logger(), "Coordinate conversion failed %s", e.what());
        lrm->valid = false;
    }

    if (!lrm->valid)
    {
        RCLCPP_ERROR(m_node.get_logger(), "In onOSEM: LRM not set, lrm is INVALID");
    }
    else if (((LeandraAtosInterfaceNode &)m_node).set_lrm_coordinates(lrm->x, lrm->y, /*lrm->z, */ lrm->zone, lrm->northp))
    {
        RCLCPP_INFO(m_node.get_logger(), "In onOSEM: Successfully set LRM coordinates.");
    }

    setObjectSettings(osemCopy); })
        .detach();
}

void LeandraAtosInterface::LeandraAtosInterface::onTRAJ()
{
    // seperate thread
    std::thread([this]()
                {
    bool log_waypoints = false;
    RCLCPP_INFO(m_node.get_logger(), "Received TRAJ message from ATOS");

    //  Get the trajectory data
    std::vector<TrajectoryWaypointType> newTraj = this->getTrajectory();

    if (newTraj.empty())
    {
        RCLCPP_WARN(m_node.get_logger(), "Received empty trajectory");
        return;
    }

    // Log received points of trajectory
    if (log_waypoints)
    {
        RCLCPP_INFO(m_node.get_logger(), "Received %zu trajectory points", newTraj.size());
        for (size_t i = 0; i < newTraj.size(); ++i)
        {
            const TrajectoryWaypointType &point = newTraj[i];
            RCLCPP_INFO(m_node.get_logger(),
                        "Waypoint %zu: Time: %ld.%06ld"
                        "\n\tPosition (x,y,z): %.3f, %.3f, %.3f"
                        "\n\tHeading: %.3f"
                        "\n\tSpeed: (long,lat): %.3f, %.3f",
                        i,
                        point.relativeTime.tv_sec, point.relativeTime.tv_usec,
                        point.pos.xCoord_m, point.pos.yCoord_m, point.pos.zCoord_m,
                        point.pos.heading_rad,
                        point.spd.longitudinal_m_s, point.spd.lateral_m_s);
        }
    }

    // Store trajectory based on testmode type
    TestModeType currentMode = this->getObjectSettings().testMode;
    if (currentMode == TEST_MODE_ONLINE)
    {
        RCLCPP_INFO(m_node.get_logger(), "Online Mode: Appending trajectory.");
        {
            std::lock_guard<std::mutex> lock(trajMutex);
            this->trajectory.insert(this->trajectory.end(), newTraj.begin(), newTraj.end());
        }


        // We might receive trajectories that overlap, we remove the duplicate points by checking the time
        std::sort(this->trajectory.begin(), this->trajectory.end(), [](const TrajectoryWaypointType &t1, const TrajectoryWaypointType &t2)
                  { return t1.relativeTime.tv_sec * 1000000 + t1.relativeTime.tv_usec < t2.relativeTime.tv_sec * 1000000 + t2.relativeTime.tv_usec; });
        this->trajectory.erase(std::unique(this->trajectory.begin(), this->trajectory.end(), [](const TrajectoryWaypointType &t1, const TrajectoryWaypointType &t2)
                                           { return t1.relativeTime.tv_sec * 1000000 + t1.relativeTime.tv_usec == t2.relativeTime.tv_sec * 1000000 + t2.relativeTime.tv_usec; }),
                               this->trajectory.end());
    }
    else if (currentMode == TEST_MODE_PREPLANNED)
    {
        RCLCPP_INFO(m_node.get_logger(), "Preplanned mode: Replacing trajectory");
        {
            std::lock_guard<std::mutex> lock(trajMutex);
            this->trajectory = newTraj;
        }
    }
    RCLCPP_INFO(m_node.get_logger(), "New after overlapping points Trajectory size: %ld", this->trajectory.size()); })
        .detach();
}

void LeandraAtosInterface::LeandraAtosInterface::onStateChange()
{
    auto current_state = this->getCurrentStateID();
    RCLCPP_INFO(m_node.get_logger(), "State changed to StateNAME: %s, StateID: %d ", this->getCurrentStateName().c_str(), this->getCurrentStateID());

    switch (current_state)
    {

        bool success;
    case ISO_OBJECT_STATE_ARMED:
        RCLCPP_INFO(m_node.get_logger(), "TestObjectState changed to ARMED, requesting READY state");
        success = ((LeandraAtosInterfaceNode &)m_node).request_state(leandra_dbw_msgs::msg::StateMachineRos::STATE_TRANSITION_READY);
        if (!success)
        {
            RCLCPP_ERROR(m_node.get_logger(), "Failed to request READY state");
        }
        break;
    case ISO_OBJECT_STATE_RUNNING:
        RCLCPP_INFO(m_node.get_logger(), "TestObjectState changed to RUNNING, requesting RUN_REAL state");
        success = ((LeandraAtosInterfaceNode &)m_node).request_state(leandra_dbw_msgs::msg::StateMachineRos::STATE_TRANSITION_RUN_REAL);
        if (!success)
        {
            RCLCPP_ERROR(m_node.get_logger(), "Failed to request RUN_REAL state");
        }
        break;
    case ISO_OBJECT_STATE_POSTRUN:
        RCLCPP_INFO(m_node.get_logger(), "TestObjectState changed to NORMAL_STOP, requesting UN_PREPARED state");
        success = ((LeandraAtosInterfaceNode &)m_node).request_state(leandra_dbw_msgs::msg::StateMachineRos::STATE_TRANSITION_UN_PREPARED);
        if (!success)
        {
            RCLCPP_ERROR(m_node.get_logger(), "Failed to request UN_PREPARED state");
        }
        break;
    default:
        RCLCPP_INFO(m_node.get_logger(), "Default case: Current Testobject state: %s", this->getCurrentStateName().c_str());
        break;
    }
}

void LeandraAtosInterface::LeandraAtosInterface::setMonr(const geometry_msgs::msg::Point &position, const geometry_msgs::msg::Vector3 &linear, const double &heading)
{

    // logging frequency (ms)
    uint16_t frequency = 7500;
    bool logging = false;

    // store positions of /localization/odom
    this->pos.xCoord_m = position.x;
    this->pos.yCoord_m = position.y;
    this->pos.zCoord_m = position.z;
    this->pos.heading_rad = heading;
  

    // set booleans true
    this->pos.isHeadingValid = true;
    this->pos.isPositionValid = true;
    this->pos.isXcoordValid = true;
    this->pos.isYcoordValid = true;
    this->pos.isZcoordValid = true;


    // store linear (speed) of /localization/odom
    this->spd.lateral_m_s = linear.y;
    this->spd.longitudinal_m_s = linear.x;


    // speed booleans
    spd.isLateralValid = true;
    spd.isLongitudinalValid = true;

    if (logging)
    {
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, ">>>>> START LOGGING <<<<<");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "Position_X: %f", this->pos.xCoord_m);
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "Position_Y: %f", this->pos.yCoord_m);
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "Position_Z: %f", this->pos.zCoord_m);
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "Heading_rad: %f ", this->pos.heading_rad);
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "isLateralValid: %s ", this->spd.isLateralValid ? "true" : "false");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "isLongitudinalValid: %s ", this->spd.isLongitudinalValid ? "true" : "false");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, ">>>>> END <<<<<");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "isHeadingValid: %s ", this->pos.isHeadingValid ? "true" : "false");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "isPositionValid: %s ", this->pos.isPositionValid ? "true" : "false");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "isXcoordValid: %s ", this->pos.isXcoordValid ? "true" : "false");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "isYcoordValid: %s ", this->pos.isYcoordValid ? "true" : "false");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "isZcoordValid: %s ", this->pos.isZcoordValid ? "true" : "false");
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "LateralSpeed: %f ", this->spd.lateral_m_s);
        RCLCPP_INFO_THROTTLE(m_node.get_logger(), *m_node.get_clock(), frequency, "LongitudinalSpeed: %f ", this->spd.longitudinal_m_s);
    }

    // collectivly send values
    this->setSpeed(this->spd);
    this->setPosition(this->pos);
}

void LeandraAtosInterface::LeandraAtosInterface::update_curr_location(const geometry_msgs::msg::Point &position)
{
    this->curr_loc.pose.pose.position.x = position.x;
    this->curr_loc.pose.pose.position.y = position.y;
    this->curr_loc.pose.pose.position.z = position.z;
}

double LeandraAtosInterface::LeandraAtosInterface::calculate_distance(const double &p1, const double &p2)
{ // unnötige fkt
    return (p2 - p1);
}

nav_msgs::msg::Path LeandraAtosInterface::LeandraAtosInterface::create_path()
{ // static atos trajectory 2 navmsgs Path
    bool log_units = false;
    const auto traj = this->trajectory;
    if (log_units)
        RCLCPP_INFO(this->m_node.get_logger(), "Trajectory status %s, Size = %zu ", traj.empty() ? "empty" : "filled", traj.size());
    nav_msgs::msg::Path path;
    if (!traj.empty())
    {
        // set path header
        path.header.frame_id = "map";
        path.header.stamp = this->m_node.get_clock()->now(); // tiimestamp of path
        for (size_t i = 0; i < traj.size(); i++)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            // fill Header;
            pose_stamped.header.frame_id = "map";
            pose_stamped.header.stamp.sec = traj[i].relativeTime.tv_sec;
            pose_stamped.header.stamp.nanosec = traj[i].relativeTime.tv_usec * 1000; // convert micro 2 nanosecs
            if (log_units)
                RCLCPP_INFO(this->m_node.get_logger(), "header.sec = %ld , header.microsec: %ld", traj[i].relativeTime.tv_sec, traj[i].relativeTime.tv_usec);
            // fill content
            pose_stamped.pose.position.x = traj[i].pos.xCoord_m;
            pose_stamped.pose.position.y = traj[i].pos.yCoord_m;
            pose_stamped.pose.position.z = traj[i].pos.zCoord_m;
            if (log_units)
                RCLCPP_INFO(this->m_node.get_logger(), "Pose: X%f, Y%f, Z%f", traj[i].pos.xCoord_m, traj[i].pos.yCoord_m, traj[i].pos.zCoord_m);

            tf2::Quaternion q = heading_to_quaternion(traj[i].pos);
            if (log_units)
                RCLCPP_INFO(this->m_node.get_logger(), "Now quaternions");
            pose_stamped.pose.orientation.x = q.getX();
            pose_stamped.pose.orientation.y = q.getY();
            pose_stamped.pose.orientation.z = q.getZ();
            pose_stamped.pose.orientation.w = q.getW();

            path.poses.push_back(pose_stamped);
        }
    }
    else
    {
        RCLCPP_INFO(this->m_node.get_logger(), "create_path(): trajectory is empty");
    }

    return path;
}

std::vector<double> LeandraAtosInterface::LeandraAtosInterface::create_velocity_profile()
{ // static atos trajectory 2 navmsgs Path

    // const auto traj = this->getTrajectory();
    const auto traj = this->trajectory;
    std::vector<double> velocities;
    if (!traj.empty())
    {
        for (size_t i = 0; i < traj.size(); i++)
        {
            velocities.push_back(traj[i].spd.longitudinal_m_s);
        }
    }
    else
    {
        RCLCPP_INFO(this->m_node.get_logger(), "create_velocity_profile(): trajectory is empty");
    }

    return velocities;
}

tf2::Quaternion LeandraAtosInterface::LeandraAtosInterface::heading_to_quaternion(const CartesianPosition &pos)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, pos.heading_rad);
    return q;
}

std::vector<std::uint32_t> LeandraAtosInterface::LeandraAtosInterface::create_drive_modes(const std::uint32_t &drive_mode)
{
    const auto traj = this->trajectory;
    std::vector<std::uint32_t> drive_modes;
    if (!traj.empty())
    {
        for (size_t i = 0; i < traj.size(); i++)
        {
            drive_modes.push_back(drive_mode);
        }
    }
    else
    {
        RCLCPP_INFO(this->m_node.get_logger(), "create_drive_modes(): trajectory is empty");
    }

    return drive_modes;
}

void LeandraAtosInterface::LeandraAtosInterface::set_LeanDRA_running(const bool running_status)
{
    this->leanDRA_running = running_status;
}

void LeandraAtosInterface::LeandraAtosInterface::check_leandra_status()
{
    while (m_leandra_thread_running)
    {
        try
        {
            auto leandra_status = this->is_LeanDRA_running();

            if (!leandra_status)
            {
                this->default_monr();
                this->m_leandra_ready = false;
                RCLCPP_WARN_THROTTLE(
                    this->m_node.get_logger(),
                    *this->m_node.get_clock(),
                    2000,
                    "LeanDRA is not ready -- Test execution blocked. Using default MONR values for connection.");
            }
            else
            {
                this->m_leandra_ready = true;
                RCLCPP_INFO_THROTTLE(
                    this->m_node.get_logger(),
                    *this->m_node.get_clock(),
                    10000,
                    "LeanDRA ready - Test execution permitted.");
                m_leandra_thread_running = false; // Optional: stop thread if status is ready
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(
                this->m_node.get_logger(),
                "Exception in LeanDRA status check: %s", e.what());
        }

        // 4. Interruptible Sleep Pattern
        // Break 1-second sleep into 10 smaller sleeps
        // This allows faster response to shutdown requests
        for (size_t i = 0; i < 10 && m_leandra_thread_running; i++)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    RCLCPP_INFO(m_node.get_logger(), "Exiting LeanDRA status check thread.");
}

bool LeandraAtosInterface::LeandraAtosInterface::is_LeanDRA_running()
{
    return this->leanDRA_running;
}

void LeandraAtosInterface::LeandraAtosInterface::default_monr()
{
    // Set default position
    this->pos.xCoord_m = 0.0;
    this->pos.yCoord_m = 0.0;
    this->pos.zCoord_m = 0.0;
    this->pos.heading_rad = 0.0;
    this->pos.isPositionValid = true;
    this->pos.isHeadingValid = true;
    this->pos.isXcoordValid = true;
    this->pos.isYcoordValid = true;
    this->pos.isZcoordValid = true;

    // Set default speed
    this->spd.lateral_m_s = 0.0;
    this->spd.longitudinal_m_s = 0.0;
    this->spd.isLateralValid = true;
    this->spd.isLongitudinalValid = true;

    // collectivly send values
    this->setSpeed(this->spd);
    this->setPosition(this->pos);
}

rclcpp::Logger LeandraAtosInterface::LeandraAtosInterface::get_interface_logger() const
{
    return m_node.get_logger();
}

class CustomPreArming : public ISO22133::PreArming
{
private:
    LeandraAtosInterface::LeandraAtosInterface &m_interface;

public:
    explicit CustomPreArming(LeandraAtosInterface::LeandraAtosInterface &interface) : m_interface(interface) {}

    void onEnter(ISO22133::TestObject &obj) override
    {
        if (!m_interface.is_LeanDRA_running())
        {
            RCLCPP_ERROR(m_interface.get_interface_logger(), "Cannot transition to ARMED: LeanDRA is NOT ready.");
            // back to disarmed
            this->handleEvent(obj, ISO22133::Events::W);
            return;
        }

        // Proceed with normal Prearming (leandra is ready)
        ISO22133::PreArming::onEnter(obj);
    }
};

// 2. Factory Method Override
// This method is called by the ISO22133 state machine when creating PreArming state
ISO22133::PreArming *LeandraAtosInterface::LeandraAtosInterface::createPreArming() const
{
    // const_cast needed because method is const but we need non-const reference
    // to track LeanDRA status
    return new CustomPreArming(*const_cast<LeandraAtosInterface *>(this));
}
// 3. State Machine Flow Example:
// When ATOS sends arm command:
// Disarmed -> PreArming (CustomPreArming checks LeanDRA status)
//   If LeanDRA ready: PreArming -> Armed (normal flow)
//   If LeanDRA not ready: PreArming -> Abort / Emergency Stop (blocked transition)