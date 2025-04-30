#pragma once

#include <rclcpp/rclcpp.hpp>

static const rmw_qos_profile_t rmw_qos_profile_latched =
{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

namespace imu_filter {

class BaseNode : public rclcpp::Node
{
  public:
    explicit BaseNode(std::string name, const rclcpp::NodeOptions &options)
        : Node(name, options)
    {
    }

    typedef struct {
        double from_value;
        double to_value;
        double step;
    } floating_point_range;

    typedef struct {
        int from_value;
        int to_value;
        int step;
    } integer_range;

    // Declare a parameter that has no integer or floating point range
    // constraints
    void add_parameter(const std::string &name,
                       const rclcpp::ParameterValue &default_value,
                       const std::string &description = "",
                       const std::string &additional_constraints = "",
                       bool read_only = false)
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;

        declare_parameter(descriptor.name, default_value, descriptor);
    }

    // Declare a parameter that has a floating point range constraint
    void add_parameter(const std::string &name,
                       const rclcpp::ParameterValue &default_value,
                       const floating_point_range fp_range,
                       const std::string &description = "",
                       const std::string &additional_constraints = "",
                       bool read_only = false)
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;
        descriptor.floating_point_range.resize(1);
        descriptor.floating_point_range[0].from_value = fp_range.from_value;
        descriptor.floating_point_range[0].to_value = fp_range.to_value;
        descriptor.floating_point_range[0].step = fp_range.step;

        declare_parameter(descriptor.name, default_value, descriptor);
    }

    // Declare a parameter that has an integer range constraint
    void add_parameter(const std::string &name,
                       const rclcpp::ParameterValue &default_value,
                       const integer_range int_range,
                       const std::string &description = "",
                       const std::string &additional_constraints = "",
                       bool read_only = false)
    {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();

        descriptor.name = name;
        descriptor.description = description;
        descriptor.additional_constraints = additional_constraints;
        descriptor.read_only = read_only;
        descriptor.integer_range.resize(1);
        descriptor.integer_range[0].from_value = int_range.from_value;
        descriptor.integer_range[0].to_value = int_range.to_value;
        descriptor.integer_range[0].step = int_range.step;

        declare_parameter(descriptor.name, default_value, descriptor);
    }

    protected:
        const rmw_qos_profile_t qos_string_to_qos(const std::string& str)
{
    #ifndef DASHING
        if (str == "UNKNOWN")
            return rmw_qos_profile_unknown;
    #endif
        if (str == "SYSTEM_DEFAULT")
            return rmw_qos_profile_system_default;
        if (str == "DEFAULT")
            return rmw_qos_profile_default;
        if (str == "HID_DEFAULT")
        {
            rmw_qos_profile_t profile = rmw_qos_profile_default;
            profile.depth = 100;
            return profile;
        }
        if (str == "EXTRINSICS_DEFAULT")
            return rmw_qos_profile_latched;
        if (str == "PARAMETER_EVENTS")
            return rmw_qos_profile_parameter_events;
        if (str == "SERVICES_DEFAULT")
            return rmw_qos_profile_services_default;
        if (str == "PARAMETERS")
            return rmw_qos_profile_parameters;
        if (str == "SENSOR_DATA")
            return rmw_qos_profile_sensor_data;
        throw std::runtime_error("Unknown QoS string " + str);
    }
};

}  // namespace imu_filter
