#include <rclcpp/rclcpp.hpp>
#include <pavo_sdk/pavo_driver.h>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>

class PavoScanNode : public rclcpp::Node
{
private:
    std::shared_ptr<pavo::pavo_driver> pavo_driver_;
    std::string frame_id, lidar_ip, host_ip, scan_topic;
    int lidar_port, host_port;
    bool inverted, enable_motor, switch_active_mode;
    int motor_speed;
    int merge_coef;
    double angle_min;
    double angle_max;
    double range_max, range_min;
    int method;
    std::vector<pavo_response_scan_t> scan_data;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    PavoScanNode() : Node("pavo_scan_node")
    {
        this->declare_parameter("frame_id", "laser_frame");
        this->declare_parameter("scan_topic", "scan");
        this->declare_parameter("angle_max", 135.00);
        this->declare_parameter("angle_min", -135.00);
        this->declare_parameter("range_max", 20.0);
        this->declare_parameter("range_min", 0.10);
        this->declare_parameter("invert", false);
        this->declare_parameter("motor_speed", 15);
        this->declare_parameter("merge_coef", 2);
        this->declare_parameter("enable_motor", true);
        this->declare_parameter("lidar_ip", "10.10.10.101");
        this->declare_parameter("lidar_port", 2368);
        this->declare_parameter("host_ip", "10.10.10.100");
        this->declare_parameter("host_port", 2368);
        this->declare_parameter("method", 0);
        this->declare_parameter("switch_active_mode", false);

        this->get_parameter("frame_id", frame_id);
        this->get_parameter("scan_topic", scan_topic);
        this->get_parameter("angle_max", angle_max);
        this->get_parameter("angle_min", angle_min);
        this->get_parameter("range_max", range_max);
        this->get_parameter("range_min", range_min);
        this->get_parameter("invert", inverted);
        this->get_parameter("motor_speed", motor_speed);
        this->get_parameter("merge_coef", merge_coef);
        this->get_parameter("enable_motor", enable_motor);
        this->get_parameter("lidar_ip", lidar_ip);
        this->get_parameter("lidar_port", lidar_port);
        this->get_parameter("host_ip", host_ip);
        this->get_parameter("host_port", host_port);
        this->get_parameter("method", method);
        this->get_parameter("switch_active_mode", switch_active_mode);

        RCLCPP_INFO(this->get_logger(),
                    "config lidar's param:angle_min:%.2f  angle_max:%.2f  "
                    "range_min:%.2f  range_max:%.2f  frame_id:%s  inverted:%s  "
                    "method:%d",
                    angle_min,
                    angle_max,
                    range_min,
                    range_max,
                    frame_id.c_str(),
                    inverted ? "True" : "False", method);

        scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);

        if (!switch_active_mode)
        {
            pavo_driver_ = std::make_shared<pavo::pavo_driver>();
        }
        else
        {
            pavo_driver_ = std::make_shared<pavo::pavo_driver>(host_ip, host_port);
        }

        pavo_driver_->pavo_open(lidar_ip, lidar_port);
        pavo_driver_->enable_motor(enable_motor);

        if (!enable_motor)
        {
            throw std::runtime_error("Motor is not enabled");
        }

        if (method >= 0 || method <= 4)
        {
            pavo_driver_->enable_tail_filter(method);
            if (method != 0)
                RCLCPP_INFO(this->get_logger(), "Tail filter method:%d", method);
        }
        else
        {
            throw std::runtime_error("Invalid tail filter method");
        }

        int motor_speed_get;
        if (pavo_driver_->get_motor_speed(motor_speed_get))
        {
            RCLCPP_INFO(this->get_logger(), "Get motor speed: %d", motor_speed_get);
        }
        else
        {
            throw std::runtime_error("Unable to get motor speed");
        }

        if (motor_speed == motor_speed_get || pavo_driver_->set_motor_speed(motor_speed))
        {
            RCLCPP_INFO(this->get_logger(), "Success to set motor speed: %d", motor_speed);
        }
        else
        {
            throw std::runtime_error("Unable to set speed");
        }

        if (pavo_driver_->set_merge_coef(merge_coef))
        {
            RCLCPP_INFO(this->get_logger(), "Success to set merge, merge_coef: %d", merge_coef);
        }
        else
        {
            throw std::runtime_error("Unable to set merge");
        }

        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / motor_speed), std::bind(&PavoScanNode::timer_callback, this));
    }

    ~PavoScanNode()
    {
        pavo_driver_->pavo_close();
    }

    void timer_callback()
    {
        auto start_scan_time = this->now();
        if (!pavo_driver_->get_scanned_data(scan_data, 150))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get scanned data");
            return;
        }
        else
        {
            auto end_scan_time = this->now();
            auto scan_duration = end_scan_time - start_scan_time;
            publish_msg(start_scan_time, scan_duration);
        }
    }

    void publish_msg(const rclcpp::Time &start_time, const rclcpp::Duration &scan_duration)
    {
        sensor_msgs::msg::LaserScan scan_msg;
        auto node_counts = scan_data.size();
        auto counts = node_counts * ((angle_max - angle_min) / 270.0);
        int angle_start = 135 + angle_min;
        int node_start = node_counts * (angle_start / 270.0);

        scan_msg.ranges.resize(counts);
        scan_msg.intensities.resize(counts);

        float range = 0.0;
        float intensity = 0.0;

        for (auto i = 0; i < counts; i++)
        {
            range = scan_data[node_start].distance * 0.002;
            intensity = scan_data[node_start].intensity;
            if ((range > range_max) || (range < range_min))
            {
                range = 0.0;
                intensity = 0.0;
            }
            if (!inverted)
            {
                scan_msg.ranges[i] = range;
                scan_msg.intensities[i] = intensity;
                node_start = node_start + 1;
            }
            else
            {
                scan_msg.ranges[counts - 1 - i] = range;
                scan_msg.intensities[counts - 1 - i] = intensity;
                node_start = node_start + 1;
            }
        }
        scan_msg.header.stamp = start_time;
        scan_msg.header.frame_id = frame_id;
        scan_msg.angle_min = Degree2Radians(angle_min);
        scan_msg.angle_max = Degree2Radians(angle_max);
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (double)counts;
        scan_msg.scan_time = scan_duration.seconds();
        scan_msg.time_increment = scan_duration.seconds() / (double)node_counts;
        scan_msg.range_min = range_min;
        scan_msg.range_max = range_max;
        scan_pub->publish(scan_msg);
    }
};

class RAII
{
public:
    RAII(int argc, char **argv)
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<PavoScanNode>());
    }

    ~RAII()
    {
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv)
{
    auto raii = RAII(argc, argv);
    return 0;
}