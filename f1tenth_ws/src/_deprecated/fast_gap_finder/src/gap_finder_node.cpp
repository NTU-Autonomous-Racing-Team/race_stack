#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <algorithm>
#include <vector>
#include <cmath>


class PID {
public:
    PID(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, double bias = 0.0, double max_integral = std::numeric_limits<double>::infinity())
            : Kp_(Kp), Ki_(Ki), Kd_(Kd), bias_(bias), max_integral_(max_integral),
              set_point_(0.0), prev_error_(0.0), integral_(0.0) {}

    double update(double feedback) {
        double error = set_point_ - feedback;

        if (std::abs(integral_ + Ki_ * error) <= max_integral_) {
            integral_ += Ki_ * error;
        }

        double derivative = error - prev_error_;
        prev_error_ = error;
        return Kp_ * error + integral_ + Kd_ * derivative + bias_;
    }

    void setPIDGains(double Kp, double Ki, double Kd, double bias = 0.0) {
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
        bias_ = bias;
    }

    void setPoint(double set_point) {
        set_point_ = set_point;
    }

private:
    double Kp_, Ki_, Kd_, bias_, max_integral_;
    double set_point_, prev_error_, integral_;
};


struct Twist {
    double speed;
    double steering;
};

class GapFinderAlgorithm {
public:
    GapFinderAlgorithm(double safety_bubble_diameter = 0.4,
                       double view_angle = M_PI,
                       double coefficient_of_friction = 0.71,
                       double disparity_threshold = 0.6,
                       double lookahead = 10.0,
                       double speed_kp = 1.0,
                       double steering_kp = 1.2,
                       double wheel_base = 0.324)
            : safety_bubble_diameter_(safety_bubble_diameter),
              view_angle_(view_angle),
              coefficient_of_friction_(coefficient_of_friction),
              disparity_threshold_(disparity_threshold),
              lookahead_(lookahead),
              wheel_base_(wheel_base),
              min_speed_(1.0),
              max_speed_(10.0),
              max_steering_(0.4),
              speed_pid_(speed_kp, 0.0, 0.0),  // Assuming PID gains are set here for illustration
              steering_pid_(steering_kp, 0.0, 0.0) {
        speed_pid_.setPoint(0.0);
        steering_pid_.setPoint(0.0);
    }

    Twist update(std::vector<double> ranges, double angle_increment) {

        Twist twist;

        // front clearance
        int mid_index = ranges.size() / 2;
        double front_clearance = std::accumulate(ranges.begin() + mid_index - 10, ranges.begin() + mid_index + 10, 0.0) / 20.0;

        // limit lookahead
        if (lookahead_) {
            std::transform(ranges.begin(), ranges.end(), ranges.begin(), [this](double r) { return std::min(r, lookahead_); });
        }

        // disparity checker
        std::vector<double> cp_ranges = ranges;
        std::vector<int> marked_indexes;

        for (size_t i = 1; i < ranges.size(); ++i) {
            if (std::abs(ranges[i] - ranges[i-1]) > disparity_threshold_) {
                marked_indexes.push_back(ranges[i] < ranges[i-1] ? i : i-1);
            }
        }


        // left right minimum
        // split ranges
        std::vector<double> ranges_right(ranges.begin(), ranges.begin() + mid_index);
        std::vector<double> ranges_left(ranges.begin() + mid_index, ranges.end());

        // find minimum
        int min_range_index = std::distance(ranges_left.begin(), std::min_element(ranges_left.begin(), ranges_left.end()));
        marked_indexes.push_back(mid_index + min_range_index);
        min_range_index = std::distance(ranges_right.begin(), std::min_element(ranges_right.begin(), ranges_right.end()));
        marked_indexes.push_back(min_range_index);

        // apply safety
        for (int idx : marked_indexes) {
            if (cp_ranges[idx] == 0.0) continue;
            int arc_increment = static_cast<int>(safety_bubble_diameter_ / (angle_increment * cp_ranges[idx]) / 2);
            int start_idx = std::max(0, idx - arc_increment);
            int end_idx = std::min(static_cast<int>(ranges.size()), idx + arc_increment + 1);
            std::fill(ranges.begin() + start_idx, ranges.begin() + end_idx, 0.0);
        }

        // prioritize center of scan
        std::vector<double> masked_ranges = ranges;
        applyMask(ranges_left, ranges_left.size() / 2);
        applyMask(ranges_right, ranges_right.size() / 2);

        // limit field of view
        int view_angle_count = static_cast<int>(view_angle_ / angle_increment);
        int lower_bound = (masked_ranges.size() - view_angle_count) / 2;
        int upper_bound = lower_bound + view_angle_count;

        // find maximum gap
        std::vector<double> limited_range(masked_ranges.begin() + lower_bound, masked_ranges.begin() + upper_bound + 1);
        int max_gap_index = std::distance(limited_range.begin(), std::max_element(limited_range.begin(), limited_range.end()));

        // calculate twist
        double goal_bearing = angle_increment * (max_gap_index - limited_range.size() / 2);
//        double goal_range = limited_range[max_gap_index];

        double init_steering = std::atan(goal_bearing * wheel_base_);
        double steering = steering_pid_.update(init_steering);

        double init_speed = std::sqrt(10 * coefficient_of_friction_ * wheel_base_ / std::max(std::tan(std::abs(steering)), 1e-9));
        init_speed = front_clearance / max_speed_ * std::min(init_speed, max_speed_);
        double speed = speed_pid_.update(init_speed);

        twist.speed = std::clamp(speed, min_speed_, max_speed_);
        twist.steering = std::max(std::min(steering, max_steering_), -max_steering_);

        return twist;
    }

    void applyMask(std::vector<double>& ranges, int mid_index) {
        double decrement_factor = 0.001; // adjust as necessary
        for (int i = 0; i <= mid_index; ++i) {
            double mask = 1.0 - decrement_factor * i;
            ranges[i] *= mask;
            ranges[ranges.size() - i - 1] *= mask;
        }
    }


private:
    double safety_bubble_diameter_;
    double view_angle_;
    double coefficient_of_friction_;
    double disparity_threshold_;
    double lookahead_;
    double wheel_base_;
    double min_speed_;
    double max_speed_;
    double max_steering_;
    PID speed_pid_;
    PID steering_pid_;
};


class GapFinderNode : public rclcpp::Node {
public:
    GapFinderNode()
            : Node("gap_finder"),
              gapFinderAlgorithm_(0.5, M_PI, 0.71, 0.25, 10.0, 1.0, 1.5, 0.324),
              max_speed_(10.0), min_speed_(1.0), max_steering_(0.4) {

        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", rclcpp::QoS(1),
                std::bind(&GapFinderNode::scanCallback, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "ego_racecar/odom", rclcpp::QoS(1),
                std::bind(&GapFinderNode::odomCallback, this, std::placeholders::_1));

        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", rclcpp::QoS(1));

        timer_ = this->create_wall_timer(
                std::chrono::milliseconds(20), // 50 Hz
                std::bind(&GapFinderNode::timerCallback, this));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!msg->ranges.empty()) { // error handles
            scan_ready_ = true;
            scan_msg_ = *msg;
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (msg != nullptr) { // error handles
            odom_ready_ = true;
            odom_msg_ = *msg;
        }
    }

    void timerCallback() {
        if (scan_ready_ && odom_ready_) {
            auto twist = gapFinderAlgorithm_.update(
                    std::vector<double>(scan_msg_.ranges.begin(), scan_msg_.ranges.end()),
                    scan_msg_.angle_increment);

            // Apply limits
            twist.steering = std::max(std::min(twist.steering, max_steering_), -max_steering_);
            twist.speed = std::clamp(twist.speed, min_speed_, max_speed_);

            // Publish drive command
            ackermann_msgs::msg::AckermannDriveStamped drive_msg;
            drive_msg.drive.speed = twist.speed;
            drive_msg.drive.steering_angle = twist.steering;
            drive_publisher_->publish(drive_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    GapFinderAlgorithm gapFinderAlgorithm_;
    double max_speed_, min_speed_, max_steering_;
    bool scan_ready_ = false, odom_ready_ = false;
    sensor_msgs::msg::LaserScan scan_msg_;
    nav_msgs::msg::Odometry odom_msg_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GapFinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
