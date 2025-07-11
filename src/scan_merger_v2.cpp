#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <mutex>
#include <memory>


class ScanMergerV2 : public rclcpp::Node
{
public:
    //constructor
    ScanMergerV2() : Node("scan_merger_v2"), tf_buffer_(get_clock())
    {
        max_range_ = 12.0f;
        angle_min_ = -M_PI;
        angle_max_ = M_PI;
        angle_increment_ = 0.00348f;
        num_points_ = static_cast<int>((angle_max_ - angle_min_) / angle_increment_) + 1;

        std::string node_namespace = get_namespace();

        // Keep this piece of code unchanged as requested
        this->declare_parameter<std::string>("robot_name", "");
        robot_name_ = this->get_parameter("robot_name").as_string();

        if (node_namespace == "/") {
            robot_namespace_ = "";
            base_frame_ = "base_link";
            lidar_right_frame_ = robot_name_ + "/base_link/gpu_lidar_right";
            lidar_left_frame_ = robot_name_ + "/base_link/gpu_lidar_left";
            scan1_topic_ = "scan1";
            scan2_topic_ = "scan2";
        } else {
            // Remove leading slash and use double namespace format to match TF tree
            robot_namespace_ = node_namespace.substr(1); // Remove leading slash
            // robot_namespace_ = robot_namespace;
            base_frame_ = robot_namespace_ + "/base_link";
            lidar_right_frame_ =  robot_name_ + "/" + robot_namespace_ + "/base_link/gpu_lidar_right";
            lidar_left_frame_ =   robot_name_ + "/" + robot_namespace_ + "/base_link/gpu_lidar_left";
            // scan1_topic_ = node_namespace + "/" + "scan1";  // Will be automatically namespaced
            // scan2_topic_ = node_namespace + "/" + "scan2";  // Will be automatically namespaced
            scan1_topic_ = "scan1";  // Will be automatically namespaced
            scan2_topic_ = "scan2";  // Will be automatically namespaced
        }

        RCLCPP_INFO(this->get_logger(), "Robot name: %s, Namespace: %s", 
                    robot_name_.c_str(), node_namespace.c_str());
        RCLCPP_INFO(this->get_logger(), "Using frames - base: %s, lidar_right: %s, lidar_left: %s", 
                    base_frame_.c_str(), lidar_right_frame_.c_str(), lidar_left_frame_.c_str());
        
        RCLCPP_INFO(this->get_logger(), "Subscribing to topics - scan1: %s, scan2: %s", 
                    scan1_topic_.c_str(), scan2_topic_.c_str());

        // Set TF buffer cache time to handle past transforms
        tf_buffer_.setUsingDedicatedThread(true);

        lidar1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan1_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&ScanMergerV2::scan1_callback, this, std::placeholders::_1)
        );
        
        lidar2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan2_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&ScanMergerV2::scan2_callback, this, std::placeholders::_1)
        );

        merged_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "scan",
            rclcpp::SensorDataQoS()
        );

        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this);

        // Initialize last processed times
        last_scan1_time_ = rclcpp::Time(0);
        last_scan2_time_ = rclcpp::Time(0);
    }

private:
    void scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    std::vector<std::pair<float, float>> laserscan_to_point_at_time(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, 
        std::string lidar_frame,
        rclcpp::Time reference_time);
    void process_and_publish_scans();

    // initializing pointers and variables
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar2_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_;
    
    // Store latest scans with thread safety
    sensor_msgs::msg::LaserScan::SharedPtr scan1_;
    sensor_msgs::msg::LaserScan::SharedPtr scan2_;
    std::mutex scan_mutex_;

    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    float max_range_;
    float angle_min_;
    float angle_max_;
    float angle_increment_;
    int num_points_;

    // Track last processed timestamps to avoid duplicate processing
    rclcpp::Time last_scan1_time_;
    rclcpp::Time last_scan2_time_;

    std::string robot_namespace_;
    std::string robot_name_;
    std::string base_frame_;
    std::string lidar_right_frame_;
    std::string lidar_left_frame_;
    std::string scan1_topic_;
    std::string scan2_topic_;
};


void ScanMergerV2::scan1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan1_ = msg;
    last_scan1_time_ = msg->header.stamp;
    
    // Process immediately when scan arrives
    // process_and_publish_scans(); // commenting out because it is duplicating
}


void ScanMergerV2::scan2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    std::lock_guard<std::mutex> lock(scan_mutex_);
    scan2_ = msg;
    last_scan2_time_ = msg->header.stamp;
    
    // Process immediately when scan arrives
    process_and_publish_scans();
}


// // Transform scans to a specific reference time for temporal consistency
// std::vector<std::pair<float, float>> ScanMergerV2::laserscan_to_point_at_time(
//     const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, 
//     std::string lidar_frame,
//     rclcpp::Time reference_time) {
    
//     std::vector<std::pair<float, float>> points;

//     try {
//         // Transform scan points to base_frame at the reference time
//         geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
//             base_frame_,
//             lidar_frame,
//             reference_time,  // Use reference time for consistent transforms
//             rclcpp::Duration::from_seconds(0.1)
//         );

//         float tx = transform.transform.translation.x;
//         float ty = transform.transform.translation.y;
//         float tz = transform.transform.translation.z;

//         geometry_msgs::msg::Quaternion q = transform.transform.rotation;
//         tf2::Quaternion quat(q.x, q.y, q.z, q.w);
//         tf2::Matrix3x3 rot_matrix(quat);

//         int size = scan_msg->ranges.size();
//         for (int i = 0; i < size; i++){
//             float r = scan_msg->ranges[i];

//             if (r < scan_msg->range_min || r > scan_msg->range_max || !std::isfinite(r)){
//                 continue;
//             }

//             float angle = scan_msg->angle_min + i * (scan_msg->angle_increment);
//             float x_lidar = r * std::cos(angle);
//             float y_lidar = r * std::sin(angle);
//             float z_lidar = 0.0;

//             tf2::Vector3 p_lidar(x_lidar, y_lidar, z_lidar);
//             tf2::Vector3 p_base = rot_matrix * p_lidar + tf2::Vector3(tx, ty, tz);

//             float angle_in_base = std::atan2(p_base.getY(), p_base.getX());
//             float distance = sqrt(pow(p_base.getX(), 2) + pow(p_base.getY(), 2));

//             if (distance <= max_range_) {
//                 points.push_back({angle_in_base, distance});
//             }
//         }

//     }
//     catch (const tf2::TransformException &e) {
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, 
//                             "Transform failed for %s at time %f: %s", 
//                             lidar_frame.c_str(), reference_time.seconds(), e.what());
//         return points;
//     }

//     return points;
// }

std::vector<std::pair<float, float>> ScanMergerV2::laserscan_to_point_at_time(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, 
    std::string lidar_frame,
    rclcpp::Time reference_time) {
    
    std::vector<std::pair<float, float>> points;

    // Calculate the actual time each point was captured
    rclcpp::Time scan_start_time(scan_msg->header.stamp);
    
    int size = scan_msg->ranges.size();
    for (int i = 0; i < size; i++){
        float r = scan_msg->ranges[i];

        if (r < scan_msg->range_min || r > scan_msg->range_max || !std::isfinite(r)){
            continue;
        }

        // Calculate the ACTUAL time this specific point was captured
        rclcpp::Time point_time = scan_start_time + rclcpp::Duration::from_seconds(i * scan_msg->time_increment);
        
        try {
            // Get transform at the EXACT time this point was captured
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                base_frame_,
                lidar_frame,
                point_time,  // Transform at the actual point capture time
                rclcpp::Duration::from_seconds(0.1)
            );

            float tx = transform.transform.translation.x;
            float ty = transform.transform.translation.y;
            float tz = transform.transform.translation.z;

            geometry_msgs::msg::Quaternion q = transform.transform.rotation;
            tf2::Quaternion quat(q.x, q.y, q.z, q.w);
            tf2::Matrix3x3 rot_matrix(quat);

            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            float x_lidar = r * std::cos(angle);
            float y_lidar = r * std::sin(angle);
            float z_lidar = 0.0;

            tf2::Vector3 p_lidar(x_lidar, y_lidar, z_lidar);
            tf2::Vector3 p_base = rot_matrix * p_lidar + tf2::Vector3(tx, ty, tz);

            float angle_in_base = std::atan2(p_base.getY(), p_base.getX());
            float distance = sqrt(pow(p_base.getX(), 2) + pow(p_base.getY(), 2));

            if (distance <= max_range_) {
                points.push_back({angle_in_base, distance});
            }

        }
        catch (const tf2::TransformException &e) {
            // Skip this point if transform fails
            continue;
        }
    }

    return points;
}


void ScanMergerV2::process_and_publish_scans(){
    // Already locked by caller
    // Don't require both scans - publish if we have at least one
    if (!scan1_ && !scan2_){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "No scan data available");
        return;
    }

    // Use the timestamp of the most recent scan for ALL transforms and final output
    rclcpp::Time reference_time;
    if (scan1_ && scan2_) {
        // Use the newer scan time as reference for temporal consistency
        rclcpp::Time time1(scan1_->header.stamp);
        rclcpp::Time time2(scan2_->header.stamp);
        reference_time = (time1 > time2) ? time1 : time2;
    } else if (scan1_) {
        reference_time = rclcpp::Time(scan1_->header.stamp);
    } else {
        reference_time = rclcpp::Time(scan2_->header.stamp);
    }

    std::vector<std::pair<float, float>> points1, points2;
    
    // Transform BOTH scans to the SAME reference time for consistency
    if (scan1_) {
        points1 = this->laserscan_to_point_at_time(scan1_, lidar_right_frame_, reference_time);
    }
    
    if (scan2_) {
        points2 = this->laserscan_to_point_at_time(scan2_, lidar_left_frame_, reference_time);
    }
    
    // Continue even if one transform failed, as long as we have some points
    if (points1.empty() && points2.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, 
                           "No valid transformed points available - transform failures");
        return;
    }

    // Combine points
    std::vector<std::pair<float, float>> all_points = points1;
    all_points.insert(all_points.end(), points2.begin(), points2.end());

    sensor_msgs::msg::LaserScan combined_scan;
    combined_scan.header.stamp = reference_time;  // Use the SAME reference time as transforms
    combined_scan.header.frame_id = base_frame_;
    combined_scan.angle_increment = angle_increment_;
    combined_scan.angle_min = angle_min_;
    combined_scan.angle_max = angle_max_;
    combined_scan.range_min = 0.5;
    combined_scan.range_max = max_range_;
    combined_scan.scan_time = 0.01;  // Faster scan time for reduced latency
    combined_scan.time_increment = 0.0;

    // Initialize ranges array with inf
    combined_scan.ranges.assign(num_points_, std::numeric_limits<float>::infinity());

    // Fill in ranges based on combined points with optimized binning
    for (const auto& [angle, distance] : all_points) {
        // Normalize angle to be within scan range
        float normalized_angle = angle;
        while (normalized_angle < angle_min_) normalized_angle += 2 * M_PI;
        while (normalized_angle > angle_max_) normalized_angle -= 2 * M_PI;
        
        // Skip if still outside range after normalization
        if (normalized_angle < angle_min_ || normalized_angle > angle_max_) {
            continue;
        }
        
        int idx = static_cast<int>((normalized_angle - angle_min_) / angle_increment_);
        if (idx >= 0 && idx < static_cast<int>(combined_scan.ranges.size())) {
            // Take the minimum distance if multiple points fall in the same bin
            combined_scan.ranges[idx] = std::min(combined_scan.ranges[idx], distance);
        }
    }

    // Publish immediately without any additional delay
    merged_scan_->publish(combined_scan);
    
    RCLCPP_DEBUG(this->get_logger(), "Published merged scan with %zu points at time %f", 
                all_points.size(), reference_time.seconds());
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMergerV2>());
  rclcpp::shutdown();
  return 0;
}