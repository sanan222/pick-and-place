/* cw2_class.cpp for Pick and Place using MoveIt!
*/

#include <cw2_class.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

// Namespace for robot-related constants
namespace RobotConstants {
    const double GRIPPER_OPEN = 0.08;     // Fully open gripper width in meters
    const double GRIPPER_CLOSED = 0.01;   // Fully closed gripper width in meters
    const double TIGHT_GRIP = 0.008;      // Tighter grip for secure holding

    // Approach and pickup distances
    const double APPROACH_OFFSET = 0.15;  // 15cm above object for approach
    const double PICKUP_OFFSET = 0.12;    // 12cm above object for pickup
    
    // Movement and planning parameters
    const double PLANNING_TIME = 20.0;
    const int PLANNING_ATTEMPTS = 10;
    const double MAX_VELOCITY = 0.30;
    const double MAX_ACCELERATION = 0.30;
    const int MAX_EXECUTION_ATTEMPTS = 3;
}

// Namespace for point cloud processing constants
namespace PCLConstants {
    const float VOXEL_LEAF_SIZE = 0.001f;  // Voxel grid leaf size (1mm)
    const double Z_THRESHOLD = 0.02;       // Height threshold for table filtering
    const float MIN_BRIGHTNESS = 0.1f;     // Minimum color brightness
    
}

// Namespace for color utilities
namespace ColorUtils {
    /**
     * Convert RGB to HSV color space
     * 
     * @param r Red component (0-255)
     * @param g Green component (0-255)
     * @param b Blue component (0-255)
     * @return HSV structure with h(0-360), s(0-1), v(0-1)
     */
    HSV rgb2hsv(float r, float g, float b) {
        r /= 255.0f;
        g /= 255.0f;
        b /= 255.0f;
        
        float max_val = std::max(std::max(r, g), b);
        float min_val = std::min(std::min(r, g), b);
        float diff = max_val - min_val;
        
        HSV hsv;
        hsv.v = max_val;
        
        if (max_val == 0.0f) {
            hsv.s = 0.0f;
        } else {
            hsv.s = diff / max_val;
        }
        
        if (diff == 0.0f) {
            hsv.h = 0.0f;
        } else if (max_val == r) {
            hsv.h = 60.0f * fmodf(((g - b) / diff), 6.0f);
        } else if (max_val == g) {
            hsv.h = 60.0f * (((b - r) / diff) + 2.0f);
        } else {
            hsv.h = 60.0f * (((r - g) / diff) + 4.0f);
        }
        
        if (hsv.h < 0.0f) {
            hsv.h += 360.0f;
        }
        
        return hsv;
    }
    
    /**
     * Check if a color in HSV space is greenish
     * 
     * @param h Hue (0-360)
     * @param s Saturation (0-1)
     * @param v Value/brightness (0-1)
     * @return True if the color is greenish
     */
    bool is_greenish(double h, double s, double v) {
        return (h >= 90 && h <= 150 && s > 0.2);
    }
    
    /**
     * Determine if a point's color matches a specific color category
     * 
     * @param hsv HSV values of the point
     * @param color_name Target color name ("red", "blue", "purple")
     * @return True if the point matches the color category
     */
    bool matches_color(const HSV& hsv, const std::string& color_name) {
        // Red: Hue near 0/360 with good saturation
        if (color_name == "red") {
            return ((hsv.h < 20.0f || hsv.h > 340.0f) && hsv.s > 0.4f && hsv.v > 0.3f);
        }
        // Blue: Hue around 220 (200-240) with good saturation
        else if (color_name == "blue") {
            return (hsv.h >= 200.0f && hsv.h <= 240.0f && hsv.s > 0.4f && hsv.v > 0.3f);
        }
        // Purple: Hue around 290 (270-310) with decent saturation
        else if (color_name == "purple") {
            return (hsv.h >= 270.0f && hsv.h <= 310.0f && hsv.s > 0.3f && hsv.v > 0.25f);
        }
        // Brown: Hue around 20-40 with lower saturation and value
        else if (color_name == "brown") {
            return (hsv.h >= 10.0f && hsv.h <= 40.0f && hsv.s > 0.2f && hsv.s < 0.7f && hsv.v > 0.2f && hsv.v < 0.6f);
        }
        
        return false;
    }
    
    /**
     * Check if a color in HSV space matches brown
     * 
     * @param hsv HSV values of the point
     * @return True if the color is brownish
     */
    bool is_brown(const HSV& hsv) {
        // Brown is typically a dark/desaturated orange or red
        // We're using a much wider range to ensure detection
        // Regular brown: hue 0-30 (red-orange), low-medium saturation, low-medium value
        return ((hsv.h >= 0.0f && hsv.h <= 50.0f) && 
                hsv.s >= 0.15f && hsv.s <= 0.8f && 
                hsv.v >= 0.15f && hsv.v <= 0.7f);
    }
}

// -----------------------------------------------------------------------------
// Class Implementation
// -----------------------------------------------------------------------------

/**
 * Class constructor for cw2 class
 */
cw2::cw2(ros::NodeHandle nh) : nh_(nh)
{
    // Advertise services for coursework tasks
    t1_service_ = nh_.advertiseService("/task1_start", &cw2::t1_callback, this);
    t2_service_ = nh_.advertiseService("/task2_start", &cw2::t2_callback, this);
    t3_service_ = nh_.advertiseService("/task3_start", &cw2::t3_callback, this);

    ROS_INFO("cw2 class initialized");
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    // Initialize the merged_cloud_pub_ with latching enabled
    merged_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("merged_point_cloud", 1, true);

    // Initialize transform buffer and listener
    tf_buffer_.reset(new tf2_ros::Buffer);
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
}

// -----------------------------------------------------------------------------
// Helper Functions for Robot Control
// -----------------------------------------------------------------------------

/**
 * Control the gripper to a target width
 * 
 * @param target_width Target width in meters
 * @return True if gripper movement was successful
 */
bool cw2::move_gripper(double target_width)
{
    // Constrain target width to valid range
    if (target_width > RobotConstants::GRIPPER_OPEN) {
        target_width = RobotConstants::GRIPPER_OPEN;
    }
    if (target_width < RobotConstants::GRIPPER_CLOSED) {
        target_width = RobotConstants::GRIPPER_CLOSED;
    }

    // Calculate joint values (half of width for each finger)
    double each_joint = target_width / 2.0;
    std::vector<double> gripper_joint_targets(2, each_joint);

    // Create hand move group and execute movement
    MoveGroupInterface hand_group("hand");
    hand_group.setJointValueTarget(gripper_joint_targets);

    Plan hand_plan;
    bool success = (hand_group.plan(hand_plan) == MoveItErrorCode::SUCCESS);
    
    if (success) {
        hand_group.move();
    }
    
    return success;
}

/**
 * Execute a pose target with retry attempts
 * 
 * @param move_group MoveGroupInterface to control
 * @param target Target pose to achieve
 * @param max_attempts Maximum number of planning/execution attempts
 * @return True if move was successful
 */
bool cw2::execute_pose_target(
    MoveGroupInterface &move_group,
    const geometry_msgs::Pose &target, 
    int max_attempts)
{
    for (int i = 0; i < max_attempts; i++) {
        move_group.setPoseTarget(target);
        Plan plan;
        
        if (move_group.plan(plan) == MoveItErrorCode::SUCCESS) {
            if (move_group.execute(plan) == MoveItErrorCode::SUCCESS) {
                return true;
            }
        }
        
        ROS_WARN("Attempt %d to move to target pose failed. Retrying...", i+1);
        ros::Duration(1.0).sleep();
    }
    
    return false;
}

/**
 * Initialize a MoveIt interface with standard parameters
 * 
 * @param group_name Name of the move group (default: "panda_arm")
 * @return Configured MoveGroupInterface object
 */
MoveGroupInterface cw2::initialize_move_group(const std::string& group_name) {
    MoveGroupInterface move_group(group_name);
    move_group.setPlanningTime(RobotConstants::PLANNING_TIME);
    move_group.setNumPlanningAttempts(RobotConstants::PLANNING_ATTEMPTS);
    move_group.setMaxVelocityScalingFactor(RobotConstants::MAX_VELOCITY);
    move_group.setMaxAccelerationScalingFactor(RobotConstants::MAX_ACCELERATION);
    return move_group;
}

/**
 * Store the current joint values for later use
 * 
 * @param move_group MoveGroupInterface to get joint values from
 * @return Vector of joint values
 */
std::vector<double> cw2::store_home_position(MoveGroupInterface& move_group) {
    std::vector<double> home_joint_values;
    move_group.getCurrentState()->copyJointGroupPositions(
        move_group.getCurrentState()->getJointModelGroup("panda_arm"), 
        home_joint_values);
    
    ROS_INFO("Stored home position: %zu joint values", home_joint_values.size());
    return home_joint_values;
}

/**
 * Return the robot to a stored home position using joint values
 * 
 * @param move_group MoveGroupInterface to control
 * @param home_joint_values Joint values defining the home position
 * @return True if successfully moved to home position
 */
bool cw2::return_to_home_position(
    MoveGroupInterface& move_group, 
    const std::vector<double>& home_joint_values)
{
    if (home_joint_values.empty()) {
        ROS_ERROR("Empty home joint values provided");
        return false;
    }
    
    ROS_INFO("Returning to home position...");
    move_group.setJointValueTarget(home_joint_values);
    
    Plan home_plan;
    bool success = (move_group.plan(home_plan) == MoveItErrorCode::SUCCESS);
    
    if (success && move_group.execute(home_plan) == MoveItErrorCode::SUCCESS) {
        ROS_INFO("Successfully returned to home position");
        return true;
    } else {
        ROS_ERROR("Failed to return to home position");
        return false;
    }
}

// Add a class member variable to store the detected orientation
double last_detected_orientation_deg_ = 0.0;
double opencv_detected_orientation_deg_ = 0.0;
bool orientation_was_detected_ = false;

// Add a class member variable to store the maximum obstacle height
double max_obstacle_height_ = 0.0;

/**
 * Process point cloud specifically for orientation detection
 * Preserves more of the object shape for better contour detection
 * 
 * @param input_cloud Raw input point cloud
 * @param object_center Approximate center of the object
 * @param search_radius Radius to search around the object center
 * @return Processed point cloud containing just the object
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cw2::process_cloud_for_orientation(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    const geometry_msgs::Point& object_center,
    double search_radius)
{
    if (!input_cloud || input_cloud->empty()) {
        ROS_ERROR("Invalid or empty input point cloud for orientation detection");
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }
    
    // Create output cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    try {
        // 1. Downsample the cloud (but with smaller leaf size to preserve more detail)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(input_cloud);
        sor.setLeafSize(0.0005f, 0.0005f, 0.0005f);  // 0.5mm voxel size (finer than usual)
        sor.filter(*cloud_downsampled);
        
        ROS_INFO("Downsampled cloud from %zu to %zu points for orientation detection", 
                 input_cloud->points.size(), cloud_downsampled->points.size());
        
        // 2. Filter points by distance from object center and height
        for (const auto& point : cloud_downsampled->points) {
            // Calculate distance from object center (XY plane only)
            double dx = point.x - object_center.x;
            double dy = point.y - object_center.y;
            double distance_xy = std::sqrt(dx*dx + dy*dy);
            
            // Keep points that are:
            // - Within search radius of object center (in XY plane)
            // - Above the table (Z threshold)
            // - Not too high above the object center
            if (distance_xy <= search_radius && 
                point.z > PCLConstants::Z_THRESHOLD &&
                point.z < object_center.z + 0.05) {  // Only consider points up to 5cm above center
                
                // Convert RGB to HSV for color filtering
                HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
                
                // Skip green points (table/background)
                if (!ColorUtils::is_greenish(hsv.h, hsv.s, hsv.v)) {
                    object_cloud->points.push_back(point);
                }
            }
        }
        
        object_cloud->width = object_cloud->points.size();
        object_cloud->height = 1;
        object_cloud->is_dense = true;
        
        ROS_INFO("Extracted %zu points for orientation detection", object_cloud->points.size());
        
        return object_cloud;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception during point cloud processing for orientation: %s", e.what());
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }
}

/**
 * Detect object orientation using OpenCV instead of PCA
 * 
 * @param cloud Point cloud of the object
 * @return Detected orientation in degrees (0-90 range)
 */
double cw2::detect_orientation_opencv(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        ROS_ERROR("Empty cloud provided for orientation detection");
        return 0.0;
    }
    
    // Create a 2D projection of the point cloud to an image
    int img_size = 300; // Size of the image
    double scale = 1000.0; // Scale factor to convert meters to pixels
    
    // Find min/max of the point cloud
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    
    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, (double)point.x);
        min_y = std::min(min_y, (double)point.y);
        max_x = std::max(max_x, (double)point.x);
        max_y = std::max(max_y, (double)point.y);
    }
    
    // Calculate the center of the object
    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;
    
    // Calculate the size of the object
    double object_width = max_x - min_x;
    double object_height = max_y - min_y;
    double object_size = std::max(object_width, object_height);
    
    // Adjust scale to ensure the object fits well in the image
    // We want the object to take up about 50% of the image to leave room for processing
    double target_size_pixels = img_size * 0.5;
    scale = target_size_pixels / object_size;
    
    ROS_INFO("Object size: %.3f x %.3f m, using scale: %.1f", object_width, object_height, scale);
    
    // Create a black image
    cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);
    
    // Store all projected points for direct bounding box calculation
    std::vector<cv::Point> all_points;
    
    // Project points to the image
    for (const auto& point : cloud->points) {
        // Center the point cloud and scale to image coordinates
        int img_x = (int)((point.x - center_x) * scale + img_size/2);
        // Flip Y axis to match the robot's coordinate system
        int img_y = (int)(-(point.y - center_y) * scale + img_size/2);
        
        // Store all valid points for direct bounding box calculation
        if (img_x >= 0 && img_x < img_size && img_y >= 0 && img_y < img_size) {
            image.at<uchar>(img_y, img_x) = 255; // Set pixel to white
            all_points.push_back(cv::Point(img_x, img_y));
        }
    }
    
    // Apply even more aggressive dilation to fill holes and connect nearby points
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9));
    cv::dilate(image, image, kernel, cv::Point(-1, -1), 5);
    
    // Apply light erosion to remove noise but preserve shape
    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::erode(image, image, kernel, cv::Point(-1, -1), 1);
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // If no contours found, but we have points, use the points directly
    cv::RotatedRect min_rect;
    if (contours.empty()) {
        if (all_points.empty()) {
            ROS_WARN("No points or contours found in the projected image");
            return 0.0;
        }
        // Use all points directly to compute the minimum area rectangle
        min_rect = cv::minAreaRect(all_points);
        ROS_INFO("Using direct point calculation for bounding box");
    } else {
        // Find the largest contour (by area)
        int largest_contour_idx = 0;
        double largest_area = 0.0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area > largest_area) {
                largest_area = area;
                largest_contour_idx = i;
            }
        }
        
        // Get the minimum area rectangle for the largest contour
        min_rect = cv::minAreaRect(contours[largest_contour_idx]);
    }
    
    // Get the angle from the rotated rectangle
    double angle = min_rect.angle;
    
    // Adjust angle based on width and height of the rectangle
    if (min_rect.size.width < min_rect.size.height) {
        angle += 0;  // No adjustment needed
    } else {
        angle += 90;  // Adjust to make it in [0, 90) range
    }
    
    // Normalize to [0, 90) range
    while (angle < 0) angle += 90;
    while (angle >= 90) angle -= 90;
    
    ROS_INFO("OpenCV detected orientation: %.2f degrees", angle);
    
    // Create a debug image for visualization
    cv::Mat debug_image = cv::Mat::zeros(img_size, img_size, CV_8UC3);
    
    // Draw the original point cloud projection in gray
    for (int y = 0; y < img_size; y++) {
        for (int x = 0; x < img_size; x++) {
            if (image.at<uchar>(y, x) > 0) {
                debug_image.at<cv::Vec3b>(y, x) = cv::Vec3b(100, 100, 100); // Gray
            }
        }
    }
    
    // Draw all original points in white to clearly see them
    for (const auto& pt : all_points) {
        if (pt.x >= 0 && pt.x < img_size && pt.y >= 0 && pt.y < img_size) {
            cv::circle(debug_image, pt, 1, cv::Scalar(255, 255, 255), -1);
        }
    }
    
    // Draw the contours in green if available
    if (!contours.empty()) {
        for (size_t i = 0; i < contours.size(); i++) {
            cv::drawContours(debug_image, contours, i, cv::Scalar(0, 255, 0), 1);
        }
    }
    
    // Draw the minimum area rectangle in red
    cv::Point2f rect_points[4];
    min_rect.points(rect_points);
    for (int i = 0; i < 4; i++) {
        cv::line(debug_image, rect_points[i], rect_points[(i+1)%4], cv::Scalar(0, 0, 255), 2);
    }
    
    // Draw the orientation line in blue
    cv::Point2f center = min_rect.center;
    double radian_angle = angle * CV_PI / 180.0;
    cv::Point2f direction(cos(radian_angle), sin(radian_angle));
    cv::Point2f end_point = center + direction * 50.0;
    cv::line(debug_image, center, end_point, cv::Scalar(255, 0, 0), 2);
    
    // Add text showing the detected angle
    std::string angle_text = "Angle: " + std::to_string(angle) + " deg";
    cv::putText(debug_image, angle_text, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    
    // Publish the debug image to a ROS topic
    static ros::Publisher debug_img_pub;
    if (!debug_img_pub) {
        debug_img_pub = nh_.advertise<sensor_msgs::Image>("/orientation_detection/debug_image", 1);
    }
    
    // Convert OpenCV image to ROS message
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "debug_image";
    
    cv_bridge::CvImage cv_image(header, "bgr8", debug_image);
    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    
    // Publish the image
    debug_img_pub.publish(ros_image);
    ROS_INFO("Published debug image to /orientation_detection/debug_image");
    
    return angle;
}

/**
 * Execute a complete pick and place operation
 * 
 * @param move_group MoveGroupInterface to control
 * @param params Pick and place parameters
 * @return True if the operation was successful
 */
bool cw2::execute_pick_and_place(
    MoveGroupInterface& move_group,
    const PickPlaceParams& params)
{
    // Visualize object center with yellow marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "panda_link0";
    marker.header.stamp = ros::Time::now();
    marker.ns = "object_centers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = params.object_pose.position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(10.0);
    marker_pub_.publish(marker);
    
    ROS_INFO("Published yellow marker at object center point: (%.3f, %.3f, %.3f)",
             params.object_pose.position.x, params.object_pose.position.y, params.object_pose.position.z);

    // 1. First, detect object orientation using viewpoint
    geometry_msgs::Pose viewpoint_pose;
    viewpoint_pose.position.x = params.object_pose.position.x;
    viewpoint_pose.position.y = params.object_pose.position.y;
    viewpoint_pose.position.z = params.object_pose.position.z + 0.55; // 50cm above object
    
    // Set orientation to look at the object from an angle
    tf2::Quaternion look_angle_q;
    look_angle_q.setRPY(M_PI, 0, -M_PI/4); // 45 degree tilt to see orientation better
    viewpoint_pose.orientation = tf2::toMsg(look_angle_q);
    
    ROS_INFO("Moving to viewpoint position to detect object orientation...");
    if (!execute_pose_target(move_group, viewpoint_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_WARN("Failed to move to viewpoint position. Will use default orientation.");
    } else {
        // Wait for camera to stabilize
        ros::Duration(2.0).sleep();
    }
    
    // Get point cloud data to detect orientation
    double object_yaw = 0.0; // Default orientation
    orientation_was_detected_ = false;
    
    sensor_msgs::PointCloud2ConstPtr cloud_msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/r200/camera/depth_registered/points", nh_, ros::Duration(2.0));
            
    if (cloud_msg) {
        // Process point cloud to detect orientation
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*cloud_msg, *cloud);
        
        // Transform the point cloud from the camera frame to the robot frame
        try {
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
                "panda_link0", cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(1.0));
                
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl_ros::transformPointCloud(*cloud, *cloud_transformed, transform_stamped.transform);
            cloud = cloud_transformed;
            
            // Process the point cloud specifically for orientation detection
            // Use a larger search radius (10cm) to ensure we get the whole object
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud = 
                process_cloud_for_orientation(cloud, params.object_pose.position, 0.10);
            
            if (object_cloud && !object_cloud->empty()) {
                // Use OpenCV for orientation detection
                double opencv_angle_deg = detect_orientation_opencv(object_cloud);
                
                // Calculate object_yaw_deg based on shape type
                double object_yaw_deg = 0.0;

                // Continue with orientation detection as before
                if (params.shape_type == "nought") {
                    // For nought: 90 - detected angle
                    object_yaw_deg = 90.0 - opencv_angle_deg;
                    ROS_INFO("Nought object: OpenCV angle = %.2f°, Adjusted yaw = %.2f°", 
                             opencv_angle_deg, object_yaw_deg);
                } 
                else if (params.shape_type == "cross") {
                    // For cross: different rules based on detected angle
                    if (opencv_angle_deg > 45.0) {
                        // If angle > 45: angle - 45, then 90 - result
                        object_yaw_deg = 90.0 - (opencv_angle_deg - 45.0);
                    } else {
                        // If angle <= 45: angle + 45, then 90 - result
                        object_yaw_deg = 90.0 - (opencv_angle_deg + 45.0);
                    }
                    ROS_INFO("Cross object: OpenCV angle = %.2f°, Adjusted yaw = %.2f°", 
                             opencv_angle_deg, object_yaw_deg);
                }
                else {
                    // Default case for other shapes
                    object_yaw_deg = opencv_angle_deg;
                    ROS_INFO("Unknown shape: Using direct OpenCV angle = %.2f°", object_yaw_deg);
                }
                
                // Convert back to radians
                object_yaw = object_yaw_deg * M_PI / 180.0;
                
                // Store the detected orientation in the class member variable
                last_detected_orientation_deg_ = object_yaw_deg;
                opencv_detected_orientation_deg_ = opencv_angle_deg;
                orientation_was_detected_ = true;
                
                ROS_INFO("OpenCV detected object orientation: %.2f degrees", opencv_angle_deg);
                ROS_INFO("Object yaw for the gripper turn: %.2f degrees", object_yaw_deg);
            } else {
                ROS_WARN("Failed to extract object cloud for orientation detection");
            }
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not transform point cloud: %s", ex.what());
        }
    } else {
        ROS_WARN("No point cloud received for orientation detection");
    }
    
    // 2. Adjust orientation based on detected yaw
    geometry_msgs::Quaternion adjusted_orientation = params.object_pose.orientation;
    
    if (orientation_was_detected_) {
        // Base yaw angle (315 degrees) plus detected object orientation
        double yaw_deg = 315.0;
        double object_yaw_deg = last_detected_orientation_deg_;
        
        // Add detected orientation as an offset
        yaw_deg += object_yaw_deg;
        double yaw_rad = yaw_deg * M_PI / 180.0;
        
        ROS_INFO("Gripper orientation: Base yaw = 315.0°, Object yaw = %.2f°, Final yaw = %.2f°", 
                 object_yaw_deg, yaw_deg);
        
        // RPY (Roll: 180°, Pitch: 0°, Yaw: adjusted)
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, yaw_rad);
        adjusted_orientation = tf2::toMsg(q);
    } else {
        // Fallback to fixed orientation if detection failed
        double default_yaw_deg = 315.0;
        double default_yaw_rad = default_yaw_deg * M_PI / 180.0;
        
        ROS_WARN("Orientation detection failed. Using default orientation (yaw = %.1f°)", default_yaw_deg);
        
        // RPY (Roll: 180°, Pitch: 0°, Yaw: default)
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, default_yaw_rad);
        adjusted_orientation = tf2::toMsg(q);
    }
    
    // 3. Open the gripper before approaching the object
    ROS_INFO("Opening gripper...");
    if (!move_gripper(RobotConstants::GRIPPER_OPEN)) {
        ROS_ERROR("Failed to open gripper");
        return false;
    }
    
    // 4. Create the approach pose with the adjusted orientation
    geometry_msgs::Pose approach_pose = params.object_pose;
    approach_pose.position.z += params.approach_offset;

    // Apply an offset depending on shape type and detected size
    double x_offset = 0.0;
    double y_offset = 0.0;
    
    // Scale factor based on detected size (40mm is the reference)
    double size_scale = last_detected_size_mm_ / 40.0;
    ROS_INFO("Using size scale factor: %.2f based on %dmm object", size_scale, last_detected_size_mm_);

    if (params.shape_type == "nought") {
        // For a nought, pick from a side
        ROS_INFO("Object type: nought");
        if (orientation_was_detected_) {
            // Apply offset based on detected orientation, scaled by size
            double orientation_rad = last_detected_orientation_deg_ * M_PI / 180.0;
            y_offset = -0.08 * std::cos(orientation_rad) * size_scale;
            x_offset = 0.08 * std::sin(orientation_rad) * size_scale;
            ROS_INFO("Using detected orientation: %.2f degrees for nought offset (x=%.3f, y=%.3f)", 
                     last_detected_orientation_deg_, x_offset, y_offset);
        } else {
            // Default offset if orientation not detected, scaled by size
            y_offset = 0.08 * size_scale;
            ROS_INFO("Using default offset for nought (y=%.3f)", y_offset);
        }
    } else if (params.shape_type == "cross") {
        // For a cross, pick from a corner
        ROS_INFO("Object type: cross");
        if (orientation_was_detected_) {
            // Apply offset based on detected orientation, scaled by size
            double orientation_rad = last_detected_orientation_deg_ * M_PI / 180.0;
            x_offset = 0.05 * std::cos(orientation_rad) * size_scale;
            y_offset = 0.05 * std::sin(orientation_rad) * size_scale;
            ROS_INFO("Using detected orientation: %.2f degrees for cross offset (x=%.3f, y=%.3f)", 
                     last_detected_orientation_deg_, x_offset, y_offset);
        } else {
            // Default offset if orientation not detected, scaled by size
            x_offset = 0.05 * size_scale;
            ROS_INFO("Using default offset for cross (x=%.3f)", x_offset);
        }
    }

    // Apply offsets
    approach_pose.position.x += x_offset;
    approach_pose.position.y += y_offset;

    // Ros info x and y offsets
    ROS_INFO("X offset: %.3f, Y offset: %.3f", x_offset, y_offset);

    // Apply an offset depending on shape type

    approach_pose.orientation = adjusted_orientation;
    
    // 5. Move to the approach pose
    ROS_INFO("Moving to approach pose (%.3f, %.3f, %.3f)...",
            approach_pose.position.x, approach_pose.position.y, approach_pose.position.z);
    
    if (!execute_pose_target(move_group, approach_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to move to approach pose");
        return false;
    }
    
    // 6. Create and move to the pickup pose
    geometry_msgs::Pose pickup_pose = approach_pose;
    pickup_pose.position.z -= 0.06;
    pickup_pose.orientation = adjusted_orientation;
    
    ROS_INFO("Moving to pickup pose (%.3f, %.3f, %.3f)...",
            pickup_pose.position.x, pickup_pose.position.y, pickup_pose.position.z);
    
    if (!execute_pose_target(move_group, pickup_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to move to pickup pose");
        return false;
    }
    
    // 7. Close the gripper to grasp the object
    ROS_INFO("Closing gripper to grasp object...");
    if (!move_gripper(RobotConstants::TIGHT_GRIP)) {
        ROS_ERROR("Failed to close gripper");
        return false;
    }
    
    // Allow time for grip to solidify
    ros::Duration(0.5).sleep();
    
    // 8. Lift the object by moving back to the approach pose
    ROS_INFO("Lifting the object...");
    
    // Adjust the approach pose height to clear obstacles if needed
    double min_lift_height = max_obstacle_height_ + 0.35; // 20cm clearance
    if (approach_pose.position.z < min_lift_height) {
        ROS_INFO("Adjusting lift height to clear obstacles (%.3f -> %.3f m)",
                approach_pose.position.z, min_lift_height);
        approach_pose.position.z = min_lift_height;
    }
    
    if (!execute_pose_target(move_group, approach_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to lift the object");
        return false;
    }
    
    // 9. Create the place approach pose with the adjusted orientation
    // First create an intermediate pose: same x,y as target but keeping the current z height
    geometry_msgs::Pose place_approach_pose_safe = params.place_pose;
    place_approach_pose_safe.position.z = approach_pose.position.z; // Keep safe height
    place_approach_pose_safe.orientation = adjusted_orientation;
    
    // Move to the safe approach position first (horizontally at safe height)
    ROS_INFO("Moving horizontally to position above target (%.3f, %.3f, %.3f)...",
            place_approach_pose_safe.position.x, place_approach_pose_safe.position.y, 
            place_approach_pose_safe.position.z);
    
    if (!execute_pose_target(move_group, place_approach_pose_safe, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to move to safe approach position");
        return false;
    }
    
    // Now create the final approach pose with the proper height offset
    geometry_msgs::Pose place_approach_pose = params.place_pose;
    place_approach_pose.position.z += params.place_approach_offset;
    place_approach_pose.orientation = adjusted_orientation;
    
    // 10. Move down to the final place approach pose
    ROS_INFO("Moving down to place approach pose (%.3f, %.3f, %.3f)...",
            place_approach_pose.position.x, place_approach_pose.position.y, 
            place_approach_pose.position.z);
    
    if (!execute_pose_target(move_group, place_approach_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to move to place approach pose");
        return false;
    }

    // 11. Release the object
    ROS_INFO("Releasing object...");
    if (!move_gripper(RobotConstants::GRIPPER_OPEN)) {
        ROS_ERROR("Failed to open gripper to release object");
        return false;
    }
    
    // Allow time for release
    ros::Duration(0.5).sleep();
    
    ROS_INFO("Pick and place operation completed successfully");
    return true;
}

// -----------------------------------------------------------------------------
// Helper Functions for Point Cloud Processing
// -----------------------------------------------------------------------------

/**
 * Apply voxel grid downsampling to a point cloud
 * 
 * @param cloud Input point cloud to be downsampled
 * @param cloud_filtered Output downsampled point cloud
 */
void cw2::apply_voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(PCLConstants::VOXEL_LEAF_SIZE, 
                   PCLConstants::VOXEL_LEAF_SIZE, 
                   PCLConstants::VOXEL_LEAF_SIZE);
    sor.filter(*cloud_filtered);
    
    ROS_INFO("Downsampled cloud from %zu to %zu points", 
             cloud->points.size(), cloud_filtered->points.size());
}

/**
 * Remove green background from a point cloud using HSV color space
 * 
 * @param cloud Point cloud to be filtered in-place
 */
void cw2::remove_green_background(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    size_t original_size = cloud->points.size();
    
    for (const auto &point : cloud->points)
    {
        // Skip points too close to the table
        if (point.z < PCLConstants::Z_THRESHOLD) {
            continue;
        }
        
        // Convert RGB to HSV for better color filtering
        HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
        
        // Check for green (typically 80-170° hue) and dark colors
        bool is_green = (hsv.h >= 80.0f && hsv.h <= 170.0f && 
                       hsv.s > 0.2f && hsv.v > 0.2f);
        bool is_too_dark = (hsv.v < PCLConstants::MIN_BRIGHTNESS);
        
        // Keep point only if it's not green and not too dark
        if (!is_green && !is_too_dark) {
            cloud_filtered->points.push_back(point);
        }
    }
    
    cloud_filtered->width = static_cast<uint32_t>(cloud_filtered->points.size());
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = true;
    
    *cloud = *cloud_filtered;
    
    ROS_INFO("Background removal: %zu → %zu points (removed %zu points)",
             original_size, cloud->points.size(), original_size - cloud->points.size());
}

////////////////////////////////////////////////////////////////////////////////////////////
// Task 1
////////////////////////////////////////////////////////////////////////////////////////////

bool cw2::t1_callback(cw2_world_spawner::Task1Service::Request &request,
                      cw2_world_spawner::Task1Service::Response &response) 
{
    ROS_INFO("Task 1: Starting pick and place operation for shape: %s", 
             request.shape_type.c_str());

    // 1) Initialize the MoveIt! interface for the "panda_arm"
    MoveGroupInterface move_group = initialize_move_group("panda_arm");
    
    // Store home position for returning later if needed
    std::vector<double> home_joint_values = store_home_position(move_group);

    // 2) Execute pick and place operation
    PickPlaceParams params;
    
    // Copy the centroid (object_point) into params
    params.object_pose.position = request.object_point.point;
    
    // Set the shape type
    params.shape_type = request.shape_type;

    // Set gripper orientation
    // Base yaw angle (315 degrees) - orientation will be detected in execute_pick_and_place
    double yaw_deg = 315.0;
    double yaw_rad = yaw_deg * M_PI / 180.0;
    
    // RPY (Roll: 180°, Pitch: 0°, Yaw: adjusted)
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, yaw_rad);
    params.object_pose.orientation = tf2::toMsg(q);

    // 4) Set the basket pose from goal_point
    params.place_pose.position = request.goal_point.point;

    // ros info the place pose
    ROS_INFO("Place pose: (%.3f, %.3f, %.3f)",
             params.place_pose.position.x,
             params.place_pose.position.y,
             params.place_pose.position.z);
             
             
    // The basket orientation is typically identity (0,0,0,1)
    params.place_pose.orientation.w = 1.0;

    // 5) Set approach/pick offsets
    params.approach_offset       = 0.20;
    params.pickup_offset         = 0.14;   
    params.place_approach_offset = 0.28; 

    // Debug logs
    ROS_INFO("Adjusted object pose for pick: (%.3f, %.3f, %.3f)",
             params.object_pose.position.x,
             params.object_pose.position.y,
             params.object_pose.position.z);

    // 6) Execute pick & place
    bool success = execute_pick_and_place(move_group, params);
    if (success) {
        ROS_INFO("Task 1: Successfully picked up %s and placed it in basket",
                 request.shape_type.c_str());
    } else {
        ROS_ERROR("Task 1: Failed to pick/place %s shape", 
                  request.shape_type.c_str());
    }
    
    // 7) Return to home position regardless of pick and place success
    ROS_INFO("Returning to home position...");
    return_to_home_position(move_group, home_joint_values);
    
    return success;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Task 2
////////////////////////////////////////////////////////////////////////////////////////////

bool
cw2::t2_callback(cw2_world_spawner::Task2Service::Request &request,
  cw2_world_spawner::Task2Service::Response &response)
{
  ROS_INFO("Task 2: Shape matching task started");

  // Store processed clouds and their point counts
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> shape_clouds;
  std::vector<float> center_densities;
  
  // Get the reference points and mystery point
  const auto& ref_points = request.ref_object_points;
  const auto& mystery_point = request.mystery_object_point;
  
  if (ref_points.size() != 2) {
    ROS_ERROR("Task 2: Expected 2 reference points, got %zu", ref_points.size());
    response.mystery_object_num = -1;
    return false;
  }
  
  ROS_INFO("Task 2: Received 2 reference points and 1 mystery point");
  ROS_INFO("Reference 0: (%.3f, %.3f, %.3f)", 
           ref_points[0].point.x, ref_points[0].point.y, ref_points[0].point.z);
  ROS_INFO("Reference 1: (%.3f, %.3f, %.3f)", 
           ref_points[1].point.x, ref_points[1].point.y, ref_points[1].point.z);
  ROS_INFO("Mystery: (%.3f, %.3f, %.3f)", 
           mystery_point.point.x, mystery_point.point.y, mystery_point.point.z);
  
  // Create MoveGroupInterface for the robot arm
  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);

  // Store home position for returning later if needed
  std::vector<double> home_joint_values = store_home_position(move_group);

  // Define fixed orientation looking down at the table
  geometry_msgs::Quaternion looking_down_orientation;
  tf2::Quaternion look_down_q;
  look_down_q.setRPY(M_PI, 0, -M_PI/4); // Look straight down
  looking_down_orientation = tf2::toMsg(look_down_q);
  
  // Combine reference points and mystery point for processing
  std::vector<geometry_msgs::PointStamped> all_points = ref_points;
  all_points.push_back(mystery_point);
  
  // Process each point by moving to it and capturing point cloud
  for (size_t i = 0; i < all_points.size(); i++) {
    const auto& point = all_points[i];
    std::string point_type = (i < 2) ? "Reference " + std::to_string(i + 1) : "Mystery";  // Changed indexing
    
    // Create view pose with offset in camera direction
    geometry_msgs::Pose view_pose;
    view_pose.position.x = point.point.x;
    view_pose.position.y = point.point.y;
    view_pose.position.z = point.point.z + 0.42;
    view_pose.orientation = looking_down_orientation;
    
    // Move to viewing position
    ROS_INFO("Task 2: Moving to view position for %s", point_type.c_str());
    if (!execute_pose_target(move_group, view_pose, 3)) {
      ROS_ERROR("Task 2: Failed to move to view position for %s", point_type.c_str());
      if (i < 2) {
        response.mystery_object_num = -1;
        return false;
      }
      continue;
    }
    
    // Wait for robot to stabilize
    ros::Duration(2.0).sleep();
    
    // Capture point cloud
    sensor_msgs::PointCloud2ConstPtr cloud_msg = 
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
        "/r200/camera/depth_registered/points", nh_, ros::Duration(5.0));

    ros::Duration(3.0).sleep();
    
    if (!cloud_msg) {
      ROS_ERROR("Task 2: No point cloud received for %s", point_type.c_str());
      if (i < 2) {
        response.mystery_object_num = -1;
        return false;
      }
      continue;
    }
    
    // Convert to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Transform the point cloud to the common frame
    std::string target_frame = "panda_link0";
    geometry_msgs::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
        target_frame,
        cloud_msg->header.frame_id,
        ros::Time(0),
        ros::Duration(3.0));
      
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl_ros::transformPointCloud(*cloud, *cloud_transformed, transform_stamped.transform);
      cloud = cloud_transformed;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Task 2: Could not transform point cloud: %s", ex.what());
      if (i < 2) {
        response.mystery_object_num = -1;
        return false;
      }
      continue;
    }
    
    // Extract object from point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    double search_radius = 0.1;
    
    // Extract points within radius of the object center
    for (const auto& pcl_point : cloud->points) {
      double dx = pcl_point.x - point.point.x;
      double dy = pcl_point.y - point.point.y;
      double dz = pcl_point.z - point.point.z;
      double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
      
      if (distance <= search_radius) {
        object_cloud->points.push_back(pcl_point);
      }
    }
    
    object_cloud->width = object_cloud->points.size();
    object_cloud->height = 1;
    object_cloud->is_dense = true;
    
    if (object_cloud->points.empty()) {
      ROS_ERROR("Task 2: No points found for %s object", point_type.c_str());
      if (i < 2) {
        response.mystery_object_num = -1;
        return false;
      }
      continue;
    }
    
    // Clean up point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    apply_voxel_filter(object_cloud, filtered_cloud);
    remove_green_background(filtered_cloud);
    
    // Store processed cloud
    shape_clouds.push_back(filtered_cloud);
    
    // Calculate center density
    int total_points = filtered_cloud->points.size();
    int center_points = 0;
    double center_x = point.point.x;
    double center_y = point.point.y;
    double center_radius = 0.02; // 2cm radius for center region
    
    for (const auto& pcl_point : filtered_cloud->points) {
      double dx = pcl_point.x - center_x;
      double dy = pcl_point.y - center_y;
      if (std::sqrt(dx*dx + dy*dy) <= center_radius) {
        center_points++;
      }
    }
    
    // Calculate center density based on point count rules
    bool use_constant_denominator = (total_points < 2100 || total_points > 6000);
    int denominator;
    if (total_points < 2100) {
      denominator = 3500;
    } else if (total_points > 6000) {
      denominator = 6000;
    } else {
      denominator = total_points;
    }
    
    float center_density = (float)center_points / denominator;
    center_densities.push_back(center_density);
    
    ROS_INFO("%s - Total points: %d, Center points: %d, Center density: %.4f",
             point_type.c_str(), total_points, center_points, center_density);
    
    // Return to home position
    return_to_home_position(move_group, home_joint_values);
  }
  
  // Check if we successfully processed all shapes
  if (shape_clouds.size() < 3 || center_densities.size() < 3) {
    ROS_ERROR("Task 2: Failed to process all shapes");
    response.mystery_object_num = -1;
    return false;
  }
  
  // Classify shapes based on center density
  std::vector<std::string> shape_types;
  for (float density : center_densities) {
    shape_types.push_back(density < 0.045 ? "nought" : "cross");
  }
  
  ROS_INFO("Shape classifications:");
  ROS_INFO("Reference 1: %s (density: %.4f)", shape_types[0].c_str(), center_densities[0]);  // Changed from Reference 0 to 1
  ROS_INFO("Reference 2: %s (density: %.4f)", shape_types[1].c_str(), center_densities[1]);  // Changed from Reference 1 to 2
  ROS_INFO("Mystery: %s (density: %.4f)", shape_types[2].c_str(), center_densities[2]);
  
  // Match mystery shape with reference shape
  if (shape_types[2] == shape_types[0]) {
    ROS_INFO("Task 2: Mystery shape matches reference 1");  // Changed from reference 0 to 1
    response.mystery_object_num = 1;  // Changed from 0 to 1
  } else {
    ROS_INFO("Task 2: Mystery shape matches reference 2");  // Changed from reference 1 to 2
    response.mystery_object_num = 2;  // Changed from 1 to 2
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////
// Task 3
////////////////////////////////////////////////////////////////////////////////////////////

bool
cw2::t3_callback(cw2_world_spawner::Task3Service::Request &request,
  cw2_world_spawner::Task3Service::Response &response)
{
  ROS_INFO("The coursework solving callback for task 3 has been triggered");

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  // 1. Initialize the MoveIt! interface for the "panda_arm"
  MoveGroupInterface move_group = initialize_move_group("panda_arm");
  
  // Store home position for returning later
  std::vector<double> home_joint_values = store_home_position(move_group);

  // 2. Define fixed orientation looking down at the table
  geometry_msgs::Quaternion looking_down_orientation;
  tf2::Quaternion look_down_q;
  look_down_q.setRPY(M_PI, 0, -M_PI/4); // Look straight down
  looking_down_orientation = tf2::toMsg(look_down_q);
  
  // 3. Define the center and radius for the circular path
  double center_x = 0.0;
  double center_y = 0.0;
  double radius = 0.46;  // 46cm from center
  double height = 0.80;  // 80cm high
  
  // 4. Define number of viewpoints for 360-degree rotation
  const int num_viewpoints = 8;  // 8 viewpoints (45 degrees apart)

  // Vector to store point clouds from each viewpoint
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> viewpoint_clouds;
  
  ROS_INFO("Starting 360-degree scan with %d viewpoints...", num_viewpoints);
  
  // 5. Move to each viewpoint in a circular pattern
  for (int i = 0; i < num_viewpoints; i++) {
    // Calculate angle in radians
    double angle = i * (2 * M_PI / num_viewpoints);
    double angle_deg = angle * 180.0 / M_PI;
    
    // Calculate position on the circle
    double x = center_x + radius * cos(angle);
    double y = center_y + radius * sin(angle);
    
    // Create the viewpoint pose
    geometry_msgs::Pose viewpoint_pose;
    viewpoint_pose.position.x = x;
    viewpoint_pose.position.y = y;
    viewpoint_pose.position.z = height;
    viewpoint_pose.orientation = looking_down_orientation;  // Fixed orientation
    
    // Create viewpoint name
    std::string viewpoint_name = "viewpoint_" + std::to_string(i+1);
    
    // Log the viewpoint position
    ROS_INFO("Moving to %s (angle: %.1f°, x=%.2f, y=%.2f, z=%.2f)...",
             viewpoint_name.c_str(), angle_deg, x, y, height);
    
    // Move to the viewpoint position
    if (!execute_pose_target(move_group, viewpoint_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
      ROS_ERROR("Failed to move to %s", viewpoint_name.c_str());
      // Continue to next viewpoint instead of failing completely
      continue;
    }
    
    // Wait to stabilize
    ros::Duration(3.0).sleep();  // Sleep for 3 seconds to stabilize
    
    // Get point cloud data
    ROS_INFO("Capturing point cloud at %s...", viewpoint_name.c_str());
    
    sensor_msgs::PointCloud2ConstPtr cloud_msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/r200/camera/depth_registered/points", nh_, ros::Duration(2.0));
            
    if (!cloud_msg) {
      ROS_ERROR("No point cloud received at %s", viewpoint_name.c_str());
      continue;
    }
    
    // Process point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Transform the point cloud from the camera frame to the robot frame
    try {
      geometry_msgs::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
          "panda_link0", cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(1.0));
          
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl_ros::transformPointCloud(*cloud, *cloud_transformed, transform_stamped.transform);
      cloud = cloud_transformed;

      // Store the point cloud in the vector
      viewpoint_clouds.push_back(cloud);
      
    } catch (tf2::TransformException &ex) {
      ROS_WARN("Could not transform point cloud: %s", ex.what());
    }
  }

  return_to_home_position(move_group, home_joint_values);

  // After completing the circular motion scans, perform additional corner checks
  ROS_INFO("Starting additional corner checks with provided target positions...");

  // Define the four target corner positions using the given coordinates
  std::vector<geometry_msgs::Point> target_positions;
  {
      geometry_msgs::Point pt;
      pt.x = 0.40; pt.y = -0.35; pt.z = 0.65;
      target_positions.push_back(pt);
      pt.x = -0.05; pt.y = -0.45; pt.z = 0.55;
      target_positions.push_back(pt);
      pt.x = -0.45; pt.y = -0.30; pt.z = 0.55;
      target_positions.push_back(pt);
      pt.x = -0.45; pt.y = 0.45; pt.z = 0.55;
      target_positions.push_back(pt);
      pt.x = 0.05; pt.y = 0.45; pt.z = 0.55;
      target_positions.push_back(pt);
      pt.x = 0.40; pt.y =  0.35; pt.z = 0.65;
      target_positions.push_back(pt);
  }

  // Iterate over each target coordinate
  for (size_t i = 0; i < target_positions.size(); i++) {
      geometry_msgs::Pose target_pose;
      target_pose.position = target_positions[i];
      // Use the same fixed "looking down" orientation as before
      target_pose.orientation = looking_down_orientation;
      std::string corner_name = "corner_" + std::to_string(i + 1);

      ROS_INFO("Moving to %s (x=%.2f, y=%.2f, z=%.2f)...", corner_name.c_str(),
               target_pose.position.x, target_pose.position.y, target_pose.position.z);

      // Move to the target corner position
      if (!execute_pose_target(move_group, target_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
          ROS_ERROR("Failed to move to %s", corner_name.c_str());
          continue;  // Skip this coordinate if movement fails
      }
      ros::Duration(3.0).sleep();  // Allow the robot to stabilize

      // Capture the point cloud at the current corner
      ROS_INFO("Capturing point cloud at %s...", corner_name.c_str());
      sensor_msgs::PointCloud2ConstPtr cloud_msg =
          ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
              "/r200/camera/depth_registered/points", nh_, ros::Duration(2.0));
      if (!cloud_msg) {
          ROS_ERROR("No point cloud received at %s", corner_name.c_str());
          continue;
      }
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::fromROSMsg(*cloud_msg, *cloud);
      try {
          // Transform the captured cloud to the "panda_link0" frame
          geometry_msgs::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
              "panda_link0", cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(1.0));
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
          pcl_ros::transformPointCloud(*cloud, *cloud_transformed, transform_stamped.transform);
          cloud = cloud_transformed;

          // Append this cloud to the vector for later merging
          viewpoint_clouds.push_back(cloud);
      } catch (tf2::TransformException &ex) {
          ROS_WARN("Could not transform point cloud at %s: %s", corner_name.c_str(), ex.what());
      }
  }

  // Merge all captured point clouds (from circular motion and corner checks)
  for (const auto& cloud : viewpoint_clouds) {
      *merged_cloud += *cloud;
  }
  merged_cloud->header.frame_id = "panda_link0";
    
    // Return to home position
    return_to_home_position(move_group, home_joint_values);

  // Publish the merged point cloud
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*merged_cloud, output_msg);
  output_msg.header.frame_id = "panda_link0";
  output_msg.header.stamp = ros::Time::now();
  merged_cloud_pub_.publish(output_msg);
  ROS_INFO("Published raw merged point cloud with %zu points", merged_cloud->points.size());

  // Define DetectedObject structure
  struct DetectedObject {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
      std::string color;
      std::string shape;
      geometry_msgs::Point centroid;
  };
  
  // Filter out green points first to reduce data size for clustering
  ROS_INFO("Filtering out green points from point cloud...");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  int green_points = 0;
  int nan_points = 0;
  int total_points = merged_cloud->points.size();
  
  // Manual filtering to remove ONLY green points and invalid points
  for (const auto& point : merged_cloud->points) {
      // Skip invalid points (NaN or Inf)
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          nan_points++;
          continue;
      }
      
      // Convert RGB to HSV for better color filtering
      HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
      
      // Check if point is green (table) and skip
      if (ColorUtils::is_greenish(hsv.h, hsv.s, hsv.v)) {
          green_points++;
          continue;
      }
      
      // Keep non-green valid points
      filtered_cloud->points.push_back(point);
  }
  
  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;
  
  ROS_INFO("Filtered point cloud from %zu to %zu points (removed %d green points, %d NaN/Inf points)", 
          merged_cloud->points.size(), filtered_cloud->points.size(), 
          green_points, nan_points);
  
  // Apply downsampling using voxel grid filter to speed up clustering
  ROS_INFO("Applying downsampling to filtered cloud...");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(filtered_cloud);
  vg.setLeafSize(0.002f, 0.002f, 0.002f);  // Reduced from 5mm to 2mm voxel size for more detail
  vg.filter(*downsampled_cloud);
    
    ROS_INFO("Downsampled cloud from %zu to %zu points", 
          filtered_cloud->points.size(), downsampled_cloud->points.size());
  
  // Additional filtering to ensure no NaN/Inf values remain after downsampling
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr valid_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (const auto& point : downsampled_cloud->points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
          valid_cloud->points.push_back(point);
      }
  }
  valid_cloud->width = valid_cloud->points.size();
  valid_cloud->height = 1;
  valid_cloud->is_dense = true;
  
  if (valid_cloud->points.size() < downsampled_cloud->points.size()) {
      ROS_INFO("Removed %zu remaining invalid points after downsampling", 
               downsampled_cloud->points.size() - valid_cloud->points.size());
  }
  
  // Extract clusters from the valid cloud for better shape analysis
  ROS_INFO("Starting clustering on the valid point cloud...");
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  
  try {
      tree->setInputCloud(valid_cloud);
      ROS_INFO("Created KD-tree for clustering successfully");
  } catch (const std::exception& e) {
      ROS_ERROR("Failed to create KD-tree: %s", e.what());
      return false;
  }
  
  // Extract clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  
  // Starting with more relaxed parameters for large raw clouds
  ROS_INFO("Setting clustering parameters: tolerance=0.005m, min_points=1000, max_points=300000");
  ec.setClusterTolerance(0.005);  // 5mm tolerance
  ec.setMinClusterSize(1000);     // Minimum 1000 points for valid shapes
  ec.setMaxClusterSize(300000);   // Maximum points per cluster (300000 is reasonable)
  ec.setSearchMethod(tree);
  ec.setInputCloud(valid_cloud);
  
  ROS_INFO("Extracting clusters...");
  try {
      ec.extract(cluster_indices);
      ROS_INFO("Found %zu clusters with initial parameters", cluster_indices.size());
  } catch (const std::exception& e) {
      ROS_ERROR("Clustering failed with error: %s", e.what());
      cluster_indices.clear();
  }
  
  // If no clusters found, adjust parameters and try again
  if (cluster_indices.empty()) {
      ROS_WARN("No clusters found with initial parameters, trying with larger tolerance...");
      ec.setClusterTolerance(0.05);  // Increase to 5cm
      ec.setMinClusterSize(300);     // Lower minimum size further for downsampled cloud
      
      ROS_INFO("Retry clustering with: tolerance=0.05m, min_points=300");
      try {
          ec.extract(cluster_indices);
          ROS_INFO("Second attempt found %zu clusters", cluster_indices.size());
      } catch (const std::exception& e) {
          ROS_ERROR("Second clustering attempt failed with error: %s", e.what());
      }
      
      // If still no clusters, try with even more relaxed parameters
      if (cluster_indices.empty()) {
          ROS_WARN("Still no clusters found, trying with minimal parameters...");
          ec.setClusterTolerance(0.1);   // Increase to 10cm
          ec.setMinClusterSize(100);     // Lower minimum size further
          
          ROS_INFO("Final clustering attempt with: tolerance=0.1m, min_points=100");
          try {
              ec.extract(cluster_indices);
              ROS_INFO("Final attempt found %zu clusters", cluster_indices.size());
          } catch (const std::exception& e) {
              ROS_ERROR("Final clustering attempt failed with error: %s", e.what());
          }
      }
  }
  
  // Process clusters
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
  bool basket_found = false;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr basket_cloud;
  pcl::PointXYZ basket_centroid;
  
  // For tracking black obstacles
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> black_obstacles;
  std::vector<pcl::PointXYZ> black_obstacle_centroids;
  
  for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        
      for (const auto& idx : indices.indices) {
          cluster->points.push_back(downsampled_cloud->points[idx]);
        }
        
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        
      // Check for brown color (basket detection by color)
      int brown_points = 0;
      for (const auto& point : cluster->points) {
          HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
          if (ColorUtils::is_brown(hsv)) {
              brown_points++;
          }
      }
      
      // Calculate the percentage of brown points in this cluster
      float brown_percentage = (float)brown_points / cluster->points.size();
      
      // If this cluster has significant brown points (>30%) and is reasonably large,
      // it might be the basket (brown detection takes precedence over size)
      if (brown_percentage > 0.3 && cluster->points.size() > 5000) {
          basket_found = true;
          basket_cloud = cluster;
          
          // Calculate basket centroid with outlier filtering for more accurate position
          pcl::PointXYZ raw_centroid;
          pcl::computeCentroid(*cluster, raw_centroid);
          
          // Extract only brown points for more accurate centroid calculation
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr brown_only_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
          for (const auto& point : cluster->points) {
              HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
              if (ColorUtils::is_brown(hsv)) {
                  brown_only_cloud->points.push_back(point);
              }
          }
          
          brown_only_cloud->width = brown_only_cloud->points.size();
          brown_only_cloud->height = 1;
          brown_only_cloud->is_dense = true;
          
          // Calculate centroid from brown-only points
          pcl::PointXYZ brown_centroid;
          pcl::computeCentroid(*brown_only_cloud, brown_centroid);
          
          // First, calculate mean and standard deviation of distances from brown centroid
          std::vector<float> distances;
          distances.reserve(brown_only_cloud->points.size());
          
          float mean_distance = 0.0f;
          for (const auto& point : brown_only_cloud->points) {
              float dx = point.x - brown_centroid.x;
              float dy = point.y - brown_centroid.y;
              float dz = point.z - brown_centroid.z;
              float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
              distances.push_back(distance);
              mean_distance += distance;
          }
          mean_distance /= distances.size();
          
          // Calculate standard deviation
          float variance = 0.0f;
          for (const auto& distance : distances) {
              float diff = distance - mean_distance;
              variance += diff * diff;
          }
          variance /= distances.size();
          float std_dev = std::sqrt(variance);
          
          // Threshold for outliers (points more than 2 standard deviations from mean)
          float threshold = mean_distance + 2.0f * std_dev;
          
          // Recalculate centroid using only inlier points
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_basket_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
          
          for (size_t i = 0; i < brown_only_cloud->points.size(); i++) {
              if (distances[i] <= threshold) {
                  filtered_basket_cloud->points.push_back(brown_only_cloud->points[i]);
              }
          }
          
          filtered_basket_cloud->width = filtered_basket_cloud->points.size();
          filtered_basket_cloud->height = 1;
          filtered_basket_cloud->is_dense = true;
          
          // Calculate final centroid with filtered points
          pcl::computeCentroid(*filtered_basket_cloud, basket_centroid);
          
          // Don't override z-coordinate for basket, use actual z from point cloud
          // basket_centroid.z = 0.04;
          
          ROS_INFO("BASKET detected by color! Cluster with %zu points, %d brown points (%.1f%%) at position (%.3f, %.3f, %.3f)", 
                  cluster->points.size(), brown_points, brown_percentage * 100.0f,
                  basket_centroid.x, basket_centroid.y, basket_centroid.z);
          
          // Log filtering statistics
          ROS_INFO("Basket detection (by brown color): Raw centroid (%.3f, %.3f, %.3f) -> Brown centroid (%.3f, %.3f, %.3f) -> Filtered centroid (%.3f, %.3f, %.3f)",
                  raw_centroid.x, raw_centroid.y, raw_centroid.z,
                  brown_centroid.x, brown_centroid.y, brown_centroid.z,
                  basket_centroid.x, basket_centroid.y, basket_centroid.z);
          
          // Skip adding to clusters for shape analysis
          continue;
      }

      // Check if this is a black obstacle (RGB values around [0.1, 0.1, 0.1])
      int black_points = 0;
      for (const auto& point : cluster->points) {
          // Convert RGB to HSV for better color detection
          HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
          
          // Check for black color (low brightness/value)
          // Black is characterized by very low value (brightness) in HSV
          if (hsv.v < 0.15) {  // Very dark points
              black_points++;
          }
      }
      
      // If more than 60% of points are black, classify as a black obstacle
      if (black_points > 0.6 * cluster->points.size()) {
          // Calculate obstacle centroid with outlier filtering for more accurate position
          pcl::PointXYZ raw_centroid;
          pcl::computeCentroid(*cluster, raw_centroid);
          
          // First, calculate mean and standard deviation of distances from raw centroid
          std::vector<float> distances;
          distances.reserve(cluster->points.size());
          
          float mean_distance = 0.0f;
          for (const auto& point : cluster->points) {
              float dx = point.x - raw_centroid.x;
              float dy = point.y - raw_centroid.y;
              float dz = point.z - raw_centroid.z;
              float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
              distances.push_back(distance);
              mean_distance += distance;
          }
          mean_distance /= distances.size();
          
          // Calculate standard deviation
          float variance = 0.0f;
          for (const auto& distance : distances) {
              float diff = distance - mean_distance;
              variance += diff * diff;
          }
          variance /= distances.size();
          float std_dev = std::sqrt(variance);
          
          // Threshold for outliers (points more than 2 standard deviations from mean)
          float threshold = mean_distance + 2.0f * std_dev;
          
          // Recalculate centroid using only inlier points
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_obstacle_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
          
          for (size_t i = 0; i < cluster->points.size(); i++) {
              if (distances[i] <= threshold) {
                  filtered_obstacle_cloud->points.push_back(cluster->points[i]);
              }
          }
          
          filtered_obstacle_cloud->width = filtered_obstacle_cloud->points.size();
          filtered_obstacle_cloud->height = 1;
          filtered_obstacle_cloud->is_dense = true;
          
          // Calculate final centroid with filtered points
          pcl::PointXYZ obstacle_centroid;
          pcl::computeCentroid(*filtered_obstacle_cloud, obstacle_centroid);
          
          // Don't override z-coordinate for obstacles, use actual z from point cloud
          // obstacle_centroid.z = 0.04;
          
          black_obstacles.push_back(filtered_obstacle_cloud);
          black_obstacle_centroids.push_back(obstacle_centroid);
          
          ROS_INFO("BLACK OBSTACLE detected! Cluster with %zu points (filtered to %zu) at position (%.3f, %.3f, %.3f)",
                  cluster->points.size(), filtered_obstacle_cloud->points.size(),
                  obstacle_centroid.x, obstacle_centroid.y, obstacle_centroid.z);
          
          // Log filtering statistics
          ROS_INFO("Black obstacle centroid filtering: raw (%.3f, %.3f, %.3f) -> filtered (%.3f, %.3f, %.3f)",
                  raw_centroid.x, raw_centroid.y, raw_centroid.z,
                  obstacle_centroid.x, obstacle_centroid.y, obstacle_centroid.z);
          
          // Skip adding to clusters for shape analysis
          continue;
      }
      
      clusters.push_back(cluster);
            
      ROS_INFO("Created cluster %zu with %zu points", 
              clusters.size()-1, cluster->size());
  }
  
  // Process each cluster for shape detection
  std::vector<DetectedObject> detected_objects;
  
  for (size_t i = 0; i < clusters.size(); i++) {
      if (clusters[i]->empty()) {
          ROS_WARN("Cluster %zu is empty, skipping", i);
          continue;
      }
            // Skip clusters with less than 1000 points
      if (clusters[i]->points.size() < 1000) {
          ROS_WARN("Cluster %zu is too small (%zu points < 1000), not a valid shape, skipping", 
                  i, clusters[i]->points.size());
          continue;
      }
      
      // Calculate centroid with outlier filtering for more accurate position
      pcl::PointXYZ raw_centroid;
      pcl::computeCentroid(*(clusters[i]), raw_centroid);
      
      // First, calculate mean and standard deviation of distances from raw centroid
      std::vector<float> distances;
      distances.reserve(clusters[i]->points.size());
      
      float mean_distance = 0.0f;
      for (const auto& point : clusters[i]->points) {
          float dx = point.x - raw_centroid.x;
          float dy = point.y - raw_centroid.y;
          float dz = point.z - raw_centroid.z;
          float distance = std::sqrt(dx*dx + dy*dy + dz*dz);
          distances.push_back(distance);
          mean_distance += distance;
      }
      mean_distance /= distances.size();
      
      // Calculate standard deviation
      float variance = 0.0f;
      for (const auto& distance : distances) {
          float diff = distance - mean_distance;
          variance += diff * diff;
      }
      variance /= distances.size();
      float std_dev = std::sqrt(variance);
      
      // Threshold for outliers (points more than 2 standard deviations from mean)
      float threshold = mean_distance + 2.0f * std_dev;
      
      // Recalculate centroid using only inlier points
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
      
      for (size_t j = 0; j < clusters[i]->points.size(); j++) {
          if (distances[j] <= threshold) {
              filtered_cluster->points.push_back(clusters[i]->points[j]);
          }
      }
      
      filtered_cluster->width = filtered_cluster->points.size();
      filtered_cluster->height = 1;
      filtered_cluster->is_dense = true;
      
      // Calculate final centroid with filtered points
      pcl::PointXYZ centroid;
      pcl::computeCentroid(*filtered_cluster, centroid);
      
      // Save original z value for logging
      float original_z = centroid.z;
      
      // Set z-coordinate to 0.04 as specified for shapes only
      centroid.z = 0.04;
      
      ROS_INFO("Cluster %zu centroid: raw (%.3f, %.3f, %.3f) -> filtered (%.3f, %.3f, %.3f) -> fixed z (%.3f, %.3f, 0.04)", 
              i, raw_centroid.x, raw_centroid.y, raw_centroid.z,
              centroid.x, centroid.y, original_z,
              centroid.x, centroid.y);
      
      // Count colors in this cluster (use filtered cluster for better accuracy)
      int red_count = 0, blue_count = 0, purple_count = 0;
      int red_center = 0, blue_center = 0, purple_center = 0;
      
      // Count points by color
      for (const auto& point : filtered_cluster->points) {
          // Convert RGB to HSV
          HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
          
          // Calculate distance from centroid for center density
          float dx = point.x - centroid.x;
          float dy = point.y - centroid.y;
          float distance_from_center = std::sqrt(dx*dx + dy*dy);
          bool is_center = (distance_from_center < 0.02); // Increased from 2cm to 3cm for larger shapes
          
          // Group points by color
          if (ColorUtils::matches_color(hsv, "red")) {
              red_count++;
              if (is_center) red_center++;
          } else if (ColorUtils::matches_color(hsv, "blue")) {
              blue_count++;
              if (is_center) blue_center++;
          } else if (ColorUtils::matches_color(hsv, "purple")) {
              purple_count++;
              if (is_center) purple_center++;
          }
      }
      
      // Find dominant color
      int max_color = std::max({red_count, blue_count, purple_count});
      std::string dominant_color;
      float center_density = 0.0;
      
    // Require a minimum number of color points
      if (max_color < 1000) {
          ROS_WARN("Cluster %zu has too few color points (max: %d < 1000), skipping", 
                 i, max_color);
          continue;
      }
      
      // Use a constant denominator for small clusters or very large clusters
      bool use_constant_denominator = (clusters[i]->points.size() < 2100 || clusters[i]->points.size() > 6000);
      int denominator;
      
      if (clusters[i]->points.size() < 2100) {
          denominator = 3500;
      } else if (clusters[i]->points.size() > 6000) {
          denominator = 6000;
      } else {
          denominator = max_color;
      }
      
      if (max_color == red_count && red_count > 0) {
          dominant_color = "red";
          center_density = (float)red_center / denominator;
      } else if (max_color == blue_count && blue_count > 0) {
          dominant_color = "blue";
          center_density = (float)blue_center / denominator;
      } else if (max_color == purple_count && purple_count > 0) {
          dominant_color = "purple";
          center_density = (float)purple_center / denominator;
      } else {
          ROS_WARN("Cluster %zu has no clear dominant color, skipping", i);
          continue;
      }
      
      ROS_INFO("Cluster %zu color counts - Red: %d, Blue: %d, Purple: %d", 
              i, red_count, blue_count, purple_count);
      
      // Determine denominator type for log message
      std::string denominator_type;
      if (clusters[i]->points.size() < 2100) {
          denominator_type = "small cluster";
      } else if (clusters[i]->points.size() > 6000) {
          denominator_type = "large cluster";
      } else {
          denominator_type = "actual";
      }
      
      ROS_INFO("Cluster %zu center density: %.3f (using %s denominator: %d, point count: %zu)", 
              i, center_density, denominator_type.c_str(), denominator, clusters[i]->points.size());
      
      // Determine shape based on center density - CORRECT LOGIC
      std::string shape;
      if (center_density < 0.045) {
          shape = "nought"; // Lower center density -> nought (hollow center/ring shape)
      } else {
          shape = "cross";  // Higher center density -> cross (filled center)
      }
      
      // Create detected object
      DetectedObject obj;
      obj.cloud = clusters[i];
      obj.color = dominant_color;
      obj.shape = shape;
      obj.centroid.x = centroid.x;
      obj.centroid.y = centroid.y;
      obj.centroid.z = centroid.z;
      
      ROS_INFO("Cluster %zu classified as: %s %s", i, obj.color.c_str(), obj.shape.c_str());
      detected_objects.push_back(obj);
  }
  
  // Count shapes by type and color
  int red_noughts = 0, red_crosses = 0;
  int blue_noughts = 0, blue_crosses = 0;
  int purple_noughts = 0, purple_crosses = 0;
  
  for (const auto& obj : detected_objects) {
      if (obj.shape == "nought") {
          if (obj.color == "red") red_noughts++;
          else if (obj.color == "blue") blue_noughts++;
          else if (obj.color == "purple") purple_noughts++;
      } else if (obj.shape == "cross") {
          if (obj.color == "red") red_crosses++;
          else if (obj.color == "blue") blue_crosses++;
          else if (obj.color == "purple") purple_crosses++;
      }
  }
  
  ROS_INFO("Shape estimates from clusters:");
  ROS_INFO("  Red: %d noughts, %d crosses", red_noughts, red_crosses);
  ROS_INFO("  Blue: %d noughts, %d crosses", blue_noughts, blue_crosses);
  ROS_INFO("  Purple: %d noughts, %d crosses", purple_noughts, purple_crosses);
  
  // Count total shapes
  int nought_count = red_noughts + blue_noughts + purple_noughts;
  int cross_count = red_crosses + blue_crosses + purple_crosses;
  int total_shapes = nought_count + cross_count;
  
  ROS_INFO("Total shapes from clusters: %d (noughts: %d, crosses: %d)", 
          total_shapes, nought_count, cross_count);
  
  // Determine most common shape
  std::string most_common_shape;
  int num_most_common = 0;
  bool shapes_equal = false;
  
  if (cross_count > nought_count) {
      most_common_shape = "cross";
      num_most_common = cross_count;
      ROS_INFO("Cross is the most common shape (%d crosses vs %d noughts)", cross_count, nought_count);
  } else if (nought_count > cross_count) {
      most_common_shape = "nought";
      num_most_common = nought_count;
      ROS_INFO("Nought is the most common shape (%d noughts vs %d crosses)", nought_count, cross_count);
        } else {
      // Equal counts - indicate a tie
      shapes_equal = true;
      most_common_shape = "equal";
      num_most_common = cross_count; // Both are the same
      ROS_INFO("TIE! Equal number of crosses and noughts (%d each)", cross_count);
  }
  
  ROS_INFO("Most common shape from clusters: %s (count: %d)", 
          shapes_equal ? "EQUAL (tie)" : most_common_shape.c_str(), num_most_common);
  
  // Report about basket
  if (basket_found) {
      ROS_INFO("==========================================================");
      ROS_INFO("BASKET INFORMATION:");
      
      // Check if there are any brown points in the basket
      int basket_brown_points = 0;
      if (basket_cloud) {
          for (const auto& point : basket_cloud->points) {
              HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
              if (ColorUtils::is_brown(hsv)) {
                  basket_brown_points++;
              }
          }
          
          float brown_percentage = (float)basket_brown_points / basket_cloud->points.size() * 100.0f;
          
          ROS_INFO("  Position: (%.3f, %.3f, %.3f)", 
                  basket_centroid.x, basket_centroid.y, basket_centroid.z);
          ROS_INFO("  Size: %zu points", basket_cloud->points.size());
          ROS_INFO("  Brown points: %d (%.1f%%)", basket_brown_points, brown_percentage);
          ROS_INFO("  Detection method: Brown color detection");
          ROS_INFO("  Status: Basket was excluded from shape analysis");
      } else {
          ROS_INFO("  Position: (%.3f, %.3f, %.3f)", 
                  basket_centroid.x, basket_centroid.y, basket_centroid.z);
          ROS_INFO("  Status: Basket was excluded from shape analysis");
      }
      ROS_INFO("==========================================================");
  } else {
      ROS_INFO("No basket was detected in the scene (no clusters with >30%% brown color)");
  }
  
  // Report about black obstacles
  if (!black_obstacles.empty()) {
      ROS_INFO("==========================================================");
      ROS_INFO("BLACK OBSTACLES INFORMATION:");
      ROS_INFO("  Number of obstacles detected: %zu", black_obstacles.size());
      
      for (size_t i = 0; i < black_obstacles.size(); i++) {
          ROS_INFO("  Obstacle %zu:", i);
          ROS_INFO("    Position: (%.3f, %.3f, %.3f)", 
                  black_obstacle_centroids[i].x, 
                  black_obstacle_centroids[i].y, 
                  black_obstacle_centroids[i].z);
          ROS_INFO("    Size: %zu points", black_obstacles[i]->points.size());
          
          // Update the maximum obstacle height
          max_obstacle_height_ = std::max(max_obstacle_height_, 
                                         static_cast<double>(black_obstacle_centroids[i].z));
      }
      
      ROS_INFO("Maximum obstacle height detected: %.3f m", max_obstacle_height_);
      
      ROS_INFO("  Status: Black obstacles were excluded from shape analysis");
      ROS_INFO("==========================================================");
  } else {
      ROS_INFO("No black obstacles were detected in the scene");
  }
  
  // Fill in service response
  response.total_num_shapes = total_shapes;
  response.num_most_common_shape = num_most_common;
    
    // =====================================================================
    // Size detection and Task 1 integration
    // =====================================================================
    
    ROS_INFO("Performing size detection and integration with Task 1...");
    
    // Check if we found any valid clusters
    if (detected_objects.empty()) {
        ROS_WARN("No valid clusters found for size detection");
        return true;
    }
    
    // Find the best cluster based on the most common shape and color count
    size_t best_cluster_idx = 0;
    int max_color_count = 0;
    bool found_matching_shape = false;
    
    // First pass: Try to find a cluster matching the most common shape
    if (!shapes_equal) {
        ROS_INFO("Looking for clusters matching the most common shape: %s", most_common_shape.c_str());
        
        for (size_t i = 0; i < detected_objects.size(); i++) {
            if (detected_objects[i].shape == most_common_shape) {
                // Count color points in this matching shape cluster
                int red_count = 0, blue_count = 0, purple_count = 0;
                
                for (const auto& point : detected_objects[i].cloud->points) {
                    HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
                    
                    if (ColorUtils::matches_color(hsv, "red")) {
                        red_count++;
                    } else if (ColorUtils::matches_color(hsv, "blue")) {
                        blue_count++;
                    } else if (ColorUtils::matches_color(hsv, "purple")) {
                        purple_count++;
                    }
                }
                
                int total_color_count = red_count + blue_count + purple_count;
                
                ROS_INFO("Cluster %zu: %s %s with %d color points (Red: %d, Blue: %d, Purple: %d)",
                        i, detected_objects[i].color.c_str(), detected_objects[i].shape.c_str(), 
                        total_color_count, red_count, blue_count, purple_count);
                
                if (total_color_count > max_color_count) {
                    max_color_count = total_color_count;
                    best_cluster_idx = i;
                    found_matching_shape = true;
                }
            }
        }
    }
    
    // Second pass: If no matching shape found or shapes are equal, find best overall cluster
    if (!found_matching_shape) {
        ROS_INFO("No clusters matching most common shape found or shapes are equal. Finding best overall cluster.");
        max_color_count = 0; // Reset for second pass
        
        for (size_t i = 0; i < detected_objects.size(); i++) {
            // Count color points in this cluster
            int red_count = 0, blue_count = 0, purple_count = 0;
            
            for (const auto& point : detected_objects[i].cloud->points) {
                HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
                
                if (ColorUtils::matches_color(hsv, "red")) {
                    red_count++;
                } else if (ColorUtils::matches_color(hsv, "blue")) {
                    blue_count++;
                } else if (ColorUtils::matches_color(hsv, "purple")) {
                    purple_count++;
                }
            }
            
            int total_color_count = red_count + blue_count + purple_count;
            
            ROS_INFO("Cluster %zu: %s %s with %d color points (Red: %d, Blue: %d, Purple: %d)",
                    i, detected_objects[i].color.c_str(), detected_objects[i].shape.c_str(), 
                    total_color_count, red_count, blue_count, purple_count);
            
            if (total_color_count > max_color_count) {
                max_color_count = total_color_count;
                best_cluster_idx = i;
            }
        }
    }
    
    // Get the best cluster and determine size
    const DetectedObject& best_object = detected_objects[best_cluster_idx];
    
    // Reuse the calculated color count for the best cluster
    int red_count = 0, blue_count = 0, purple_count = 0;
    for (const auto& point : best_object.cloud->points) {
        HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
        
        if (ColorUtils::matches_color(hsv, "red")) {
            red_count++;
        } else if (ColorUtils::matches_color(hsv, "blue")) {
            blue_count++;
        } else if (ColorUtils::matches_color(hsv, "purple")) {
            purple_count++;
        }
    }
    
    int total_color_count = red_count + blue_count + purple_count;
    
    ROS_INFO("Selected best cluster (idx: %zu): %s %s with %d color points (size: %zu points)",
            best_cluster_idx, best_object.color.c_str(), best_object.shape.c_str(), 
            total_color_count, best_object.cloud->points.size());
    
    // Determine the size based on shape-specific thresholds
    int detected_size_mm = 40; // Default to 40mm
    
    if (best_object.shape == "cross") {
        if (total_color_count > 6200) {
            detected_size_mm = 40;
        } else if (total_color_count > 4000) {
            detected_size_mm = 30;
        } else {
            detected_size_mm = 20;
        }
    } else { // nought
        if (total_color_count > 9200) {
            detected_size_mm = 40;
        } else if (total_color_count > 5000) {
            detected_size_mm = 30;
        } else {
            detected_size_mm = 20;
        }
    }

    ROS_INFO("Detected size: %d mm (based on %s shape criteria)",
            detected_size_mm, best_object.shape.c_str());
    
    // Update the last detected size
    last_detected_size_mm_ = detected_size_mm;
    
    // Execute pick and place if requested and if basket was found
    if (basket_found) {
        ROS_INFO("Starting Task 1 integration: Pick and place operation");
        
        // Initialize MoveIt for pick and place
        MoveGroupInterface move_group = initialize_move_group("panda_arm");
        std::vector<double> home_joint_values = store_home_position(move_group);
        
        // Create object pose directly
        geometry_msgs::Pose object_pose;
        object_pose.position.x = best_object.centroid.x;
        object_pose.position.y = best_object.centroid.y;
        object_pose.position.z = 0.0;
        
        // Set orientation - default 315 degrees yaw
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, M_PI * 315.0/180.0);  // Convert to radians
        object_pose.orientation = tf2::toMsg(q);
        
        // Create basket pose directly
        geometry_msgs::Pose basket_pose;
        basket_pose.position.x = basket_centroid.x;
        basket_pose.position.y = basket_centroid.y;
        basket_pose.position.z = 0.024;
        basket_pose.orientation.w = 1.0;  // Identity orientation
        
        // Now set up parameters similar to Task 1
        PickPlaceParams params;
        params.object_pose = object_pose;
        params.place_pose = basket_pose;
        params.shape_type = best_object.shape;
        
        // Adapt parameters based on number of obstacles
        // With more obstacles, we need more conservative parameters
        if (black_obstacles.size() >= 4) {
            ROS_INFO("Detected %zu obstacles - using more conservative pick and place parameters", black_obstacles.size());
            params.approach_offset = 0.21;         // 22cm above object (increased)
            params.pickup_offset = 0.16;           // 16cm for pickup (increased)
            params.place_approach_offset = 0.35;   // 30cm above basket (increased)
        } else {
            // Use standard offsets for fewer obstacles
            params.approach_offset = 0.20;         // 20cm above object
            params.pickup_offset = 0.14;           // 14cm for pickup
            params.place_approach_offset = 0.28;   // 28cm above basket
        }
        
        ROS_INFO("Using pick and place parameters: approach=%.2fm, pickup=%.2fm, place_approach=%.2fm",
                 params.approach_offset, params.pickup_offset, params.place_approach_offset);
        
        // Execute pick & place
        ROS_INFO("Executing pick and place for %s %s at (%.3f, %.3f, %.3f)",
                 best_object.color.c_str(), best_object.shape.c_str(),
                 params.object_pose.position.x, params.object_pose.position.y, params.object_pose.position.z);
        
        bool success = execute_pick_and_place(move_group, params);
        
        if (success) {
            ROS_INFO("Successfully picked up %s %s and placed it in basket",
                    best_object.color.c_str(), best_object.shape.c_str());
        } else {
            ROS_ERROR("Failed to pick/place %s %s",
                     best_object.color.c_str(), best_object.shape.c_str());
        }
        
        // First move upward to a safe height before returning home
        geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
        geometry_msgs::Pose safe_pose = current_pose;
        safe_pose.position.z = 0.4;  // Move to 40cm above the table, which is a safe clearing height
        
        ROS_INFO("Moving to safe height (z=%.2f) before returning home...", safe_pose.position.z);
        move_group.setPoseTarget(safe_pose);
        
        if (move_group.move() == MoveItErrorCode::SUCCESS) {
            ROS_INFO("Successfully moved to safe height");
        } else {
            ROS_WARN("Failed to move to safe height, attempting to return home directly");
        }
        
        // Return to home position
        return_to_home_position(move_group, home_joint_values);
    } else {
        ROS_WARN("No basket detected, skipping pick and place operation");
    }
    
    return true;
}
