/* Revised cw1_class.cpp for Task 1: Pick and Place using MoveIt!
   Code has been refactored to improve style, structure, efficiency and clarity
   while maintaining all existing logic.
*/

#include <cw1_class.h>
#include <pcl/common/common.h>

// Namespace for robot-related constants
namespace RobotConstants {
    const double GRIPPER_OPEN = 0.08;     // Fully open gripper width in meters
    const double GRIPPER_CLOSED = 0.02;   // Fully closed gripper width in meters
    const double TIGHT_GRIP = 0.018;      // Tighter grip for secure holding (90% of closed)
    
    // Approach and pickup distances
    const double APPROACH_OFFSET = 0.15;  // 15cm above object for approach
    const double PICKUP_OFFSET = 0.12;    // 12cm above object for pickup
    const double BASKET_APPROACH = 0.30;  // 30cm above basket for approach
    
    // Movement and planning parameters
    const double PLANNING_TIME = 10.0;
    const int PLANNING_ATTEMPTS = 10;
    const double MAX_VELOCITY = 0.30;
    const double MAX_ACCELERATION = 0.30;
    const int MAX_EXECUTION_ATTEMPTS = 3;
}

// Namespace for point cloud processing constants
namespace PCLConstants {
    const float VOXEL_LEAF_SIZE = 0.001f;  // Voxel grid leaf size (1mm)
    const double Z_THRESHOLD = 0.02;       // Height threshold for table filtering
    const double CLUSTER_TOLERANCE = 0.01; // 1cm cluster tolerance
    const int MIN_CLUSTER_SIZE = 20;       // Minimum points per cluster
    const int MAX_CLUSTER_SIZE = 10000;    // Maximum points per cluster
    const float MIN_SATURATION = 0.1f;     // Minimum color saturation
    const float MIN_BRIGHTNESS = 0.1f;     // Minimum color brightness
    
    // Color detection thresholds
    const int MIN_COLOR_POINTS = 20;       // Minimum points to establish a color
    const int MIN_VOTES_NEEDED = 20;       // Minimum votes to determine dominant color
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
            return ((hsv.h >= 330.0f || hsv.h <= 30.0f) && hsv.s >= 0.3f);
        }
        // Blue: Hue around 240 (180-260) with good saturation
        else if (color_name == "blue") {
            return (hsv.h >= 180.0f && hsv.h <= 260.0f && hsv.s >= 0.3f);
        }
        // Purple: Hue around 300 (260-330) with decent saturation
        else if (color_name == "purple") {
            return (hsv.h >= 260.0f && hsv.h <= 330.0f && hsv.s >= 0.2f);
        }
        
        return false;
    }
}

// -----------------------------------------------------------------------------
// Class Implementation
// -----------------------------------------------------------------------------

/**
 * Class constructor for cw1 class
 */
cw1::cw1(ros::NodeHandle nh) : nh_(nh)
{
    // Advertise services for coursework tasks
    t1_service_ = nh_.advertiseService("/task1_start", &cw1::t1_callback, this);
    t2_service_ = nh_.advertiseService("/task2_start", &cw1::t2_callback, this);
    t3_service_ = nh_.advertiseService("/task3_start", &cw1::t3_callback, this);

    ROS_INFO("cw1 class initialized");
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);

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
bool cw1::move_gripper(double target_width)
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
bool cw1::execute_pose_target(
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
MoveGroupInterface cw1::initialize_move_group(const std::string& group_name) {
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
std::vector<double> cw1::store_home_position(MoveGroupInterface& move_group) {
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
bool cw1::return_to_home_position(
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

/**
 * Execute a complete pick and place operation
 * 
 * @param move_group MoveGroupInterface to control
 * @param params Pick and place parameters
 * @return True if the operation was successful
 */
bool cw1::execute_pick_and_place(
    MoveGroupInterface& move_group,
    const PickPlaceParams& params)
{
    // 1. Set fixed orientation using tf2 quaternion
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, M_PI / 4.0);
    geometry_msgs::Quaternion fixed_orientation = tf2::toMsg(q);
    
    // 2. Open the gripper before approaching the object
    ROS_INFO("Opening gripper...");
    if (!move_gripper(RobotConstants::GRIPPER_OPEN)) {
        ROS_ERROR("Failed to open gripper");
        return false;
    }
    
    // 3. Create the approach pose with fixed orientation
    geometry_msgs::Pose approach_pose = params.object_pose;
    approach_pose.position.z += params.approach_offset;
    approach_pose.orientation = fixed_orientation;
    
    // 4. Move to the approach pose
    ROS_INFO("Moving to approach pose (%.3f, %.3f, %.3f)...",
            approach_pose.position.x, approach_pose.position.y, approach_pose.position.z);
    
    if (!execute_pose_target(move_group, approach_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to move to approach pose");
        return false;
    }
    
    // 5. Create and move to the pickup pose
    geometry_msgs::Pose pickup_pose = params.object_pose;
    pickup_pose.position.z += params.pickup_offset;
    pickup_pose.orientation = fixed_orientation;
    
    ROS_INFO("Moving to pickup pose (%.3f, %.3f, %.3f)...",
            pickup_pose.position.x, pickup_pose.position.y, pickup_pose.position.z);
    
    if (!execute_pose_target(move_group, pickup_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to move to pickup pose");
        return false;
    }
    
    // 6. Close the gripper to grasp the object
    ROS_INFO("Closing gripper to grasp object...");
    if (!move_gripper(RobotConstants::TIGHT_GRIP)) {
        ROS_ERROR("Failed to close gripper");
        return false;
    }
    
    // Allow time for grip to solidify
    ros::Duration(0.5).sleep();
    
    // 7. Lift the object by moving back to the approach pose
    ROS_INFO("Lifting the object...");
    if (!execute_pose_target(move_group, approach_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to lift the object");
        return false;
    }
    
    // 8. Create the place approach pose with fixed orientation
    geometry_msgs::Pose place_approach_pose = params.place_pose;
    place_approach_pose.position.z += params.place_approach_offset;
    place_approach_pose.orientation = fixed_orientation;
    
    // 9. Move to the place approach pose
    ROS_INFO("Moving to place approach pose (%.3f, %.3f, %.3f)...",
            place_approach_pose.position.x, place_approach_pose.position.y, 
            place_approach_pose.position.z);
    
    if (!execute_pose_target(move_group, place_approach_pose, RobotConstants::MAX_EXECUTION_ATTEMPTS)) {
        ROS_ERROR("Failed to move to place approach pose");
        return false;
    }
    
    // 10. Release the object
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
void cw1::apply_voxel_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
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
void cw1::remove_green_background(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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

/**
 * Extract clusters from a point cloud
 * 
 * @param cloud Input point cloud
 * @return Vector of point cloud clusters
 */
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> 
cw1::extract_clusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    
    // Create KD-tree for efficient nearest neighbor search
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud);

    // Extract clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(PCLConstants::CLUSTER_TOLERANCE);
    ec.setMinClusterSize(PCLConstants::MIN_CLUSTER_SIZE);
    ec.setMaxClusterSize(PCLConstants::MAX_CLUSTER_SIZE);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    ROS_INFO("Found %zu clusters", cluster_indices.size());

    // Process each cluster
    for (const auto &indices : cluster_indices)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        
        for (const int idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }
        
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        
        // Calculate cluster dimensions
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D<pcl::PointXYZRGB>(*cluster, min_pt, max_pt);
        float width = max_pt.x - min_pt.x;
        float height = max_pt.y - min_pt.y;
        float depth = max_pt.z - min_pt.z;
        float volume = width * height * depth;
        
        ROS_INFO("Cluster with %zu points: %.3f x %.3f x %.3f m (volume: %.4f m³)", 
                 cluster->points.size(), width, height, depth, volume);
        
        clusters.push_back(cluster);
    }

    return clusters;
}

/**
 * Process a point cloud: downsample, remove background, and extract clusters
 * 
 * @param input_cloud Raw input point cloud
 * @param output_clusters Vector to store resulting clusters
 * @return True if processing was successful
 */
bool cw1::process_point_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& output_clusters)
{
    if (!input_cloud || input_cloud->empty()) {
        ROS_ERROR("Invalid or empty input point cloud");
        return false;
    }
    
    try {
        // 1. Downsample the cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>());
        apply_voxel_filter(input_cloud, cloud_downsampled);
        
        // 2. Remove green background
        remove_green_background(cloud_downsampled);
        
        // 3. Extract clusters
        output_clusters = extract_clusters(cloud_downsampled);
        
        return !output_clusters.empty();
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception during point cloud processing: %s", e.what());
        return false;
    }
}

/**
 * Detect the dominant color of a point cloud cluster
 * 
 * @param cluster Point cloud cluster to analyze
 * @param min_votes_needed Minimum number of points needed to establish a color
 * @return Dominant color name ("red", "blue", "purple", or "none")
 */
std::string cw1::detect_dominant_color(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster,
    int min_votes_needed)
{
    if (!cluster || cluster->empty()) {
        ROS_WARN("Empty cluster provided for color detection");
        return "none";
    }
    
    std::map<std::string, int> color_counts;
    int total_color_points = 0;
    
    // Examine each point in the cluster
    for (const auto& point : cluster->points) {
        // Skip points with zero RGB values (likely invalid)
        if (point.r == 0 && point.g == 0 && point.b == 0) {
            continue;
        }
        
        // Convert to HSV colorspace
        HSV hsv = ColorUtils::rgb2hsv(point.r, point.g, point.b);
        
        // Skip unsaturated colors
        if (hsv.s < PCLConstants::MIN_SATURATION) {
            continue;
        }
        
        // Check each reference color
        for (const auto& ref : COLOR_REFS) {
            if (ColorUtils::matches_color(hsv, ref.name)) {
                color_counts[ref.name]++;
                total_color_points++;
                break; // Take first match only
            }
        }
    }
    
    // Find the dominant color
    std::string dominant_color = "none";
    int max_count = min_votes_needed;
    
    for (const auto& count : color_counts) {
        if (count.second > max_count) {
            max_count = count.second;
            dominant_color = count.first;
        }
    }
    
    // Log color distribution data
    ROS_INFO("Color points detected: %d out of %zu points", 
             total_color_points, cluster->points.size());
    
    for (const auto& count : color_counts) {
        ROS_INFO("  Color %s: %d points (%.1f%%)", 
                 count.first.c_str(), count.second, 
                 100.0f * count.second / (total_color_points > 0 ? total_color_points : 1));
    }
    
    return dominant_color;
}

/**
 * Detect the color of a basket in the point cloud
 * 
 * @param cloud Point cloud containing the basket
 * @param basket_loc Location of the basket in 3D space
 * @param tf_buffer Transform buffer for coordinate transformations
 * @param cloud_frame Reference frame of the point cloud
 * @param search_radius Radius around the basket location to search for color points
 * @return Detected color ("red", "blue", "purple", or "none")
 */
std::string cw1::detect_basket_color(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const geometry_msgs::PointStamped& basket_loc,
    tf2_ros::Buffer& tf_buffer,
    const std::string& cloud_frame,
    double search_radius)
{
    // Validate point cloud
    if (!cloud || cloud->empty() || cloud->points.empty()) {
        ROS_ERROR("Empty or invalid point cloud data");
        return "none";
    }
    
    // Transform the basket location to the point cloud frame
    geometry_msgs::PointStamped transformed_basket;
    try {
        // Create a new copy of the basket location with current timestamp
        geometry_msgs::PointStamped current_basket = basket_loc;
        current_basket.header.stamp = ros::Time(0);
        
        // Transform to the cloud frame
        if (tf_buffer.canTransform(cloud_frame, current_basket.header.frame_id, 
                                  current_basket.header.stamp, ros::Duration(1.0))) {
            transformed_basket = tf_buffer.transform(current_basket, cloud_frame, ros::Duration(1.0));
            ROS_INFO("Transformed coordinates: (%f, %f, %f) → (%f, %f, %f)",
                   basket_loc.point.x, basket_loc.point.y, basket_loc.point.z,
                   transformed_basket.point.x, transformed_basket.point.y, transformed_basket.point.z);
        } else {
            ROS_ERROR("Cannot transform from '%s' to '%s'", 
                     current_basket.header.frame_id.c_str(), cloud_frame.c_str());
            return "none";
        }
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        return "none";
    }
    
    // Create KD-tree for fast nearest neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    try {
        kdtree.setInputCloud(cloud);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception creating KD-tree: %s", e.what());
        return "none";
    }
    
    // Set up the search point from transformed coordinates
    pcl::PointXYZRGB search_point;
    search_point.x = transformed_basket.point.x;
    search_point.y = transformed_basket.point.y;
    search_point.z = transformed_basket.point.z;
    
    // Search for points within radius
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    
    ROS_INFO("Checking basket at transformed point (%.3f, %.3f, %.3f) with radius %.3f", 
           search_point.x, search_point.y, search_point.z, search_radius);
    
    try {
        int points_found = kdtree.radiusSearch(search_point, search_radius, 
                                               point_indices, point_distances);
        
        if (points_found > 0) {
            ROS_INFO("Found %d points within %.3f meters of basket location",
                    points_found, search_radius);
            
            // Count votes for each color
            int votes_blue = 0;
            int votes_red = 0;
            int votes_purple = 0;
            int total_points = points_found;
            
            // Examine up to 50 nearest points (or all if less than 50)
            const int max_points_to_check = 50;
            int points_to_check = std::min(total_points, max_points_to_check);
            
            // Collect color samples for analysis
            std::vector<double> r_values, g_values, b_values;
            std::vector<double> h_values, s_values, v_values;
            
            for (int i = 0; i < points_to_check; i++) {
                // Check if the index is valid
                if (point_indices[i] >= static_cast<int>(cloud->points.size())) {
                    ROS_WARN("Invalid point index %d (cloud size: %zu)", 
                             point_indices[i], cloud->points.size());
                    continue;
                }
                
                pcl::PointXYZRGB pt = cloud->points[point_indices[i]];
                
                // Normalize color values to [0, 1]
                double r = static_cast<double>(pt.r) / 255.0;
                double g = static_cast<double>(pt.g) / 255.0;
                double b = static_cast<double>(pt.b) / 255.0;
                
                // Convert RGB to HSV
                double h, s, v;
                HSV hsv = ColorUtils::rgb2hsv(pt.r, pt.g, pt.b);
                h = hsv.h;
                s = hsv.s;
                v = hsv.v;
                
                // Skip green points (likely table/floor)
                if (ColorUtils::is_greenish(h, s, v)) {
                    continue;
                }
                
                // Only collect non-green colors
                r_values.push_back(r);
                g_values.push_back(g);
                b_values.push_back(b);
                h_values.push_back(h);
                s_values.push_back(s);
                v_values.push_back(v);
                
                // Print some sample colors for debugging
                if (i < 5) {
                    ROS_INFO("Sample color %d: RGB(%.2f, %.2f, %.2f) HSV(%.1f, %.2f, %.2f)",
                             i, r, g, b, h, s, v);
                }
                
                // HSV thresholds for basket colors - with higher saturation requirements
                if (s > 0.4 && v > 0.2) {  // Must be saturated and reasonably bright
                    // Blue: H≈240, high S, decent V
                    if (h >= 210 && h <= 270) {
                        votes_blue++;
                    }
                    // Red: H≈0 or H≈360, high S, decent V
                    else if ((h <= 30 || h >= 330) && s > 0.4) {
                        votes_red++;
                    }
                    // Purple: H≈300, high S, decent V
                    else if (h >= 270 && h <= 330) {
                        votes_purple++;
                    }
                }
            }
            
            // Check if we collected any non-green samples
            if (r_values.empty()) {
                ROS_WARN("No valid color samples collected (all points were green)");
                return "none";
            }
            
            // Calculate average RGB and HSV values for diagnostics
            double avg_r = std::accumulate(r_values.begin(), r_values.end(), 0.0) / r_values.size();
            double avg_g = std::accumulate(g_values.begin(), g_values.end(), 0.0) / g_values.size();
            double avg_b = std::accumulate(b_values.begin(), b_values.end(), 0.0) / b_values.size();
            
            double avg_h = std::accumulate(h_values.begin(), h_values.end(), 0.0) / h_values.size();
            double avg_s = std::accumulate(s_values.begin(), s_values.end(), 0.0) / s_values.size();
            double avg_v = std::accumulate(v_values.begin(), v_values.end(), 0.0) / v_values.size();
            
            ROS_INFO("Average RGB (non-green): (%.2f, %.2f, %.2f)", avg_r, avg_g, avg_b);
            ROS_INFO("Average HSV (non-green): (%.1f, %.2f, %.2f)", avg_h, avg_s, avg_v);
            ROS_INFO("Color votes - Blue: %d, Red: %d, Purple: %d", votes_blue, votes_red, votes_purple);
            
            // Determine the dominant color based on votes
            const int min_votes_needed = PCLConstants::MIN_VOTES_NEEDED;
            
            if (votes_blue > votes_red && votes_blue > votes_purple && votes_blue >= min_votes_needed) {
                ROS_INFO("Detected BLUE basket");
                return "blue";
            }
            else if (votes_red > votes_blue && votes_red > votes_purple && votes_red >= min_votes_needed) {
                ROS_INFO("Detected RED basket");
                return "red";
            }
            else if (votes_purple > votes_blue && votes_purple > votes_red && votes_purple >= min_votes_needed) {
                ROS_INFO("Detected PURPLE basket");
                return "purple";
            }
            else {
                ROS_INFO("No definitive basket color detected");
                return "none";
            }
        } else {
            ROS_INFO("No points found within search radius - marking as NONE");
            return "none";
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception during radius search: %s", e.what());
        return "none";
    }
}

/**
 * Get point cloud description for debugging
 */
void cw1::describe_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        ROS_INFO("Point cloud is empty");
        return;
    }
    
    // Basic stats
    ROS_INFO("Point cloud contains %zu points", cloud->size());
    ROS_INFO("Point cloud is %s", cloud->isOrganized() ? "organized" : "unorganized");
    
    // Find min/max values
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    
    ROS_INFO("X range: [%.3f, %.3f]", min_pt.x, max_pt.x);
    ROS_INFO("Y range: [%.3f, %.3f]", min_pt.y, max_pt.y);
    ROS_INFO("Z range: [%.3f, %.3f]", min_pt.z, max_pt.z);
    
    // Sample a few points
    ROS_INFO("Sample points (first 3):");
    for (size_t i = 0; i < std::min(size_t(3), cloud->size()); i++) {
        pcl::PointXYZRGB pt = cloud->points[i];
        ROS_INFO("  Point %zu: XYZ(%.3f, %.3f, %.3f) RGB(%d, %d, %d)", 
                i, pt.x, pt.y, pt.z, pt.r, pt.g, pt.b);
    }
}

// -----------------------------------------------------------------------------
// Task 1: Pick and Place
// -----------------------------------------------------------------------------

bool cw1::t1_callback(cw1_world_spawner::Task1Service::Request &request,
                     cw1_world_spawner::Task1Service::Response &response)
{
    ROS_INFO("Task 1: Starting pick and place operation.");

    // 1) Initialize MoveIt! interface
    MoveGroupInterface move_group = initialize_move_group("panda_arm");

    // 2) Create pick and place parameters
    PickPlaceParams params;
    
    // Set object pose from request
    params.object_pose = request.object_loc.pose;
    
    // Set place pose from request
    params.place_pose.position = request.goal_loc.point;
    
    // Set fixed orientation
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, M_PI / 4.0);
    params.object_pose.orientation = tf2::toMsg(q);
    params.place_pose.orientation = tf2::toMsg(q);
    
    // 3) Execute pick and place operation
    bool success = execute_pick_and_place(move_group, params);
    
    if (success) {
        ROS_INFO("Task 1: Pick and Place operation completed successfully.");
    } else {
        ROS_ERROR("Task 1: Pick and Place operation failed.");
    }
    
    return success;
}

// -----------------------------------------------------------------------------
// Task 2: Basket Color Detection
// -----------------------------------------------------------------------------

bool cw1::t2_callback(cw1_world_spawner::Task2Service::Request &request,
                     cw1_world_spawner::Task2Service::Response &response)
{
    ROS_INFO("Task 2: Starting basket color detection");

    // 1) Initialize MoveIt with more reliable settings
    MoveGroupInterface move_group = initialize_move_group("panda_arm");
    move_group.setMaxVelocityScalingFactor(0.25);    // Slower, more reliable movement
    move_group.setMaxAccelerationScalingFactor(0.25);

    // 2) Get basket locations
    std::vector<geometry_msgs::PointStamped> basket_locs = request.basket_locs;
    ROS_INFO("Received %zu potential basket locations to check", basket_locs.size());
    
    // Initialize results array
    std::vector<std::string> basket_colours(basket_locs.size(), "none");
    
    // Define the home position to return to between each basket
    geometry_msgs::Pose home_pose;
    home_pose.position.x = 0.4;   // Safe middle position
    home_pose.position.y = 0.0;   // Centered
    home_pose.position.z = 0.5;   // Safe height
    
    // Set fixed orientation
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, M_PI/4.0);
    home_pose.orientation = tf2::toMsg(q);
    
    // First move to home position
    ROS_INFO("Moving to home position");
    execute_pose_target(move_group, home_pose);
    ros::Duration(1.0).sleep();
    
    // Process each basket location with return to home between each
    for (size_t i = 0; i < basket_locs.size(); i++) {
        try {
            ROS_INFO("Processing basket location %zu", i);
            
            // Extract the position from the PointStamped
            geometry_msgs::Point basket_pos = basket_locs[i].point;
            ROS_INFO("Basket position: (%.3f, %.3f, %.3f)", basket_pos.x, basket_pos.y, basket_pos.z);
            
            // Calculate a viewing position directly above the basket
            geometry_msgs::Pose view_pose;
            
            // Position directly above the basket
            view_pose.position.x = basket_pos.x;
            view_pose.position.y = basket_pos.y;
            view_pose.position.z = basket_pos.z + 0.40;  // 42cm above the basket
            view_pose.orientation = tf2::toMsg(q);
            
            // Move to basket viewing position
            ROS_INFO("Moving to viewing position (%.3f, %.3f, %.3f) for basket %zu",
                    view_pose.position.x, view_pose.position.y, view_pose.position.z, i);
            
            if (execute_pose_target(move_group, view_pose)) {
                ROS_INFO("Successfully moved to viewing position for basket %zu", i);
                
                // Hold position for stable data
                ros::Duration(2.0).sleep();
                
                // Get point cloud data
                sensor_msgs::PointCloud2ConstPtr cloud_msg = 
                    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                        "/r200/camera/depth_registered/points", nh_, ros::Duration(10.0));
                        
                if (cloud_msg) {
                    std::string cloud_frame = cloud_msg->header.frame_id;
                    
                    // Process point cloud
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                    pcl::fromROSMsg(*cloud_msg, *cloud);
                    
                    // Detect basket color
                    std::string color = detect_basket_color(cloud, basket_locs[i], *tf_buffer_, cloud_frame);
                    if (color != "none") {
                        basket_colours[i] = color;
                        ROS_INFO("Basket %zu color: %s", i, color.c_str());
                    }
                }
                
                // Return to home position after inspecting this basket
                ROS_INFO("Returning to home position after basket %zu", i);
                execute_pose_target(move_group, home_pose);
                ros::Duration(1.0).sleep();
            }
            else {
                // If failed to move to primary position, try alternative from home
                ROS_WARN("Failed to move to viewing position for basket %zu", i);
                
                // Return to home position first (if not already there)
                execute_pose_target(move_group, home_pose);
                ros::Duration(1.0).sleep();
                
                // Try a different angle
                geometry_msgs::Pose alt_pose;
                alt_pose.position.x = 0.4;
                alt_pose.position.y = basket_pos.y > 0 ? -0.2 : 0.2; // Perpendicular approach
                alt_pose.position.z = 0.42;
                alt_pose.orientation = tf2::toMsg(q);
                
                ROS_INFO("Trying alternative position for basket %zu", i);
                if (execute_pose_target(move_group, alt_pose)) {
                    ros::Duration(2.0).sleep();
                    
                    // Get new point cloud
                    sensor_msgs::PointCloud2ConstPtr new_cloud_msg = 
                        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
                            "/r200/camera/depth_registered/points", nh_, ros::Duration(10.0));
                            
                    if (new_cloud_msg) {
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
                        pcl::fromROSMsg(*new_cloud_msg, *new_cloud);
                        
                        std::string color = detect_basket_color(new_cloud, basket_locs[i], *tf_buffer_, new_cloud_msg->header.frame_id);
                        if (color != "none") {
                            basket_colours[i] = color;
                            ROS_INFO("Basket %zu color from alternative position: %s", i, color.c_str());
                        }
                    }
                    
                    // Return to home position again
                    ROS_INFO("Returning to home position after alternative view of basket %zu", i);
                    execute_pose_target(move_group, home_pose);
                    ros::Duration(1.0).sleep();
                }
            }
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception processing basket %zu: %s", i, e.what());
            // Try to return to home if an exception occurs
            execute_pose_target(move_group, home_pose);
        }
    }
    
    // Populate the response with detected colors
    response.basket_colours = basket_colours;
    
    // Log final results
    ROS_INFO("Basket color detection results:");
    for (size_t i = 0; i < basket_colours.size(); i++) {
        ROS_INFO("Location %zu: %s", i, basket_colours[i].c_str());
    }

    ROS_INFO("Successfully completed basket color detection task");
    return true;
}

// -----------------------------------------------------------------------------
// Task 3: Object Detection and Sorting
// -----------------------------------------------------------------------------

/**
 * Execute a scan from a specific position
 * 
 * @param move_group MoveGroupInterface to control the robot
 * @param scan_position Position to scan from
 * @param results Scan results to be populated
 * @return True if scan was successful
 */
bool cw1::scan_from_position(
    MoveGroupInterface& move_group,
    const ScanPosition& scan_position,
    ScanResult& results)
{
    ROS_INFO("Moving to %s position (x=%.2f, y=%.2f, z=%.2f)...", 
            scan_position.name.c_str(), 
            scan_position.pose.position.x, 
            scan_position.pose.position.y, 
            scan_position.pose.position.z);

    if (!execute_pose_target(move_group, scan_position.pose)) {
        ROS_ERROR("Failed to move to %s position", scan_position.name.c_str());
        return false;
    }

    // Wait to stabilize
    ros::Duration(3.0).sleep();
        
    ROS_INFO("Successfully moved to %s position. Analyzing scene...", scan_position.name.c_str());
        
    // Get point cloud data
    ROS_INFO("Waiting for point cloud message...");
        
    // Add a slightly longer sleep to ensure the camera has time to stabilize
    ros::Duration(2.0).sleep();
        
    sensor_msgs::PointCloud2ConstPtr cloud_msg =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            "/r200/camera/depth_registered/points", nh_, ros::Duration(2.0));
        
    if (!cloud_msg) {
        ROS_ERROR("No point cloud received from %s position.", scan_position.name.c_str());
        return false;
    }
        
    // Process point cloud to extract objects
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    ROS_INFO("Received point cloud with %zu points from %s position", 
            cloud->points.size(), scan_position.name.c_str());
        
    // Transform the point cloud from the camera frame to the robot frame
    geometry_msgs::TransformStamped transform_stamped;
    try {
        transform_stamped = tf_buffer_->lookupTransform(
            "panda_link0",
            cloud_msg->header.frame_id,
            ros::Time(0),
            ros::Duration(1.0));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl_ros::transformPointCloud(*cloud, *cloud_transformed, transform_stamped.transform);
        cloud = cloud_transformed;
        ROS_INFO("Transformed point cloud to panda_link0 frame.");
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not transform point cloud to panda_link0: %s", ex.what());
        return false;
    }
        
    // Process the point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> new_clusters;
    bool processing_success = process_point_cloud(cloud, new_clusters);
    
    if (!processing_success) {
        ROS_ERROR("Failed to process point cloud from %s position", scan_position.name.c_str());
        return false;
    }
        
    ROS_INFO("Found %zu clusters from %s position", new_clusters.size(), scan_position.name.c_str());
        
    // Process each cluster with position-specific filtering
    for (size_t i = 0; i < new_clusters.size(); i++) {
        // Apply different point count thresholds based on scan position and object type
        if (scan_position.name == "center") {
            // In center scan, focus on boxes but with looser threshold
            if (new_clusters[i]->points.size() < 500) {  // Reduced from 1000
                ROS_INFO("Cluster %zu has only %zu points (< 500), likely noise, skipping", 
                        i, new_clusters[i]->points.size());
                continue;
            }
        } else {
            // In side scans, focus on baskets (typically > 3000 points)
            if (new_clusters[i]->points.size() < 3000) {
                ROS_INFO("Cluster %zu in %s scan has only %zu points (< 3000), skipping (focusing on baskets)",
                        i, scan_position.name.c_str(), new_clusters[i]->points.size());
                continue;
            }
        }
            
        // Calculate centroid for logging and position
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*new_clusters[i], centroid);
            
        ROS_INFO("Processing cluster %zu with %zu points at (%.3f, %.3f, %.3f)", 
                i, new_clusters[i]->points.size(), centroid[0], centroid[1], centroid[2]);
            
        // Process the cluster (determine object type, color, etc.)
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D<pcl::PointXYZRGB>(*new_clusters[i], min_pt, max_pt);
        float width = max_pt.x - min_pt.x;
        float height = max_pt.y - min_pt.y;
        float depth = max_pt.z - min_pt.z;
        float volume = width * height * depth;
            
        ROS_INFO("Cluster %zu dimensions: %.3f x %.3f x %.3f m (volume: %.6f m³)", 
                i, width, height, depth, volume);
            
        // Determine object type based on position and point count
        std::string object_type;
            
        // Expected dimensions
        const float basket_size = 0.1f;  // 10cm baskets
        const float box_size = 0.04f;    // 4cm boxes
        const float basket_volume = basket_size * basket_size * basket_size;  // 0.001 m³
        const float box_volume = box_size * box_size * box_size;              // 0.000064 m³
            
        if (scan_position.name == "center") {
            // In center scan, prefer box classification
            if (new_clusters[i]->points.size() >= 1000 && new_clusters[i]->points.size() <= 2500) {
                object_type = "box";
                ROS_INFO("  Center scan: Classifying as box based on point count (%zu points)", 
                        new_clusters[i]->points.size());
            } else {
                // Use volume for larger clusters
                float basket_diff = std::abs(volume - basket_volume);
                float box_diff = std::abs(volume - box_volume);
                    
                if (box_diff < basket_diff && volume < 0.001) {
                    object_type = "box";
                } else {
                    object_type = "basket";
                }
            }
        } else {
            // In side scans, only look for baskets
            object_type = "basket";
            ROS_INFO("  Side scan (%s): Classifying as basket", scan_position.name.c_str());
        }
            
        // Find dominant color
        std::string dominant_color = detect_dominant_color(
            new_clusters[i], PCLConstants::MIN_COLOR_POINTS);
            
        if (dominant_color != "none") {
            // Create location point
            geometry_msgs::PointStamped location;
            location.header.frame_id = "panda_link0";
            location.header.stamp = ros::Time::now();
            location.point.x = centroid[0];
            location.point.y = centroid[1];
            location.point.z = centroid[2];
                
            ROS_INFO("  Found from %s: %s %s at (%.3f, %.3f, %.3f)",
                    scan_position.name.c_str(), dominant_color.c_str(), object_type.c_str(),
                    centroid[0], centroid[1], centroid[2]);
                
            // Add to this position's results
            results.clusters.push_back(new_clusters[i]);
            results.colors.push_back(dominant_color);
            results.locations.push_back(location);
            results.point_counts.push_back(new_clusters[i]->points.size());
            results.types.push_back(object_type);
        } else {
            ROS_INFO("  No dominant color found for cluster %zu", i);
        }
            
        // Add additional filtering based on position and object type
        if (scan_position.name == "center" && object_type == "basket") {
            // In center, we're less confident about basket detection, so add extra validation
            if (dominant_color == "none" || new_clusters[i]->points.size() < 3000) {
                ROS_INFO("  Skipping possible basket in center scan due to low confidence");
                continue;
            }
        }
            
        if (scan_position.name != "center" && object_type == "box") {
            // In side scans, we're specifically avoiding box detection
            ROS_INFO("  Skipping box detected in side scan");
            continue;
        }
    }
        
    ROS_INFO("Scan from %s position found %zu objects (%s)", 
            scan_position.name.c_str(), 
            results.colors.size(),
            scan_position.name == "center" ? "primarily boxes" : "primarily baskets");
    return true;
}

/**
 * Add results from a scan with proper filtering
 * 
 * @param results Scan results to add
 * @param position_name Position name for logging
 * @param valid_clusters Output vector for clusters
 * @param object_colors Output vector for colors
 * @param object_locations Output vector for locations
 * @param point_counts Output vector for point counts
 * @param object_types Output vector for object types
 * @param basket_colors_added Set of already added basket colors
 */
void cw1::add_scan_results(
    const ScanResult& results,
    const std::string& position_name,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& valid_clusters,
    std::vector<std::string>& object_colors,
    std::vector<geometry_msgs::PointStamped>& object_locations,
    std::vector<size_t>& point_counts,
    std::vector<std::string>& object_types,
    std::set<std::string>& basket_colors_added)
{
    // If position_name is "reset", clear the color set and return
    if (position_name == "reset") {
        basket_colors_added.clear();
        ROS_INFO("Cleared static basket colors cache");
        return;
    }
    
    for (size_t i = 0; i < results.colors.size(); i++) {
        // Skip baskets if we already have a basket of this color
        if (results.types[i] == "basket") {
            if (basket_colors_added.count(results.colors[i]) > 0) {
                ROS_INFO("Skipping %s basket from %s scan - already have one of this color", 
                        results.colors[i].c_str(), position_name.c_str());
                continue;
            }
        }
        
        // For boxes, accept all of them - no filtering based on color
        if (results.types[i] == "box") {
            valid_clusters.push_back(results.clusters[i]);
            object_colors.push_back(results.colors[i]);
            object_locations.push_back(results.locations[i]);
            point_counts.push_back(results.point_counts[i]);
            object_types.push_back(results.types[i]);
            
            ROS_INFO("Added %s %s from %s scan at (%.3f, %.3f, %.3f) with %zu points",
                    results.colors[i].c_str(), 
                    results.types[i].c_str(), 
                    position_name.c_str(),
                    results.locations[i].point.x,
                    results.locations[i].point.y, 
                    results.locations[i].point.z,
                    results.point_counts[i]);
            continue;  // Skip the rest of the loop
        }
        
        // Handle baskets - check for duplicates of same basket from different angles
        bool is_duplicate_basket = false;
        for (size_t j = 0; j < object_locations.size(); j++) {
            if (object_types[j] == "basket" && object_colors[j] == results.colors[i]) {
                // We already have a basket of this color
                float x_diff = std::abs(results.locations[i].point.x - object_locations[j].point.x);
                float y_diff = std::abs(results.locations[i].point.y - object_locations[j].point.y);
                float z_diff = std::abs(results.locations[i].point.z - object_locations[j].point.z);
                
                float x_threshold = 0.05; 
                float y_threshold = 0.05;
                float z_threshold = 0.05;
                
                if (x_diff < x_threshold && y_diff < y_threshold && z_diff < z_threshold) {
                    is_duplicate_basket = true;
                    ROS_INFO("Found duplicate %s basket from %s scan (distance: x=%.3f, y=%.3f, z=%.3f)",
                            results.colors[i].c_str(), position_name.c_str(), x_diff, y_diff, z_diff);
                    
                    // Keep the one with more points
                    if (results.point_counts[i] > point_counts[j]) {
                        ROS_INFO("  Replacing existing basket with better quality version (points: %zu → %zu)",
                                point_counts[j], results.point_counts[i]);
                        
                        valid_clusters[j] = results.clusters[i];
                        object_locations[j] = results.locations[i];
                        point_counts[j] = results.point_counts[i];
                    }
                    break;
                }
            }
        }
        
        // Add this basket if it's not a duplicate
        if (!is_duplicate_basket) {
            valid_clusters.push_back(results.clusters[i]);
            object_colors.push_back(results.colors[i]);
            object_locations.push_back(results.locations[i]);
            point_counts.push_back(results.point_counts[i]);
            object_types.push_back(results.types[i]);
            
            // Mark this basket color as seen
            basket_colors_added.insert(results.colors[i]);
            
            ROS_INFO("Added %s %s from %s scan at (%.3f, %.3f, %.3f) with %zu points",
                    results.colors[i].c_str(), 
                    results.types[i].c_str(), 
                    position_name.c_str(),
                    results.locations[i].point.x,
                    results.locations[i].point.y, 
                    results.locations[i].point.z,
                    results.point_counts[i]);
        }
    }
}

/**
 * Execute box pick and place operations
 * 
 * @param move_group MoveGroupInterface to control
 * @param box_idx Index of the box to pick
 * @param basket_idx Index of the basket to place into
 * @param object_colors Vector of object colors
 * @param object_types Vector of object types
 * @param valid_clusters Vector of object clusters
 * @param object_locations Vector of object locations
 * @param home_joint_values Joint values for home position
 * @return True if operation was successful
 */
bool cw1::execute_box_placement(
    MoveGroupInterface& move_group,
    size_t box_idx,
    size_t basket_idx,
    const std::vector<std::string>& object_colors,
    const std::vector<std::string>& object_types,
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& valid_clusters,
    const std::vector<geometry_msgs::PointStamped>& object_locations,
    const std::vector<double>& home_joint_values)
{
    ROS_INFO("Planning to place %s box into matching %s basket", 
            object_colors[box_idx].c_str(), 
            object_colors[basket_idx].c_str());
    
    // Get object cluster for more accurate position calculation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr box_cluster = valid_clusters[box_idx];
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D<pcl::PointXYZRGB>(*box_cluster, min_pt, max_pt);
    
    // Calculate a more robust box position from the bounding box
    // Use the center of the top face of the box for better grasping
    double box_x = (min_pt.x + max_pt.x) / 2.0;
    double box_y = (min_pt.y + max_pt.y) / 2.0;
    double box_z = max_pt.z; // Use the top of the box
    
    // Get basket position
    const auto& basket_pos = object_locations[basket_idx].point;
    
    // Define safety margins
    double approach_offset = RobotConstants::APPROACH_OFFSET;
    double pickup_offset = 0.08;    // 8cm above the box for grasping
    double basket_approach_offset = 0.20; // 20cm above the basket
    
    // Add a height safety check to ensure we're not touching the ground
    if (box_z < 0.05) {  // If box appears to be very close to the ground
        ROS_WARN("Box height seems very low (%.3f). Adjusting height assumption.", box_z);
        box_z = 0.05;  // Set a minimum height of 5cm from the ground
        pickup_offset = 0.10;  // Use even more offset (10cm) for very low boxes
    }
    
    // Log the exact dimensions and positions for debugging
    ROS_INFO("Box dimensions: x: %.3f-%.3f, y: %.3f-%.3f, z: %.3f-%.3f",
            min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
    ROS_INFO("Using box position (%.3f, %.3f, %.3f) with pickup offset %.3f",
            box_x, box_y, box_z, pickup_offset);
    
    // Set fixed orientation
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, M_PI / 4.0);
    geometry_msgs::Quaternion fixed_orientation = tf2::toMsg(q);
    
    // Create pick and place parameters
    PickPlaceParams params;
    
    // Set object pose
    params.object_pose.position.x = box_x;
    params.object_pose.position.y = box_y;
    params.object_pose.position.z = box_z;
    params.object_pose.orientation = fixed_orientation;
    
    // Set place pose
    params.place_pose.position.x = basket_pos.x;
    params.place_pose.position.y = basket_pos.y;
    params.place_pose.position.z = basket_pos.z;
    params.place_pose.orientation = fixed_orientation;
    
    // Set approach and pickup offsets
    params.approach_offset = approach_offset;
    params.pickup_offset = pickup_offset;
    params.place_approach_offset = basket_approach_offset;
    
    // Execute the pick and place operation
    bool result = execute_pick_and_place(move_group, params);
    
    // Return to home position after placement
    return_to_home_position(move_group, home_joint_values);
    
    return result;
}

bool cw1::t3_callback(cw1_world_spawner::Task3Service::Request &request,
                     cw1_world_spawner::Task3Service::Response &response)
{
    // Clear the TF buffer to avoid using stale transforms from previous runs
    if (tf_buffer_) {
        ROS_INFO("Clearing TF buffer for fresh transforms");
        tf_buffer_->clear();
    }
    
    // Small delay to ensure the system is ready (especially for second runs)
    ros::Duration(1.0).sleep();

    ROS_INFO("Task 3: Starting enhanced area scanning to find all objects");

    // Initialize MoveIt! interface
    MoveGroupInterface move_group = initialize_move_group("panda_arm");
    
    // Store current joint values for home position
    std::vector<double> home_joint_values = store_home_position(move_group);
    
    // Define observation poses for different areas
    std::vector<ScanPosition> scan_positions;
    
    // Central position
    ScanPosition center_pos;
    center_pos.pose.position.x = 0.42;
    center_pos.pose.position.y = 0.00;
    center_pos.pose.position.z = 0.8;
    center_pos.pose.orientation.x = 0.92357;
    center_pos.pose.orientation.y = -0.38329;
    center_pos.pose.orientation.z = -0.0097;
    center_pos.pose.orientation.w = 0.00453;
    center_pos.name = "center";
    scan_positions.push_back(center_pos);
    
    // Left position
    ScanPosition left_pos = center_pos;
    left_pos.pose.position.y = 0.20;
    left_pos.name = "left";
    scan_positions.push_back(left_pos);
    
    // Right position
    ScanPosition right_pos = center_pos;
    right_pos.pose.position.y = -0.20;
    right_pos.name = "right";
    scan_positions.push_back(right_pos);
    
    // Store results from each scanning position separately
    std::vector<ScanResult> scan_results(scan_positions.size());
    std::vector<bool> scan_success(scan_positions.size(), false);
    
    // Execute the scans from all positions
    for (size_t i = 0; i < scan_positions.size(); i++) {
        scan_success[i] = scan_from_position(move_group, scan_positions[i], scan_results[i]);
    }
    
    // Check if at least one scan was successful
    if (std::none_of(scan_success.begin(), scan_success.end(), [](bool b) { return b; })) {
        ROS_ERROR("All scan positions failed. Task 3 failed.");
        return false;
    }

    // Now combine results and filter duplicates
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> valid_clusters;
    std::vector<std::string> object_colors;
    std::vector<geometry_msgs::PointStamped> object_locations;
    std::vector<size_t> point_counts;
    std::vector<std::string> object_types;
    
    // Set to track basket colors already added
    std::set<std::string> basket_colors_added;
    
    // Clear any static variables that might persist between runs
    static bool is_first_run = true;
    if (!is_first_run) {
        ROS_INFO("This appears to be a subsequent run. Clearing cached data...");
        // Force clear the static basket_colors_added set
        ScanResult empty_result;
        add_scan_results(empty_result, "reset", valid_clusters, object_colors, 
                        object_locations, point_counts, object_types, basket_colors_added);
    }
    is_first_run = false;
    
    // Add results from each scan
    ROS_INFO("Combining results from all scans and removing duplicates...");
    for (size_t i = 0; i < scan_positions.size(); i++) {
        if (scan_success[i]) {
            add_scan_results(
                scan_results[i], 
                scan_positions[i].name, 
                valid_clusters, 
                object_colors, 
                object_locations, 
                point_counts, 
                object_types,
                basket_colors_added);
        }
    }

    // Print summary of objects found
    ROS_INFO("Task 3 scanning complete. Found %zu unique objects:", object_colors.size());

    // First print baskets
    ROS_INFO("Baskets found:");
    for (size_t i = 0; i < object_colors.size(); i++) {
        if (object_types[i] == "basket") {
            ROS_INFO("  Basket %zu - %zu points - %s - location: (%.3f, %.3f, %.3f)", 
                    i+1, 
                    point_counts[i],
                    object_colors[i].c_str(), 
                    object_locations[i].point.x,
                    object_locations[i].point.y, 
                    object_locations[i].point.z);
        }
    }

    // Then print boxes
    ROS_INFO("Boxes found:");
    for (size_t i = 0; i < object_colors.size(); i++) {
        if (object_types[i] == "box") {
            ROS_INFO("  Box %zu - %zu points - %s - location: (%.3f, %.3f, %.3f)", 
                    i+1, 
                    point_counts[i],
                    object_colors[i].c_str(), 
                    object_locations[i].point.x,
                    object_locations[i].point.y, 
                    object_locations[i].point.z);
        }
    }
    
    // Move back to home position before starting pick and place operations
    ROS_INFO("Moving back to home position before starting pick and place...");
    return_to_home_position(move_group, home_joint_values);
    ros::Duration(1.0).sleep(); // Brief pause to stabilize
    
    // Now proceed with pick and place operations for each box
    for (size_t box_idx = 0; box_idx < object_colors.size(); box_idx++) {
        if (object_types[box_idx] != "box") continue;
        
        // Find matching basket
        size_t matching_basket_idx = SIZE_MAX;
        for (size_t basket_idx = 0; basket_idx < object_colors.size(); basket_idx++) {
            if (object_types[basket_idx] == "basket" && 
                object_colors[basket_idx] == object_colors[box_idx]) {
                matching_basket_idx = basket_idx;
                break;
            }
        }
        
        // Skip if no matching basket found
        if (matching_basket_idx == SIZE_MAX) {
            ROS_INFO("No matching basket found for %s box", object_colors[box_idx].c_str());
            continue;
        }
        
        // Execute box placement
        bool placement_success = execute_box_placement(
            move_group, 
            box_idx, 
            matching_basket_idx,
            object_colors,
            object_types,
            valid_clusters,
            object_locations,
            home_joint_values);
        
        if (placement_success) {
            ROS_INFO("Successfully placed %s box into matching basket", 
                   object_colors[box_idx].c_str());
        } else {
            ROS_ERROR("Failed to place %s box into matching basket", 
                     object_colors[box_idx].c_str());
        }
    }
    
    // Return to home pose at the end
    ROS_INFO("Task 3 completed. Returning to home position...");
    return_to_home_position(move_group, home_joint_values);
    
    return true;
}



// For backward compatibility - correct implementations
bool cw1::moveGripper(double target_width) {
    // Static implementation (can't call instance methods)
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

bool cw1::executePoseTarget(MoveGroupInterface &move_group,
                           const geometry_msgs::Pose &target, int max_attempts) {
    // Static implementation (can't call instance methods)
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

void cw1::applyVX(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered) {
    // Static implementation (can't call instance methods)
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.001f, 0.001f, 0.001f);
    sor.filter(*cloud_filtered);
    
    ROS_INFO("Downsampled cloud from %zu to %zu points", 
             cloud->points.size(), cloud_filtered->points.size());
}

void cw1::segGreenBackground(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    // Static implementation (can't call instance methods)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    for (const auto &point : cloud->points) {
        // Skip points too close to the table
        if (point.z < 0.02) {
            continue;
        }
        
        // Convert RGB to HSV for better color filtering
        float r = point.r / 255.0f;
        float g = point.g / 255.0f;
        float b = point.b / 255.0f;
        
        float max_val = std::max(std::max(r, g), b);
        float min_val = std::min(std::min(r, g), b);
        float diff = max_val - min_val;
        
        float h = 0.0f, s = 0.0f, v = max_val;
        
        if (max_val > 0.0f) {
            s = diff / max_val;
        }
        
        if (diff > 0.0f) {
            if (max_val == r) {
                h = 60.0f * fmodf(((g - b) / diff), 6.0f);
            } else if (max_val == g) {
                h = 60.0f * (((b - r) / diff) + 2.0f);
            } else {
                h = 60.0f * (((r - g) / diff) + 4.0f);
            }
            
            if (h < 0.0f) {
                h += 360.0f;
            }
        }
        
        // Check for green (typically 80-170° hue) and dark colors
        bool is_green = (h >= 80.0f && h <= 170.0f && s > 0.2f && v > 0.2f);
        bool is_too_dark = (v < 0.1f);
        
        // Keep point only if it's not green and not too dark
        if (!is_green && !is_too_dark) {
            cloud_filtered->points.push_back(point);
        }
    }
    
    cloud_filtered->width = static_cast<uint32_t>(cloud_filtered->points.size());
    cloud_filtered->height = 1;
    cloud_filtered->is_dense = true;
    
    *cloud = *cloud_filtered;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> 
cw1::segClusters(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
    // Static implementation (can't call instance methods)
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    
    // Create KD-tree for efficient nearest neighbor search
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    tree->setInputCloud(cloud);

    // Extract clusters
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.01);  // 1cm
    ec.setMinClusterSize(20);      // Minimum points per cluster
    ec.setMaxClusterSize(10000);   // Maximum points per cluster
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    ROS_INFO("Found %zu clusters", cluster_indices.size());

    // Process each cluster
    for (const auto &indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>());
        
        for (const int idx : indices.indices) {
            cluster->points.push_back(cloud->points[idx]);
        }
        
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        
        clusters.push_back(cluster);
    }

    return clusters;
}