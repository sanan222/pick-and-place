/* feel free to change any part of this file, or delete this file. In general,
you can do whatever you want with this template code, including deleting it all
and starting from scratch. The only requirment is to make sure your entire 
solution is contained within the cw2_team_09 package */

// include guards, prevent .h file being defined multiple times (linker error)
#ifndef CW2_CLASS_H_
#define CW2_CLASS_H_

// Standard includes
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <sstream>
#include <algorithm>
#include <map>
#include <set>

// ROS includes
#include <ros/ros.h>
#include <ros/duration.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// include services from the spawner package - we will be responding to these
#include "cw2_world_spawner/Task1Service.h"
#include "cw2_world_spawner/Task2Service.h"
#include "cw2_world_spawner/Task3Service.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Type aliases for commonly used types
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using Plan = MoveGroupInterface::Plan;
using MoveItErrorCode = moveit::core::MoveItErrorCode;

// Color definition structure
struct ColorRef {
    std::string name;
    uint8_t r, g, b;
};

/**
 * Parameters for pick and place operations
 */
struct PickPlaceParams {
    geometry_msgs::Pose object_pose;       // Position of object to pick
    geometry_msgs::Pose place_pose;        // Position to place the object
    double approach_offset;                // Distance above object for approach
    double pickup_offset;                  // Distance above object for pickup
    double place_approach_offset;          // Distance above place position for approach
    bool release_at_approach;              // Whether to release at approach or place position
    std::string shape_type;               // Shape of the object
    
    // Constructor with default values
    PickPlaceParams() 
        : approach_offset(0.15),
          pickup_offset(0.12),
          place_approach_offset(0.30),
          release_at_approach(true) {}
};

// Structure to store scan position details
struct ScanPosition {
    geometry_msgs::Pose pose;
    std::string name;
};

// Structure to store scan results
struct ScanResult {
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
    std::vector<std::string> colors;
    std::vector<geometry_msgs::PointStamped> locations;
    std::vector<size_t> point_counts;
    std::vector<std::string> types;
};

// HSV color representation (used in color utilities)
struct HSV {
    float h, s, v;
};

class cw2
{
public:
    /* ----- class member functions ----- */

    // constructor
    cw2(ros::NodeHandle nh);

    // service callbacks for tasks 1, 2, and 3
    bool t1_callback(cw2_world_spawner::Task1Service::Request &request,
                    cw2_world_spawner::Task1Service::Response &response);
    bool t2_callback(cw2_world_spawner::Task2Service::Request &request,
                    cw2_world_spawner::Task2Service::Response &response);
    bool t3_callback(cw2_world_spawner::Task3Service::Request &request,
                    cw2_world_spawner::Task3Service::Response &response);

    // Function to get basket colors using PCL
    std::vector<std::string> getBasketColoursPCL(
        const std::vector<geometry_msgs::PointStamped>& basket_locs,
        ros::NodeHandle& nh);

    // Function to visualize basket locations
    void visualizeBasketLocations(
        const std::vector<geometry_msgs::PointStamped>& basket_locs, 
        const std::string& frame_id);

    // Function to visualize clusters
    void visualizeCluster(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster, 
        int id, double r, double g, double b);

    // Movement and control functions
    bool move_gripper(double target_width);
    bool execute_pose_target(
        MoveGroupInterface &move_group,
        const geometry_msgs::Pose &target, 
        int max_attempts = 3);
    MoveGroupInterface initialize_move_group(const std::string& group_name = "panda_arm");
    std::vector<double> store_home_position(MoveGroupInterface& move_group);
    bool return_to_home_position(
        MoveGroupInterface& move_group, 
        const std::vector<double>& home_joint_values);
    bool execute_pick_and_place(
        MoveGroupInterface& move_group,
        const PickPlaceParams& params);
        
    // Point cloud processing functions
    void apply_voxel_filter(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered);
    void remove_green_background(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extract_clusters(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    bool process_point_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& output_clusters);
    std::string detect_dominant_color(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster,
        int min_votes_needed = 20);
    std::string detect_basket_color(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const geometry_msgs::PointStamped& basket_loc,
        tf2_ros::Buffer& tf_buffer,
        const std::string& cloud_frame,
        double search_radius = 0.12);
    void describe_point_cloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    
    // Task 3 specific functions
    bool scan_from_position(
        MoveGroupInterface& move_group,
        const ScanPosition& scan_position,
        ScanResult& results);
    void add_scan_results(
        const ScanResult& results,
        const std::string& position_name,
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& valid_clusters,
        std::vector<std::string>& object_colors,
        std::vector<geometry_msgs::PointStamped>& object_locations,
        std::vector<size_t>& point_counts,
        std::vector<std::string>& object_types,
        std::set<std::string>& basket_colors_added);
    bool execute_box_placement(
        MoveGroupInterface& move_group,
        size_t box_idx,
        size_t basket_idx,
        const std::vector<std::string>& object_colors,
        const std::vector<std::string>& object_types,
        const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& valid_clusters,
        const std::vector<geometry_msgs::PointStamped>& object_locations,
        const std::vector<double>& home_joint_values);

    // Viewpoint handling functions
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr process_viewpoint_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
        const std::string& viewpoint_name);
        
    bool visualize_viewpoint_objects(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
        const std::string& viewpoint_name,
        double viewpoint_angle,
        std::vector<cv::Mat>& saved_images);

    cv::Mat combine_viewpoint_images(
        const std::vector<cv::Mat>& images, 
        int num_viewpoints);
        
    cv::Mat create_top_down_view(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr capture_viewpoint_cloud(
        MoveGroupInterface& move_group,
        const geometry_msgs::Pose& viewpoint_pose,
        const std::string& viewpoint_name);

    /**
     * Merge a set of viewpoint point clouds and publish the result.
     *
     * @param viewpoint_clouds A vector of point clouds captured from different viewpoints (all transformed to the same coordinate frame).
     */
    void mergeAndPublishClouds(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& viewpoint_clouds);
    
    /* ----- class member variables ----- */
    ros::NodeHandle nh_;
    ros::ServiceServer t1_service_;
    ros::ServiceServer t2_service_;
    ros::ServiceServer t3_service_;
    ros::Publisher marker_pub_;
    ros::Publisher merged_cloud_pub_;

    // Transform and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Reference color definitions
    const std::vector<ColorRef> COLOR_REFS = {
        {"blue", 25, 25, 200},
        {"red", 200, 25, 25},
        {"purple", 200, 25, 200}
    };
    
private:
    // Private helper functions
    double detect_orientation_opencv(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    bool refineObjectPose(geometry_msgs::Pose &object_pose);
    bool visualServoingAdjustment(MoveGroupInterface &move_group, 
                                 geometry_msgs::Pose &target_pose, 
                                 int max_attempts);

    /**
     * Process point cloud specifically for orientation detection
     * Preserves more of the object shape for better contour detection
     * 
     * @param input_cloud Raw input point cloud
     * @param object_center Approximate center of the object
     * @param search_radius Radius to search around the object center
     * @return Processed point cloud containing just the object
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr process_cloud_for_orientation(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
        const geometry_msgs::Point& object_center,
        double search_radius);

    // Store last detected size for objects
    int last_detected_size_mm_ = 40;  // Default size is 40mm
};

#endif // end of include guard for CW2_CLASS_H_