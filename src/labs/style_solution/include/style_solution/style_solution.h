/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2019-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef STYLE_SOLUTION_H_
#define STYLE_SOLUTION_H_

#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>

// ROS includes
#include <geometry_msgs/Pose.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// TF specific includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

/** \brief Style Solution.
  *
  * \author Dimitrios Kanoulas
  */
class StyleSolution
{
  public:
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    StyleSolution (ros::NodeHandle &nh);

    /** \brief Point Cloud CallBacceid55k function.
      * 
      * \input[in] cloud_input a PCLPointCloud2 const pointer
      */
    void
    cloudCallBackOne (const pcl::PCLPointCloud2ConstPtr& cloud_input);

    /** \brief Point Cloud CallBack function.
      * 
      * \input[in] cloud_input a PointCloud2 sensor_msgs const pointer
      */
    void
    cloudCallBackTwo (const sensor_msgs::PointCloud2ConstPtr& cloud_input_msg);

    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    pubFilteredPC (ros::Publisher &pc_pub, pcl::PCLPointCloud2 &pc);

    /** \brief Point Cloud publisher.
      * 
      *  \input pc_pub ROS publisher
      *  \input pc point cloud to be published
      */
    void
    pubFilteredPCMsg (ros::Publisher &pc_pub, sensor_msgs::PointCloud2 &pc);

    /** \brief Print a number.
      * 
      *  \input[in] num_one first double number to be added
      *  \input[in] num_two second double number to be added
      *  
      *  \output true if z was calculated
      */
    bool
    findZ (double num_one, double num_two);

    /** \brief Print a number.
      * 
      *  \input[in] num_one first double number to be added
      *  \input[in] num_two second double number to be added
      *  
      *  \output true if z was calculated
      */
    bool
    findZBetter (double num_one, double num_two);

    /** \brief Print a number.
      * 
      *  \input[in] x the x-axis scale
      *  \input[in] y the y-axis scale
      *  \input[in] z the z-axis scale
      *  
      *  \output true if three numbers are added
      */
    void
    publishPose (double x, double y, double z);
    
  public:
    /** \brief Node handle. */
    ros::NodeHandle g_nh;

    /** \brief ROS publishers. */
    ros::Publisher g_pub_one, g_pub_two;

    /** \brief ROS geometry message pose. */
    geometry_msgs::Pose g_pose_msg;

    /** \brief ROS pose publishers. */
    ros::Publisher g_pub_pose;
  
  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif