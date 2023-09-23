/*
# Copyright (c) 2022 José Miguel Guerrero Hernández
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
*/

#include <memory>
#include <chrono>
#include <thread>
#include <filesystem>
#include <fstream>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <image_transport/image_transport.hpp>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/dnn.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class ComputerPublisher : public rclcpp::Node
{
  public:
    std::string dataset_path;
    std::string frame_id, global_frame_id;
    int max_files;
    int wait_millis;
    double fx, fy, cx, cy, factor;
    ComputerPublisher()
    : Node("computer_vision_publisher")
    {
      auto qos = rclcpp::QoS( rclcpp::QoSInitialization( RMW_QOS_POLICY_HISTORY_KEEP_LAST, 5 ));
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
      "cv_image", qos);

      publisher_3d_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "cloud_in", qos);

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      dataset_path = declare_parameter("dataset_path", "");
      max_files = declare_parameter("max_files", 1000000);
      frame_id = declare_parameter("frame_id", "sensor");
      global_frame_id = declare_parameter("global_frame_id", "map");
      wait_millis = declare_parameter("wait_millis", 200);
      // 7-Scenes Dataset's Kinect Intrinsic Parameters
      fx = declare_parameter("fx", 585.0);
      fy = declare_parameter("fy", 585.0);
      cx = declare_parameter("cx", 320.0);
      cy = declare_parameter("cy", 240.0);
      factor = declare_parameter("factor", 1000.0);

    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_3d_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void broadcast_transform(const Eigen::Isometry3d& pose)
    {
        Eigen::Vector3d T = pose.translation();
        Eigen::Matrix3d rot = pose.rotation();
        Eigen::Quaterniond q(rot);

        geometry_msgs::msg::TransformStamped tf_pose;

        // Read message content and assign it to
        // corresponding tf variables
        tf_pose.header.stamp = this->now();
        tf_pose.header.frame_id = global_frame_id;
        tf_pose.child_frame_id = frame_id;

        tf_pose.transform.translation.x = T.x();
        tf_pose.transform.translation.y = T.y();
        tf_pose.transform.translation.z = T.z();

        tf_pose.transform.rotation.x = q.x();
        tf_pose.transform.rotation.y = q.y();
        tf_pose.transform.rotation.z = q.z();
        tf_pose.transform.rotation.w = q.w();

        // broadcast tf
        tf_broadcaster_->sendTransform(tf_pose);
    }
    void publish_color_img(const cv::Mat& color_img)
    {
          // >> message to be sent
          sensor_msgs::msg::Image out_image; 
          out_image.header.stamp = this->now();
          out_image.header.frame_id = frame_id;

          cv_bridge::CvImage img_bridge = cv_bridge::CvImage(out_image.header, sensor_msgs::image_encodings::BGR8, color_img);

          // from cv_bridge to sensor_msgs::Image
          img_bridge.toImageMsg(out_image); 
          // publish image 
          publisher_->publish(out_image);
    }

    void publishPointcloud(const cv::Mat& color_img /*24-bit RGB image*/,
                           const cv::Mat& depth_img /*depth_img*/)
    {


      pcl::PointCloud<pcl::PointXYZ> point_cloud;

      for(int i = 0; i < depth_img.rows; i++)
      {
        for(int j = 0; j < depth_img.cols; j++)
        {

          uint16_t depth_mm = depth_img.at<uint16_t>(i,j);

          if (depth_mm == 65535 || depth_mm == 0)
          {
            continue;
          }
          else
          {
            pcl::PointXYZ point;
            point.z = depth_mm/factor;
            point.x = (j - cx)*point.z / fx;
            point.y = (i - cy)*point.z / fy;

            // Add color dimensions when available
            // cv::Vec3b intensity = img.at<cv::Vec3b>(i, j);
            // uchar blue = intensity.val[0];
            // uchar green = intensity.val[1];
            // uchar red = intensity.val[2];

            point_cloud.push_back(point);
          }
        }
      }

      sensor_msgs::msg::PointCloud2 cloud_out;
      pcl::toROSMsg(point_cloud, cloud_out);

      cloud_out.header.stamp = this->now();
      cloud_out.header.frame_id = frame_id;

      // publish both pointcloud 
      publisher_3d_->publish(cloud_out);

    }

};

bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void file2eigen(std::string path, Eigen::Matrix4d& pose)
{

    std::fstream file;
    std::string word, t, q, filename;
 
    // filename of the file
    filename = path;
 
    // opening file
    file.open(filename.c_str());
 
    // extracting words from the file
    int i = 0;
    while (file >> word)
    {
        double number = std::stod(word); 

        pose(i/4, i%4) = number;

        // displaying content
        i++;
    }
}

 void ReadData(const std::string& dataset, std::vector<std::string>& poses, 
                                           std::vector<std::string>& color_imgs, 
                                           std::vector<std::string>& depth_imgs)
{
  if(!std::filesystem::exists(dataset))
  {
    throw std::runtime_error("Calibration file not found");
  }
  
  for (const auto & entry : fs::directory_iterator(dataset))
  {
    if(ends_with(entry.path(), ".txt"))
    {
        poses.push_back(entry.path());

    }
    else if(ends_with(entry.path(), ".color.png"))
    {
        color_imgs.push_back(entry.path());

    }
    else if(ends_with(entry.path(), ".depth.png"))
    {
        depth_imgs.push_back(entry.path());
    }

  }

  std::sort(poses.begin(), poses.end());
  std::sort(color_imgs.begin(), color_imgs.end());
  std::sort(depth_imgs.begin(), depth_imgs.end());

}


std::vector<Eigen::Isometry3d> ReadPoses(const std::vector<std::string>& poses_files)
{

  std::vector<Eigen::Isometry3d> poses;

  for(const auto& pose_file: poses_files)
  {
    Eigen::Matrix4d pose;
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos;

    file2eigen(pose_file, pose);

    pos = pose.block(0, 3, 3, 1);
    rot = pose.block(0, 0, 3, 3);

    poses.emplace_back(Eigen::Translation3d(pos) * Eigen::Quaterniond(rot));

  }

  return poses;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = std::make_shared<ComputerPublisher>();

  // Containers for storing the data to be processed
  std::vector<std::string> poses_files, color_imgs_files, depth_imgs_files;
  
  if (!publisher_node->dataset_path.empty())
  {
      try
      {
        // Read all file names
        ReadData(publisher_node->dataset_path, poses_files, color_imgs_files, depth_imgs_files); 
        RCLCPP_INFO(publisher_node->get_logger(),"Dataset Loaded Succesfully");
      }
      catch(...)
      {
        RCLCPP_INFO(publisher_node->get_logger(),"Please provide a valid seq-XX dataset absolute path (where XX is the equence number) in the params/params.yaml files ");
      }
  }
  else
  {
    RCLCPP_INFO(publisher_node->get_logger(),"Dataset Path Empty. Please provide the seq-XX dataset absolute path (where XX is the equence number) in the params/params.yaml files ");
  }

  // Get std::vector of Isometry3d poses
  const auto poses = ReadPoses(poses_files);

  int max_pointclouds = std::min(static_cast<size_t>(publisher_node->max_files), poses_files.size());

  color_imgs_files.resize(max_pointclouds);
  depth_imgs_files.resize(max_pointclouds);

  // Manual Calibration
  const auto calibration = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX());

  for (size_t count = 0; (count < max_pointclouds && rclcpp::ok()); count++)
  {

    RCLCPP_INFO(publisher_node->get_logger(),("PUBLISHING: "+poses_files[count]).c_str());

    // Read depth and color images
    const auto& color_img_filename = color_imgs_files[count];
    const auto& depth_img_filename = depth_imgs_files[count];
    const cv::Mat color_img = cv::imread(color_img_filename, cv::IMREAD_COLOR);
    const cv::Mat depth_img = cv::imread(depth_img_filename, -1);    
    
    // Manual Calibration for better visualization
    const Eigen::Isometry3d transform = calibration*poses[count];
    // Broadcast Transformation
    publisher_node->broadcast_transform(transform);
    // Publish RGB Image
    publisher_node->publish_color_img(color_img);
    // Publish Pointcloud Image
    publisher_node->publishPointcloud(color_img, depth_img);
    //wait some milliseconds before next iteration
    std::this_thread::sleep_for(std::chrono::milliseconds(std::chrono::milliseconds(publisher_node->wait_millis)));
  }
    
  rclcpp::shutdown();
  return 0;
}