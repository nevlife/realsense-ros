// Copyright 2023 RealSense, Inc. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "ros_node_base.h"
#include <sensor_msgs/msg/image.hpp>
#include <flux_wrapper/publisher.hpp>
#include <opencv2/core.hpp>

#include <image_transport/image_transport.hpp>

namespace realsense2_camera {
class image_publisher
{
public:
    virtual void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) = 0;
    virtual size_t get_subscription_count() const = 0;
    // Writes the image straight into the publisher's buffer, skipping the ROS message. False
    // when this publisher has no such buffer and publish() has to be used.
    virtual bool publish_in_place( const cv::Mat & /*image*/,
                                   const std::string & /*encoding*/,
                                   const std_msgs::msg::Header & /*header*/ )
    {
        return false;
    }
    virtual ~image_publisher() = default;
};

// Native RCL implementation of an image publisher (needed for intra-process communication).
// With ENABLE_FLUX=1 it publishes over flux instead.
class image_rcl_publisher : public image_publisher
{
public:
    image_rcl_publisher( RosNodeBase & node,
                         const std::string & topic_name,
                         const rmw_qos_profile_t & qos,
                         flux_wrapper::Device device = flux_wrapper::Device::Host );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    bool publish_in_place( const cv::Mat & image,
                           const std::string & encoding,
                           const std_msgs::msg::Header & header ) override;
    size_t get_subscription_count() const override;

private:
    flux_wrapper::Publisher< sensor_msgs::msg::Image > image_publisher_impl;
};

// image_transport implementation of an image publisher (adds a compressed image topic)
class image_transport_publisher : public image_publisher
{
public:
    image_transport_publisher( rclcpp::Node & node,
                               const std::string & topic_name,
                               const rmw_qos_profile_t & qos );
    void publish( sensor_msgs::msg::Image::UniquePtr image_ptr ) override;
    size_t get_subscription_count() const override;

private:
    std::shared_ptr< image_transport::Publisher > image_publisher_impl;
};

}  // namespace realsense2_camera
