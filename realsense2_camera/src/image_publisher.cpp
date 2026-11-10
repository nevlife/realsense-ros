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

#include <image_publisher.h>

using namespace realsense2_camera;

// --- image_rcl_publisher implementation ---
namespace
{
flux_wrapper::PublisherOptions flux_options( flux_wrapper::Device device )
{
    flux_wrapper::PublisherOptions options;
    options.device = device;
    return options;
}
}  // namespace

image_rcl_publisher::image_rcl_publisher( RosNodeBase & node,
                                          const std::string & topic_name,
                                          const rmw_qos_profile_t & qos,
                                          flux_wrapper::Device device )
    : image_publisher_impl( node, topic_name, rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos ), qos ),
                            flux_options( device ) )
{
}

void image_rcl_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    image_publisher_impl.publish( std::move( image_ptr ) );
}

bool image_rcl_publisher::publish_in_place( const cv::Mat & image,
                                            const std::string & encoding,
                                            const std_msgs::msg::Header & header )
{
    // The ROS path keeps the upstream message path, which intra-process delivery relies on.
    if( ! image_publisher_impl.uses_flux() )
        return false;
    auto loan = image_publisher_impl.loan();
    const size_t row_bytes = image.cols * image.elemSize();
    // Host memory, or device memory with flux_gpu_images; copy_from takes one contiguous run.
    const cv::Mat packed = image.isContinuous() ? image : image.clone();
    auto data = loan.alloc_buffer( row_bytes * image.rows );
    if( ! loan || ! data || ! data.copy_from( packed.data, row_bytes * image.rows ) )
        return true;  // no free slot, or the image is over the slot size: the frame is dropped
    loan.set_header( header );
    loan.set_height( image.rows );
    loan.set_width( image.cols );
    loan.set_encoding( encoding );
    loan.set_is_bigendian( false );
    loan.set_step( row_bytes );
    image_publisher_impl.publish( std::move( loan ) );
    return true;
}

size_t image_rcl_publisher::get_subscription_count() const
{
    return image_publisher_impl.get_subscription_count();
}

// --- image_transport_publisher implementation ---
image_transport_publisher::image_transport_publisher( rclcpp::Node & node,
                                                      const std::string & topic_name,
                                                      const rmw_qos_profile_t & qos )
{
#if defined( LYRICAL ) || defined( ROLLING )
    // On Rolling/Lyrical, image_transport::create_publisher deduces NodeT
    // from a Node reference (it then calls node.get_node_base_interface()),
    // and the QoS argument is rclcpp::QoS rather than rmw_qos_profile_t.
    rclcpp::QoS rclcpp_qos( rclcpp::QoSInitialization::from_rmw( qos ), qos );
    image_publisher_impl = std::make_shared< image_transport::Publisher >(
        image_transport::create_publisher( node, topic_name, rclcpp_qos ) );
#else
    image_publisher_impl = std::make_shared< image_transport::Publisher >(
        image_transport::create_publisher( &node, topic_name, qos ) );
#endif
}
void image_transport_publisher::publish( sensor_msgs::msg::Image::UniquePtr image_ptr )
{
    image_publisher_impl->publish( *image_ptr );
}

size_t image_transport_publisher::get_subscription_count() const
{
    return image_publisher_impl->getNumSubscribers();
}
