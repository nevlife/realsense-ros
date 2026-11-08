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

#include <cstring>

using namespace realsense2_camera;

// --- image_rcl_publisher implementation ---
image_rcl_publisher::image_rcl_publisher( RosNodeBase & node,
                                          const std::string & topic_name,
                                          const rmw_qos_profile_t & qos )
    : image_publisher_impl( node, topic_name, rclcpp::QoS( rclcpp::QoSInitialization::from_rmw( qos ), qos ) )
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
    auto data = loan.alloc_data( row_bytes * image.rows );
    if( ! loan || data.empty() )
        return true;  // no free slot, or the image is over the slot size: the frame is dropped
    loan.set_header( header );
    loan.set_height( image.rows );
    loan.set_width( image.cols );
    loan.set_encoding( encoding );
    loan.set_is_bigendian( false );
    loan.set_step( row_bytes );
    for( int row = 0; row < image.rows; ++row )
        std::memcpy( data.data() + row * row_bytes, image.ptr( row ), row_bytes );
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
