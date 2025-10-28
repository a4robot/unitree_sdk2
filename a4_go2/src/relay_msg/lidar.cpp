#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"
#include "ros/console.h"

#include "a4/ros/node_handle.hpp"

std_msgs::Header header;
ros::Publisher publisher;

void Handler( const void* message )
{
    const sensor_msgs::msg::dds_::PointCloud2_ *cloud_msg = (const sensor_msgs::msg::dds_::PointCloud2_ *)message;

    sensor_msgs::PointCloud2 ros_msg;
    ros_msg.header = header;
    ros_msg.header.stamp = ros::Time::now();

    ros_msg.height = cloud_msg->height();
    ros_msg.width = cloud_msg->width();
    ros_msg.is_bigendian = cloud_msg->is_bigendian();
    ros_msg.point_step = cloud_msg->point_step();
    ros_msg.row_step = cloud_msg->row_step();
    ros_msg.is_dense = cloud_msg->is_dense();
    const std::vector<::sensor_msgs::msg::dds_::PointField_> _fields = cloud_msg->fields();
    ros_msg.fields.resize( _fields.size() );
    for( unsigned int idx = 0 ; idx < _fields.size(); idx++ )
    {
        ros_msg.fields[ idx ].name = _fields[ idx ].name(); 
        ros_msg.fields[ idx ].offset = _fields[ idx ].offset();
        ros_msg.fields[ idx ].datatype = _fields[ idx ].datatype();
        ros_msg.fields[ idx ].count = _fields[ idx ].count();
    }
    ros_msg.header.stamp = ros::Time::now();

    ROS_INFO_THROTTLE( 1, "Publish point cloud to %s points %u", publisher.getTopic().c_str(), ros_msg.height * ros_msg.width );
    publisher.publish( ros_msg );
}

int main( int argc, char** argv )
{
    ros::init( argc, argv, "unitree_lidar");

    ros::NodeHandle nh;

    std::string interface = a4_ros::param< std::string >( "~interface", "lo", true );

    // Topic choice "rt/utilidar/cloud" and "ut/utilidar/cloud_desckewed"
    std::string topic_unitree = a4_ros::param< std::string >( "~topic/unitree", "rt/utilidar/cloud", true );
    std::string output_frame = a4_ros::param< std::string >( "~frame_id", "odom", true );
    std::string topic_output = a4_ros::param_interface( "~topic/output", topic_unitree, true );

    header.frame_id = output_frame;

    publisher = nh.advertise< sensor_msgs::PointCloud2 >( topic_output, 1, true );

    unitree::robot::ChannelFactory::Instance()->Init( 0, interface.c_str() );

    unitree::robot::ChannelSubscriber< sensor_msgs::msg::dds_::PointCloud2_> subscriber( topic_unitree );
    subscriber.InitChannel( Handler );

    ros::Rate rate(1);

    ROS_INFO( "Start relay lidar from %s -> %s on frame %s", topic_unitree.c_str(), publisher.getTopic().c_str(), output_frame.c_str() );
    while ( ros::ok() )
    {
        rate.sleep();
        ros::spinOnce();
    }
}