// Reference https://support.unitree.com/home/en/developer/sports_services

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "ros/ros.h"
#include "ros/console.h"

#include "nav_msgs/Odometry.h"

#include <tf/transform_broadcaster.h>

#include "a4/ros/node_handle.hpp"
#include "a4/ros/transform.hpp"

class UnitreeSportModeState
{

public:
    UnitreeSportModeState(){}

    void init( int argc, char** argv )
    {

        ros::NodeHandle nh;

        std::string interface = a4_ros::param< std::string >( "~interface", "lo", true );

        std::string topic_sportmodestate = a4_ros::param< std::string >( "~topic/unitree", "rt/sportmodestate", true );

        this->odom_publisher = nh.advertise< nav_msgs::Odometry >(
            a4_ros::param_interface( "~topic/output", topic_sportmodestate, true ),
            10
        );
        this->msg_odom.header.frame_id = a4_ros::param< std::string >( "~frame/parent", "odom_unitree_link", true );
        this->msg_odom.child_frame_id = a4_ros::param< std::string >( "~frame/child", "child_link", true );

        unitree::robot::ChannelFactory::Instance()->Init( 0, interface.c_str() );

        this->sportmodestate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(topic_sportmodestate));

        this->sportmodestate_subscriber->InitChannel(
            std::bind(
                &UnitreeSportModeState::SportModeStateMessageHandler, 
                this,
                std::placeholders::_1
            ),
            1
        );
    }

    int main()
    {

        ros::Rate rate(1);

        while( ros::ok() )
        {
            rate.sleep();
            ros::spinOnce();
        }

        return 0;
    }

protected:
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sportmodestate_subscriber;
    unitree_go::msg::dds_::SportModeState_ state;

    tf::TransformBroadcaster tf_br;
    ros::Publisher odom_publisher;

    nav_msgs::Odometry msg_odom;


private:
    void SportModeStateMessageHandler( const void* message )
    {
        this->msg_odom.header.stamp = ros::Time::now();

        std::array<float, 4> buffer4;
        std::array<float, 3> buffer3;

        this->state = *(unitree_go::msg::dds_::SportModeState_*)message;
        const unitree_go::msg::dds_::IMUState_ &imu = state.imu_state();

        buffer3 = this->state.position();
        this->msg_odom.pose.pose.position.x = buffer3[0];
        this->msg_odom.pose.pose.position.y = buffer3[1];
        this->msg_odom.pose.pose.position.z = buffer3[2];

        buffer3 = this->state.velocity();
        this->msg_odom.twist.twist.linear.x = buffer3[0];
        this->msg_odom.twist.twist.linear.y = buffer3[1];
        this->msg_odom.twist.twist.linear.z = buffer3[2];
        
        buffer4 = imu.quaternion();
        this->msg_odom.pose.pose.orientation.w = buffer4[0];
        this->msg_odom.pose.pose.orientation.x = buffer4[1];
        this->msg_odom.pose.pose.orientation.y = buffer4[2];
        this->msg_odom.pose.pose.orientation.z = buffer4[3];

        buffer3 = imu.gyroscope();
        this->msg_odom.twist.twist.angular.x = buffer3[0];
        this->msg_odom.twist.twist.angular.y = buffer3[1];
        this->msg_odom.twist.twist.angular.z = buffer3[2];

        this->odom_publisher.publish( this->msg_odom );

        tf::Transform tf_transform = tf::Transform( 
            tf::Quaternion(
                this->msg_odom.pose.pose.orientation.x,
                this->msg_odom.pose.pose.orientation.y,
                this->msg_odom.pose.pose.orientation.z,
                this->msg_odom.pose.pose.orientation.w
            ), 
            tf::Vector3( 
                this->msg_odom.pose.pose.position.x,
                this->msg_odom.pose.pose.position.y,
                this->msg_odom.pose.pose.position.z
            )
        );
        ROS_INFO_THROTTLE( 1, "Transform inverse of xyz %.2f %.2f %.2f xyzw %.2f %.2f %.2f %.2f by frame %s -> %s", 
            this->msg_odom.pose.pose.position.x,
            this->msg_odom.pose.pose.position.y,
            this->msg_odom.pose.pose.position.z,
            this->msg_odom.pose.pose.orientation.x,
            this->msg_odom.pose.pose.orientation.y,
            this->msg_odom.pose.pose.orientation.z,
            this->msg_odom.pose.pose.orientation.w,
            this->msg_odom.child_frame_id.c_str(),
            this->msg_odom.header.frame_id.c_str()
        );
        this->tf_br.sendTransform( tf::StampedTransform(
            tf_transform.inverse(),
            this->msg_odom.header.stamp,
            this->msg_odom.child_frame_id,
            this->msg_odom.header.frame_id
        ) );
    }

}; // Constrict UnitreeSportModeState

int main( int argc, char** argv )
{
    ros::init( argc, argv, "unitree_sportmodestate" );
}