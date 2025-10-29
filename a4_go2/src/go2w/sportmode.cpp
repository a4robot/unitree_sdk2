// Reference https://support.unitree.com/home/en/developer/sports_services

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/robot/go2/sport/sport_client.hpp>

#include "ros/ros.h"
#include "ros/console.h"

#include "nav_msgs/Odometry.h"

#include <tf/transform_broadcaster.h>

#include "a4/ros/node_handle.hpp"
#include "a4/ros/transform.hpp"

#include "geometry_msgs/Twist.h"

namespace SportClientState
{
    enum SportClientState_t
    {
        INVALID = -2,
        START_PROGRAM,
        INIT,
        DAMP,
        STAND_UP,
        STAND_DOWN,
        MOVE,
        STOP_MOVE
    };

    std::string to_string( const SportClientState_t data )
    {
        std::string result = "UNKNOW";

        switch (data)
        {
        case SportClientState::INVALID:
            result = "INVALID";
            break;
        case SportClientState::START_PROGRAM:
            result = "START_PROGRAM";
            break;
        case SportClientState::INIT:
            result = "INIT";
            break;
        case SportClientState::DAMP:
            result = "DAMP";
            break;
        case SportClientState::STAND_UP:
            result = "STAND_UP";
            break;
        case SportClientState::STAND_DOWN:
            result = "STAND_DOWN";
            break;
        case SportClientState::MOVE:
            result = "MOVE";
            break;
        case SportClientState::STOP_MOVE:
            result = "STOP_MOVE";
            break;
        default:
            break;
        }

        return result;
    };
}

class UnitreeGO2WSportMode
{

public:
    UnitreeGO2WSportMode()
    {
        this->mode_client = SportClientState::START_PROGRAM;
    }

    void init( int argc, char** argv )
    {

        ros::NodeHandle nh;

        std::string interface = a4_ros::param< std::string >( "~interface", "lo", true );

        std::string topic_sportmodestate = a4_ros::param< std::string >( "~topic/unitree", "rt/sportmodestate", true );

        this->odom_publisher = nh.advertise< nav_msgs::Odometry >(
            a4_ros::param_interface( "~topic/output", topic_sportmodestate, true ),
            10
        );
        this->cmd_feedback_publisher = nh.advertise< geometry_msgs::Twist >(
            a4_ros::param_interface( "~topic/cmd_feedback", "/feedback/cmd/vel", true ),
            1
        );
        this->cmd_subscriber = nh.subscribe< geometry_msgs::Twist >(
            a4_ros::param_interface( "~topic/cmd", "/cmd/vel", true ),
            1,
            &UnitreeGO2WSportMode::cmd_vel_callback,
            this
        );
        this->msg_odom.header.frame_id = a4_ros::param< std::string >( "~frame/parent", "odom_unitree_link", true );
        this->msg_odom.child_frame_id = a4_ros::param< std::string >( "~frame/child", "child_link", true );

        unitree::robot::ChannelFactory::Instance()->Init( 0, interface.c_str() );

        this->sportmodestate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(topic_sportmodestate));

        this->sportmodestate_subscriber->InitChannel(
            std::bind(
                &UnitreeGO2WSportMode::SportModeStateMessageHandler, 
                this,
                std::placeholders::_1
            ),
            1
        );

        this->mode_delay = a4_ros::param< double >( "~sport/delay", 10.f, true );

        this->sport_client = new unitree::robot::go2::SportClient();
        this->sport_client->SetTimeout( a4_ros::param< float >( "~sport/timeout", 20.0f, true ) );
        this->sport_client->Init();
    }

    int main()
    {

        ros::Rate rate(1);

        this->mode_client_expect = SportClientState::INIT;

        while( ros::ok() )
        {
            rate.sleep();
            ros::spinOnce();
            ros::Time stamp = ros::Time::now();

            if( this->mode_client <= SportClientState::DAMP )
            {
                ;
            }
            else if( (stamp - this->cmd_stamp).toSec() > 30 )
            {
                this->mode_client_expect = SportClientState::STAND_DOWN;
            }
            else
            {

            }
            switch ( this->mode_client )
            {
            case SportClientState::START_PROGRAM:
                this->mode_client_expect = SportClientState::INIT;
                this->mode_stamp = ros::Time(0, 0);
                break;
            case SportClientState::INIT:
                this->mode_client_expect = SportClientState::DAMP;
                break;
            case SportClientState::DAMP:
                this->mode_client_expect = SportClientState::STAND_DOWN;
                break;
            case SportClientState::STAND_UP:
                break;
            case SportClientState::STAND_DOWN:
                if( this->mode_client_expect == SportClientState::MOVE )
                {
                    this->mode_client_expect = SportClientState::STAND_UP;
                }
                break;
            case SportClientState::MOVE:
                if( this->mode_client_expect == SportClientState::STAND_DOWN )
                {
                    this->mode_client_expect = SportClientState::STOP_MOVE;
                }
                break;
            case SportClientState::STOP_MOVE:
                break;
            default:
                break;
            }

            if( this->mode_client_expect != this->mode_client )
            {
                double dt = (stamp - this->mode_stamp).toSec();
                ROS_INFO_THROTTLE( 1, "Current client in mode %s expect %s time %.2f(%.2f) ago", 
                    SportClientState::to_string( this->mode_client ).c_str(),
                    SportClientState::to_string( this->mode_client_expect ).c_str(),
                    dt,
                    this->mode_delay
                );

                if( dt > this->mode_delay )
                {
                    int32_t state_code = 0; 
                    bool update_state = true;
                    if( this->mode_client_expect == SportClientState::INIT )
                    {
                        this->mode_client = this->mode_client_expect;
                    }
                    else if( this->mode_client_expect == SportClientState::DAMP )
                    {
                        state_code = this->sport_client->Damp();
                        if( state_code == 0 ) this->mode_client = this->mode_client_expect;
                        else
                        {
                            update_state = false;
                            ROS_FATAL( "Failure to command %s code %d", SportClientState::to_string( this->mode_client_expect ).c_str(), state_code );
                            this->mode_stamp = stamp - ros::Duration( 10 );
                        }
                    }
                    else if( this->mode_client_expect == SportClientState::MOVE )
                    {
                        this->mode_client = this->mode_client_expect;
                    }
                    else if( this->mode_client_expect == SportClientState::STAND_DOWN )
                    {
                        state_code = this->sport_client->StandDown();
                        if( state_code == 0 ) this->mode_client = this->mode_client_expect;
                        else
                        {
                            update_state = false;
                            ROS_FATAL( "Failure to command %s code %d", SportClientState::to_string( this->mode_client_expect ).c_str(), state_code );
                            this->mode_stamp = stamp - ros::Duration( 10 );
                        }
                    }
                    else if( this->mode_client_expect == SportClientState::STAND_DOWN )
                    {
                        state_code = this->sport_client->StandUp();
                        if( state_code == 0 ) this->mode_client = this->mode_client_expect;
                        else
                        {
                            update_state = false;
                            ROS_FATAL( "Failure to command %s code %d", SportClientState::to_string( this->mode_client_expect ).c_str(), state_code );
                            this->mode_stamp = stamp - ros::Duration( 10 );
                        }
                    }
                    else if( this->mode_client_expect == SportClientState::STOP_MOVE )
                    {
                        state_code = this->sport_client->StopMove();
                        if( state_code == 0 ) this->mode_client = this->mode_client_expect;
                        else
                        {
                            update_state = false;
                            ROS_FATAL( "Failure to command %s code %d", SportClientState::to_string( this->mode_client_expect ).c_str(), state_code );
                            this->mode_stamp = stamp - ros::Duration( 10 );
                        }
                    }
                    else
                    {
                        update_state = false;
                        ROS_FATAL( "Unknow how to response expect mode %s", SportClientState::to_string( this->mode_client_expect ).c_str() );
                    }

                    if( update_state )
                    {
                        ROS_WARN( "Update mode client to %s", SportClientState::to_string( this->mode_client ).c_str() );
                        this->mode_stamp = stamp;
                    }
                }
                else
                {

                }
            } // Mode not match
            else
            {
                ROS_INFO_THROTTLE( 10, "Current on mode %s", SportClientState::to_string( this->mode_client ).c_str() );
            }
        }

        return 0;
    }

protected:
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sportmodestate_subscriber;
    unitree_go::msg::dds_::SportModeState_ state;

    double mode_delay;
    ros::Time mode_stamp;
    SportClientState::SportClientState_t mode_client;
    SportClientState::SportClientState_t mode_client_expect;
    unitree::robot::go2::SportClient* sport_client;

    tf::TransformBroadcaster tf_br;
    ros::Publisher odom_publisher;

    ros::Time cmd_stamp;
    ros::Subscriber cmd_subscriber;
    ros::Publisher cmd_feedback_publisher;

    nav_msgs::Odometry msg_odom;


private:

    void cmd_vel_callback( const geometry_msgs::TwistConstPtr& ptr_msg )
    {
        this->cmd_stamp = ros::Time::now();
        geometry_msgs::Twist feedback_msg = *ptr_msg;

        if( this->mode_client == SportClientState::MOVE )
        {
            if( 
                std::fabs( feedback_msg.linear.x ) < 1e-4 && 
                std::fabs( feedback_msg.linear.y ) < 1e-4 &&
                std::fabs( feedback_msg.angular.z ) < 1e-4
            )
            {
                if( this->sport_client->StopMove() != 0 )
                {
                    this->sport_client->Move( 0.0, 0.0, 0.0 );
                }
                else
                {
                    ;
                }
            }
            else
            {
                this->sport_client->Move( 
                    feedback_msg.linear.x,
                    feedback_msg.linear.y,
                    feedback_msg.angular.z
                );
            }
        }
        else
        {
            this->mode_client_expect = SportClientState::MOVE;
            feedback_msg.linear.x = 0.0;
            feedback_msg.linear.y = 0.0;
            feedback_msg.angular.z = 0.0;
        }

        this->cmd_feedback_publisher.publish( feedback_msg );
    }

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

}; // Constrict UnitreeGO2WSportMode

int main( int argc, char** argv )
{
    ros::init( argc, argv, "unitree_sportmodestate" );
}