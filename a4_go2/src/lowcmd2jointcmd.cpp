#include "ros/ros.h"
#include "ros/console.h"

#include "unitree_sdk2/LowCmd.h"

#include "std_msgs/Float64.h"

#include "a4/ros/node_handle.hpp"


namespace a4_go2
{

namespace node
{

class PrivateLowCmd2JointCmd
{
    public:
        ros::Publisher publisher;
        std::string key;
        uint8_t mode;
};

namespace LowCmdMode
{
    enum CmdModeT
    {
        POSITION = 0,
        VELOCITY
    };
}

class LowCmd2JointCmd
{

public:
    LowCmd2JointCmd()
    {
        ;
    }

    int init()
    {
        ros::NodeHandle nh;

        std::vector< int > list_mode = a4_ros::param_v< int >( "~cmd/mode", std::vector<int>{}, true );
        std::vector< std::string > list_interface = a4_ros::param_v< std::string >( "~cmd/topic", std::vector< std::string >{}, true );

        if( list_mode.size() != list_interface.size() )
        {
            ROS_FATAL( "Size of mode %lu not match size of interface %lu", list_mode.size(), list_interface.size() );
            return 1;
        }

        this->cmd_handles.resize( list_mode.size() );

        for( unsigned int idx = 0 ; idx < this->cmd_handles.size(); idx++ )
        {
            this->cmd_handles[idx].mode = list_mode[idx];
            this->cmd_handles[idx].key = list_interface[idx];
            this->cmd_handles[idx].publisher = nh.advertise<std_msgs::Float64>(
                a4_ros::param_interface(
                    this->cmd_handles[idx].key,
                    this->cmd_handles[idx].key,
                    true
                ),
                1
            );
        }

        this->subscriber = nh.subscribe< unitree_sdk2::LowCmd >(
            a4_ros::param_interface( "~input", "/rt/lowcmd", true ), 
            1,
            &a4_go2::node::LowCmd2JointCmd::callback,
            this
        );

        ROS_INFO( "Finish init" );

        return 0;
    }

    int main()
    {
        ros::Rate rate(1);
        std_msgs::Float64 msg_zero;
        msg_zero.data = 0.0;
        ROS_INFO( "Start Loop run");
        while( ros::ok() )
        {
            rate.sleep();
            ros::spinOnce();

            if( (ros::Time::now() - this->last_stamp).toSec() > 1.0 )
            {
                ROS_INFO( "COMMAND DISAPPEAR SEND ZERO");
                for( auto iter = this->cmd_handles.begin(); iter != this->cmd_handles.end(); iter++ )
                {
                    iter->publisher.publish( msg_zero );
                }
                last_stamp = ros::Time::now();
            }
        }

        return 0;
    }

    ros::Subscriber subscriber;
    ros::Time last_stamp;
    void callback( const unitree_sdk2::LowCmdConstPtr& ptr_msg )
    {
        this->last_stamp = ros::Time::now();
        std_msgs::Float64 msg_output;
        std::ostringstream pre_message;
        pre_message.precision(2);
        for( unsigned int idx = 0 ; idx < this->cmd_handles.size(); idx++ )
        {
            switch (this->cmd_handles[idx].mode)
            {
            case LowCmdMode::POSITION:
                msg_output.data = ptr_msg->motor_cmd[idx].q;
                break;
            case LowCmdMode::VELOCITY:
                msg_output.data = ptr_msg->motor_cmd[idx].dq;
                break;
            default:
                msg_output.data = 0;
                break;
            }
            pre_message << " " << msg_output.data; 
            this->cmd_handles[idx].publisher.publish( msg_output );
        }
        ROS_INFO(
            "Output : %s", pre_message.str().c_str()
        );
    }

    std::vector< PrivateLowCmd2JointCmd > cmd_handles;

}; // Declate class LowCmd2JointCmd

} // namespace node

} // namespace a4_go2

int main( int argc, char** argv )
{

    ros::init( argc, argv, "lowcmd2jointcmd_converter");
    a4_go2::node::LowCmd2JointCmd node;
    node.init();
    node.main();

    return 0;
}