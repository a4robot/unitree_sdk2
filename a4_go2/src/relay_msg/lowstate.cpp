#include "unitree/idl/go2/LowState_.hpp"

#include "unitree/robot/channel/channel_subscriber.hpp"

#include "ros/ros.h"
#include "ros/console.h"

#include "sensor_msgs/Imu.h"

#include "a4/ros/node_handle.hpp"

#include "unitree_sdk2/MotorStates.h"
#include "unitree_sdk2/BMSState.h"

class UnitreeLowState
{

public:

    UnitreeLowState(){}

    void init( int argc, char** argv )
    {
        ros::init( argc, argv, "unitree_lowstate");

        ros::NodeHandle nh;

        std::string interface = a4_ros::param< std::string >( "~interface", "lo", true );

        // Topic choice "rt/utilidar/cloud" and "ut/utilidar/cloud_desckewed"
        std::string topic_lowstate = a4_ros::param< std::string >( "~topic/unitree", "rt/lowstate", true );

        this->imu_active = a4_ros::param< int >( "~active/imu", 1, true );
        this->motor_active = a4_ros::param< int >( "~active/motor", 0, true );
        this->bms_active = a4_ros::param< int >( "~active/bms", 0, true );

        if( this->imu_active != 0 )
        {
            this->imu_publisher = nh.advertise< sensor_msgs::Imu >(
                a4_ros::param_interface( "~topic/imu", "/unitree/imu", true ),
                10
            );
        }
        if( this->motor_active != 0 )
        {
            this->motor_publisher = nh.advertise< unitree_sdk2::MotorStates >(
                a4_ros::param_interface( "~topic/motor", "/unitree/motor", true ),
                10
            );
        }
        if( this->bms_active != 0 )
        {
            this->bms_publisher = nh.advertise< unitree_sdk2::BMSState >(
                a4_ros::param_interface( "~topic/bms", "/unitree/bms", true ),
                10
            );
        }

        unitree::robot::ChannelFactory::Instance()->Init( 0, interface.c_str() );

        this->lowstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(topic_lowstate));

        this->lowstate_subscriber->InitChannel(
            std::bind(
                &UnitreeLowState::LowStateMessageHandler, 
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
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    unitree_go::msg::dds_::LowState_ state;

    int imu_active;
    ros::Publisher imu_publisher;
    int motor_active;
    ros::Publisher motor_publisher;
    int bms_active;
    ros::Publisher bms_publisher;

private:
    void LowStateMessageHandler(const void *message)
    {
        std::array<float, 4> buffer4;
        std::array<float, 3> buffer3;

        this->state = *(unitree_go::msg::dds_::LowState_ *)message;
        
        const unitree_go::msg::dds_::IMUState_ &imu = state.imu_state();

        sensor_msgs::Imu msg_imu;
        msg_imu.header.stamp = ros::Time::now();
        if( this->imu_active != 0 )
        {
            buffer4 = imu.quaternion();
            msg_imu.orientation.w = buffer4[0];
            msg_imu.orientation.x = buffer4[1];
            msg_imu.orientation.y = buffer4[2];
            msg_imu.orientation.z = buffer4[3];

            buffer3 = imu.gyroscope();
            msg_imu.angular_velocity.x = buffer3[0];
            msg_imu.angular_velocity.y = buffer3[1];
            msg_imu.angular_velocity.z = buffer3[2];

            buffer3 = imu.accelerometer();
            msg_imu.linear_acceleration.x = buffer3[0];
            msg_imu.linear_acceleration.y = buffer3[1];
            msg_imu.linear_acceleration.z = buffer3[2];
            this->imu_publisher.publish( msg_imu );
        }


        const std::array<unitree_go::msg::dds_::MotorState_, 20> &motor = state.motor_state();
        unitree_sdk2::MotorStates msg_motors;
        if( this->motor_active != 0)
        {
            msg_motors.states.resize(20);
            for( int idx = 0 ; idx < 20 ; idx++ )
            {
                msg_motors.states[idx].mode = motor[idx].mode();
                msg_motors.states[idx].q = motor[idx].q();
                msg_motors.states[idx].dq = motor[idx].dq();
                msg_motors.states[idx].ddq = motor[idx].ddq();
                msg_motors.states[idx].tau_est = motor[idx].tau_est();
                msg_motors.states[idx].q_raw = motor[idx].q_raw();
                msg_motors.states[idx].dq_raw = motor[idx].dq_raw();
                msg_motors.states[idx].ddq_raw = motor[idx].ddq_raw();
                msg_motors.states[idx].temperature = motor[idx].temperature();
                msg_motors.states[idx].lost = motor[idx].lost();
                std::copy( 
                    motor[idx].reserve().cbegin(), 
                    motor[idx].reserve().cend(), 
                    msg_motors.states[idx].reserve.begin() 
                );
            }
            this->motor_publisher.publish( msg_motors );
        }

        const unitree_go::msg::dds_::BmsState_ &bms = state.bms_state();
        unitree_sdk2::BMSState msg_bms;
        if( this->bms_active != 0 )
        {
            msg_bms.version_high = bms.version_high(); 
            msg_bms.version_low = bms.version_low(); 
            msg_bms.status = bms.status(); 
            msg_bms.soc = bms.soc(); 
            msg_bms.current = bms.current(); 
            msg_bms.cycle = bms.cycle(); 
            std::copy( 
                bms.bq_ntc().cbegin(),
                bms.bq_ntc().cend(),
                msg_bms.bq_ntc.begin()
            );
            std::copy( 
                bms.mcu_ntc().cbegin(),
                bms.mcu_ntc().cend(),
                msg_bms.mcu_ntc.begin()
            );
            std::copy( 
                bms.cell_vol().cbegin(),
                bms.cell_vol().cend(),
                msg_bms.cell_vol.begin()
            );
            this->bms_publisher.publish( msg_bms );
        }
    }

};

int main( int argc, char** argv )
{
    UnitreeLowState node;

    node.init( argc, argv );

    return node.main();
}