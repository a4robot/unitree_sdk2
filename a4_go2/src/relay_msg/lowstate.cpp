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

private:
    void LowStateMessageHandler(const void *message)
    {
        std::array<float, 4> buffer4;
        std::array<float, 3> buffer3;

        this->state = *(unitree_go::msg::dds_::LowState_ *)message;
        
        const unitree_go::msg::dds_::IMUState_ &imu = state.imu_state();

        sensor_msgs::Imu msg_imu;
        buffer4 = imu.quaternion();
        msg_imu.orientation.x = buffer4[0];
        msg_imu.orientation.y = buffer4[1];
        msg_imu.orientation.z = buffer4[2];
        msg_imu.orientation.w = buffer4[3];

        buffer3 = imu.gyroscope();
        msg_imu.angular_velocity.x = buffer3[0];
        msg_imu.angular_velocity.y = buffer3[1];
        msg_imu.angular_velocity.z = buffer3[2];

        buffer3 = imu.accelerometer();
        msg_imu.linear_acceleration.x = buffer3[0];
        msg_imu.linear_acceleration.y = buffer3[1];
        msg_imu.linear_acceleration.z = buffer3[2];


        const std::array<unitree_go::msg::dds_::MotorState_, 20> &motor = state.motor_state();
        unitree_sdk2::MotorStates motor_states;
        motor_states.states.resize(20);
        for( int idx = 0 ; idx < 20 ; idx++ )
        {
            motor_states.states[idx].mode = motor[idx].mode();
            motor_states.states[idx].q = motor[idx].q();
            motor_states.states[idx].dq = motor[idx].dq();
            motor_states.states[idx].ddq = motor[idx].ddq();
            motor_states.states[idx].tau_est = motor[idx].tau_est();
            motor_states.states[idx].q_raw = motor[idx].q_raw();
            motor_states.states[idx].dq_raw = motor[idx].dq_raw();
            motor_states.states[idx].ddq_raw = motor[idx].ddq_raw();
            motor_states.states[idx].temperature = motor[idx].temperature();
            motor_states.states[idx].lost = motor[idx].lost();
            std::copy( 
                motor[idx].reserve().cbegin(), 
                motor[idx].reserve().cend(), 
                motor_states.states[idx].reserve.begin() 
            );
        }

        const unitree_go::msg::dds_::BmsState_ &bms = state.bms_state();
        unitree_sdk2::BMSState bms_state;
        bms_state.version_high = bms.version_high(); 
        bms_state.version_low = bms.version_low(); 
        bms_state.status = bms.status(); 
        bms_state.soc = bms.soc(); 
        bms_state.current = bms.current(); 
        bms_state.cycle = bms.cycle(); 
        std::copy( 
            bms.bq_ntc().cbegin(),
            bms.bq_ntc().cend(),
            bms_state.bq_ntc.begin()
        );
        std::copy( 
            bms.mcu_ntc().cbegin(),
            bms.mcu_ntc().cend(),
            bms_state.mcu_ntc.begin()
        );
        std::copy( 
            bms.cell_vol().cbegin(),
            bms.cell_vol().cend(),
            bms_state.cell_vol.begin()
        );
    }

};

int main( int argc, char** argv )
{
    UnitreeLowState node;

    node.init( argc, argv );

    return node.main();
}