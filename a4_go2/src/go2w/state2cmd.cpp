#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include "ros/ros.h"
#include "unitree_sdk2/LowCmd.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::b2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

class Custom
{
public:
    explicit Custom(){}
    ~Custom(){}

    void Init();
    void Start();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
    int queryMotionStatus();
    std::string queryServiceName(std::string form,std::string name);
 
private:
    float Kp = 70.0;
    float Kd = 5.0;
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01

    MotionSwitcherClient msc;

    unitree_sdk2::LowCmd low_cmd_msg;
    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    ros::Publisher lowcmd_ros_publisher;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

    float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

    float _startPos[12];
    float _duration_1 = 500;   
    float _duration_2 = 500; 
    float _duration_3 = 2000;   
    float _duration_4 = 900;   
    float _percent_1 = 0;    
    float _percent_2 = 0;    
    float _percent_3 = 0;    
    float _percent_4 = 0;    

    bool firstRun = true;
    bool done = false;
};

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Custom::Init()
{
    InitLowCmd();

    /*create publisher*/
    // lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    // lowcmd_publisher->InitChannel();

    ros::NodeHandle nh;
    this->lowcmd_ros_publisher = nh.advertise< unitree_sdk2::LowCmd >( TOPIC_LOWCMD, 1 );

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);

    /*init MotionSwitcherClient*/
    msc.SetTimeout(10.0f); 
    msc.Init();

    /*Shut down motion control-related service*/
    // while(queryMotionStatus())
    // {
    //     std::cout << "Try to deactivate the motion control-related service." << std::endl;
    //     int32_t ret = msc.ReleaseMode(); 
    //     if (ret == 0) {
    //         std::cout << "ReleaseMode succeeded." << std::endl;
    //     } else {
    //         std::cout << "ReleaseMode failed. Error code: " << ret << std::endl;
    //     }
    //     sleep(5);
    // }
}

void Custom::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    this->low_cmd_msg.head[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    this->low_cmd_msg.head[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    this->low_cmd_msg.level_flag = 0xFF;
    low_cmd.gpio() = 0;
    this->low_cmd_msg.gpio = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        this->low_cmd_msg.motor_cmd[i].mode = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        this->low_cmd_msg.motor_cmd[i].q = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        this->low_cmd_msg.motor_cmd[i].kp = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        this->low_cmd_msg.motor_cmd[i].dq = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        this->low_cmd_msg.motor_cmd[i].kd = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
        this->low_cmd_msg.motor_cmd[i].tau = (0);
    }
}

int Custom::queryMotionStatus()
{
    std::string robotForm,motionName;
    int motionStatus;
    int32_t ret = msc.CheckMode(robotForm,motionName);
    if (ret == 0) {
        std::cout << "CheckMode succeeded." << std::endl;
    } else {
        std::cout << "CheckMode failed. Error code: " << ret << std::endl;
    }
    if(motionName.empty())
    {
        std::cout << "The motion control-related service is deactivated." << std::endl;
        motionStatus = 0;
    }
    else
    {
        std::string serviceName = queryServiceName(robotForm,motionName);
        std::cout << "Service: "<< serviceName<< " is activate" << std::endl;
        motionStatus = 1;
    }
    return motionStatus;
}

std::string Custom::queryServiceName(std::string form,std::string name)
{
    if(form == "0")
    {
        if(name == "normal" ) return "sport_mode"; 
        if(name == "ai" ) return "ai_sport"; 
        if(name == "advanced" ) return "advanced_sport"; 
    }
    else
    {
        if(name == "ai-w" ) return "wheeled_sport(go2W)"; 
        if(name == "normal-w" ) return "wheeled_sport(b2W)";
    }
    return "";
}

void Custom::Start()
{
    /*loop publishing thread*/
    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, int(dt * 1000000), &Custom::LowCmdWrite, this);
}

void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;

    std::stringstream output_cmd;
    output_cmd.precision(2);

    for( unsigned int idx = 0 ; idx < 12 ; idx++ )
    {
        switch (idx)
        {
        case 0:
        case 3:
        case 6:
        case 9:
            this->low_cmd_msg.motor_cmd[idx].q = -1 * low_state.motor_state()[idx].q();
            break;
        default:
            this->low_cmd_msg.motor_cmd[idx].q = low_state.motor_state()[idx].q();
            break;
        }

        output_cmd << " " << this->low_cmd_msg.motor_cmd[idx].q;
    }

    for( unsigned int idx = 12 ; idx < 16 ; idx++ )
    {
        this->low_cmd_msg.motor_cmd[idx].dq = 0.0;
        output_cmd << " " << this->low_cmd_msg.motor_cmd[idx].dq;
    }

    ROS_INFO( 
        "Output : %s", output_cmd.str().c_str()
    );
    this->lowcmd_ros_publisher.publish( this->low_cmd_msg );
}

void Custom::LowCmdWrite()
{
    ;
}

int main(int argc, char** argv)
{
    // if (argc < 2)
    // {
    //     std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    //     exit(-1); 
    // }

    // std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
    //         << "Press Enter to continue..." << std::endl;
    // std::cin.ignore();

    ros::init( argc, argv, "go2_stand_forward" );
    ChannelFactory::Instance()->Init(0, "lo");

    Custom custom;
    custom.Init();
    custom.Start();
  
    while (ros::ok())
    {
        sleep(10);
    }

    return 0;
}