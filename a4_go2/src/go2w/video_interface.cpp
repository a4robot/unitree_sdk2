#include <unitree/robot/go2/video/video_client.hpp>

#include <iostream>
#include <fstream>
#include <ctime>

#include "a4/ros/node_handle.hpp"

#include "sensor_msgs/Image.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

std::string nowString()
{
    using namespace std::chrono;

    auto now = system_clock::now();
    std::time_t t = system_clock::to_time_t(now);
    std::tm tm;
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d-%H_%M_%S");
    return oss.str();
}

namespace a4_go2
{

namespace node
{
 
class VideoInterface
{

public:
    VideoInterface()
    {
        ;
    }

    ~VideoInterface()
    {
        ;
    }

    ros::Publisher image_publisher;

    void init()
    {
        ros::NodeHandle nh;

        std::string interface = a4_ros::param< std::string >( "~interface", "lo", true );

        unitree::robot::ChannelFactory::Instance()->Init( 0, interface.c_str() );

        this->image_publisher = nh.advertise< sensor_msgs::Image >(
            a4_ros::param_interface( "~topic/output", "~output", true ),
            1
        );
    }

    int main()
    {

        unitree::robot::go2::VideoClient video_client;

        video_client.SetTimeout(1.0f);
        video_client.Init();

        std::vector< uint8_t > image_sample;
        int ret;

        int frequency = a4_ros::param< int >( "~rate", 1, true );
        ros::Rate rate( frequency );
        
        ROS_INFO( "Start image run rate %d to topic %s", frequency, this->image_publisher.getTopic().c_str() );
        int count = 0;
        while( ros::ok() )
        {
            rate.sleep();

            ret = video_client.GetImageSample(image_sample);

            if (ret == 0) 
            {
                time_t rawtime;
                struct tm *timeinfo;
                char buffer[80];

                time(&rawtime);
                timeinfo = localtime(&rawtime);

                count+=1;
                snprintf(buffer, sizeof(buffer), "temp_front_robot_%d.jpg", count);
                count = count % 10;

                std::string image_name(buffer);

                std::ofstream image_file(image_name, std::ios::binary);
                if (image_file.is_open()) 
                {
                    image_file.write(reinterpret_cast<const char*>(image_sample.data()), image_sample.size());
                    image_file.close();
                    std::cout << "Image saved successfully as " << image_name << std::endl;
                } 
                else 
                {
                    std::cerr << "Error: Failed to save image." << std::endl;
                }

                cv::Mat buf(
                        1, 
                        static_cast<int>(image_sample.size()), CV_8UC1,
                        const_cast<uint8_t*>(image_sample.data())
                );

                cv::Mat img = cv::imdecode(buf, cv::IMREAD_COLOR);  // BGR image

                if( img.empty() )
                {
                    ROS_ERROR_THROTTLE( 1.0, "Failed to decode image buffer");
                }
                else
                {
                    std_msgs::Header header;
                    header.stamp = ros::Time::now();
                    header.frame_id = "camera_optical_frame";  // change to your frame

                    sensor_msgs::ImagePtr msg =
                        cv_bridge::CvImage(header, "bgr8", img).toImageMsg();

                    this->image_publisher.publish(msg);
                    ROS_INFO_THROTTLE( 1.0, "Publish image on stam %.2f", header.stamp.toSec() );
                }
            }
            else
            {
                ROS_FATAL_THROTTLE( 1.0, "Failure to get image sample");
            }
        }

        return 0;
    }


}; 

}

}

int main( int argc, char** argv )
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); 
    }

    std::cout << "WARNING: Make sure the robot is hung up or lying on the ground." << std::endl
            << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::init( argc, argv, "go2_video_interface" );
    
    a4_go2::node::VideoInterface node;
    node.init();
    node.main();
}