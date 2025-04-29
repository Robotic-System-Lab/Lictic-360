#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "image_converter.h"
#include <jetson-utils/videoSource.h>

class VideoSourceNode : public rclcpp::Node
{
public:
    VideoSourceNode()
    : Node("video_source")
    {
        // Declare parameters with defaults
        this->declare_parameter<std::string>("resource", "");
        this->declare_parameter<std::string>("codec", "");
        this->declare_parameter<int>("width", video_options.width);
        this->declare_parameter<int>("height", video_options.height);
        this->declare_parameter<double>("framerate", video_options.frameRate);
        this->declare_parameter<bool>("loop", video_options.loop);
        this->declare_parameter<std::string>("flip", "");
        this->declare_parameter<int>("latency", video_options.latency);

        // Retrieve parameter values
        std::string resource_str = this->get_parameter("resource").as_string();
        std::string codec_str = this->get_parameter("codec").as_string();
        int video_width = this->get_parameter("width").as_int();
        int video_height = this->get_parameter("height").as_int();
        video_options.frameRate = this->get_parameter("framerate").as_double();
        video_options.loop = this->get_parameter("loop").as_bool();
        std::string flip_str = this->get_parameter("flip").as_string();
        int latency = this->get_parameter("latency").as_int();

        if( resource_str.empty() )
        {
            RCLCPP_ERROR(this->get_logger(),
                         "resource parameter wasn't set - please set the node's resource "
                         "parameter to the input device/filename/URL");
            rclcpp::shutdown();
            return;
        }

        if( !codec_str.empty() )
            video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

        if( !flip_str.empty() )
            video_options.flipMethod = videoOptions::FlipMethodFromStr(flip_str.c_str());

        video_options.width  = video_width;
        video_options.height = video_height;
        video_options.latency = latency;

        RCLCPP_INFO(this->get_logger(), "opening video source: %s", resource_str.c_str());

        // Open video source
        stream = videoSource::Create(resource_str.c_str(), video_options);
        if( !stream )
        {
            RCLCPP_ERROR(this->get_logger(), "failed to open video source");
            rclcpp::shutdown();
            return;
        }

        // Create image converter instance
        image_cvt = new imageConverter();
        if( !image_cvt )
        {
            RCLCPP_ERROR(this->get_logger(), "failed to create imageConverter");
            rclcpp::shutdown();
            return;
        }

        // Create publisher for sensor_msgs::msg::Image messages
        image_pub = this->create_publisher<sensor_msgs::msg::Image>("raw", 2);

        // Start video stream
        if( !stream->Open() )
        {
            RCLCPP_ERROR(this->get_logger(), "failed to start streaming video source");
            rclcpp::shutdown();
            return;
        }
    }

    ~VideoSourceNode()
    {
        if (stream)
            delete stream;
        if (image_cvt)
            delete image_cvt;
    }

    bool acquireFrame()
    {
        imageConverter::PixelType* nextFrame = nullptr;

        // Get the latest frame with a timeout
        if( !stream->Capture(&nextFrame, 1000) )
        {
            RCLCPP_ERROR(this->get_logger(), "failed to capture next frame");
            return false;
        }

        // Assure correct image size
        if( !image_cvt->Resize(stream->GetWidth(), stream->GetHeight(), imageConverter::ROSOutputFormat) )
        {
            RCLCPP_ERROR(this->get_logger(), "failed to resize camera image converter");
            return false;
        }

        // Populate the message
        sensor_msgs::msg::Image msg;
        if( !image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame) )
        {
            RCLCPP_ERROR(this->get_logger(), "failed to convert video stream frame to sensor_msgs::Image");
            return false;
        }

        // Set the timestamp in header field
        msg.header.stamp = this->now();
        image_pub->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "published %ux%u video frame", stream->GetWidth(), stream->GetHeight());
        
        return true;
    }

    void run()
    {
        rclcpp::Rate loop_rate(30);
        while( rclcpp::ok() )
        {
            if( !acquireFrame() )
            {
                if( !stream->IsStreaming() )
                {
                    RCLCPP_INFO(this->get_logger(), "stream is closed or reached EOS, exiting node...");
                    break;
                }
            }
            rclcpp::spin_some(shared_from_this());
            loop_rate.sleep();
        }
    }

private:
    videoSource* stream = nullptr;
    imageConverter* image_cvt = nullptr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    videoOptions video_options;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Use std::make_shared to properly support shared_from_this in our node.
    auto node = std::make_shared<VideoSourceNode>();

    // Only run if the node was successfully initialized
    if (rclcpp::ok())
    {
        node->run();
    }

    rclcpp::shutdown();
    return 0;
}