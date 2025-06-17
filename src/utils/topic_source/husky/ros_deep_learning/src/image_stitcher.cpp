#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageStitcher
{
public:
    ImageStitcher()
    {
        // Initialize ROS subscribers and publisher
        image_transport::ImageTransport it(nh_);
        image_sub1_ = it.subscribe("/camera1/image_raw", 1, &ImageStitcher::imageCallback1, this);
        image_sub2_ = it.subscribe("/camera2/image_raw", 1, &ImageStitcher::imageCallback2, this);
        image_sub3_ = it.subscribe("/camera3/image_raw", 1, &ImageStitcher::imageCallback3, this);
        image_pub_ = it.advertise("/combined_image", 1);

        // Initialize image pointers
        image1_ = nullptr;
        image2_ = nullptr;
        image3_ = nullptr;
    }

    // Callback untuk menerima gambar dari kamera 1
    void imageCallback1(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            image1_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            stitchImages();
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR("CV Bridge exception: %s", e.what());
        }
    }

    // Callback untuk menerima gambar dari kamera 2
    void imageCallback2(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            image2_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            stitchImages();
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR("CV Bridge exception: %s", e.what());
        }
    }

    // Callback untuk menerima gambar dari kamera 3
    void imageCallback3(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            image3_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
            stitchImages();
        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR("CV Bridge exception: %s", e.what());
        }
    }

    // Fungsi untuk menggabungkan gambar
    void stitchImages()
    {
        // Periksa jika semua gambar sudah diterima
        if (image1_ != nullptr && image2_ != nullptr && image3_ != nullptr)
        {
            // Gabungkan gambar secara horizontal
            cv::Mat stitched_image;
            cv::hconcat(image1_, image2_, stitched_image);
            cv::hconcat(stitched_image, image3_, stitched_image);

            // Konversi gambar OpenCV ke ROS message
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", stitched_image).toImageMsg();

            // Publikasikan gambar gabungan
            image_pub_.publish(msg);
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub1_;
    image_transport::Subscriber image_sub2_;
    image_transport::Subscriber image_sub3_;
    image_transport::Publisher image_pub_;

    cv::Mat image1_;
    cv::Mat image2_;
    cv::Mat image3_;
};

int main(int argc, char** argv)
{
    // Inisialisasi ROS
    ros::init(argc, argv, "image_stitcher");
    ImageStitcher stitcher;
    ros::spin();
    return 0;
}
