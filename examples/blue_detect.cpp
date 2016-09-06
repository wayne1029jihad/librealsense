//this code for blue detect ,camera : realsense

//Realsense Header 

#include <librealsense/rs.hpp>
#include "example.hpp"

//OpenCV header

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

//STL header

#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>

#include <thread>
#include <mutex>

// Color Detection Header

#include "color_detect.hpp"

//ROS header
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "ros_wrapper.hpp"
 
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480

using namespace std;
CvPoint mouseC; 
RosWrapper * g_ros_wrapper;
std::mutex g_pointcloud_mutex;

void onMouse(int event,int x,int y,int flags,void* param);

rs::float3 g_pointcloud[FRAME_WIDTH * FRAME_HEIGHT] = { 0 };

cv::Mat equalizeIntensity(const cv::Mat& inputImage)
{
    if(inputImage.channels() >= 3)
    {
        cv::Mat ycrcb;

        cvtColor(inputImage,ycrcb,CV_BGR2YCrCb);

        std::vector<cv::Mat> channels;
        split(ycrcb,channels);

        cv::equalizeHist(channels[0], channels[0]);

        cv::Mat result;
        cv::merge(channels,ycrcb);

        cv::cvtColor(ycrcb,result,CV_YCrCb2BGR);

        return result;
    }
    return cv::Mat();
}

static void run_opencv(rs::device & dev)
{
    // Determine depth value corresponding to one meter
    const uint16_t one_meter = static_cast<uint16_t>(1.0f / dev.get_depth_scale());

    uint32_t display_mode = 0;
    float ts_height = 0.1;
    cout<<"deth info"<<endl;
    while(true)
    {
        // This call waits until a new coherent set of frames is available on a device
        // Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        dev.wait_for_frames();
        
       // Retrieve depth data, which was previously configured as a 640 x 480 image of 16-bit depth values
       // const uint16_t * depth_frame = reinterpret_cast<const uint16_t *>(dev.get_frame_data(rs::stream::depth));
        const uint8_t * color_frame = reinterpret_cast<const uint8_t *>(dev.get_frame_data(rs::stream::color_aligned_to_depth));
        auto points = reinterpret_cast<const rs::float3 *>(dev.get_frame_data(rs::stream::points));
        
        g_pointcloud_mutex.lock();
        memcpy(g_pointcloud, points, sizeof(g_pointcloud));
        g_pointcloud_mutex.unlock();
        
        cv::Mat mColorImage(480, 640, CV_8UC3, (void*) color_frame);
        cv::cvtColor(mColorImage, mColorImage, cv::COLOR_RGB2BGR);
        mColorImage = equalizeIntensity(mColorImage);
        std::vector<cv::Rect> rects;
        color_detect::find_blue(mColorImage, rects, 
            800, // Minimum area threshold
            cv::Scalar(100, 150, 100), // Blue lower bound
            cv::Scalar(140, 255, 255) // Blue upper bound 
        );
        
        for(auto rect : rects) {
            cv::rectangle(mColorImage, rect.tl(), rect.br(), cv::Scalar(0, 255, 0), 2);
            cv::circle(mColorImage, rect.tl(),5,cv::Scalar(255,0,0),2);
        }
                 
        circle(mColorImage,mouseC,5,cv::Scalar(0,0,255), 3);   
        cv::imshow( "Color Image" , mColorImage);

        char key = cv::waitKey(1);
        switch(key) {
            case 27:
                return;
                break;
            default:
                break;
        }

    }
}
int main(int argc, char * argv[]) 
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");
    mouseC = cvPoint(-100,-100);
     g_ros_wrapper = new RosWrapper(argc, argv, "blue_detect", "chatter");
    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
//    try { dev.enable_stream(rs::stream::infrared2, rs::preset::best_quality); } catch(...) {}
    dev.start();
    cout<<"device work"<<endl;
    cv::namedWindow( "Color Image",  CV_WINDOW_NORMAL );    
    cvSetMouseCallback("Color Image",onMouse,NULL);
    run_opencv(dev);

    return EXIT_SUCCESS; 
}
/*
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}   
*/void onMouse(int event,int x,int y,int flag,void* param){
    static rs::float3 last_point = rs::float3{0, 0, 0};
 
    std_msgs::Float32MultiArray arr;
    float armX = 0, armY = 0;
    if(event==CV_EVENT_LBUTTONDOWN||event==CV_EVENT_RBUTTONDOWN){
        /*  Get point cloud data */
        g_pointcloud_mutex.lock();
        rs::float3 & point = g_pointcloud[FRAME_WIDTH * y + x];
        g_pointcloud_mutex.unlock();        

        double distance = sqrt(pow((double)(point.x - last_point.x), 2.0) + pow((double)(point.y, - last_point.y), 2.0) + pow((double)(point.z - last_point.z), 2.0));
        
        printf("Point world(t) (%f, %f, %f)\n", last_point.x, last_point.y, last_point.z);
        printf("Point world(t+1) (%f, %f, %f)\n", point.x, point.y, point.z);
        printf("Distance (%f)\n", distance);    
        last_point = point;   

        arr.data.clear();        
        cout<<"X:"<<x<< ",Y: "<<y<<endl;
        mouseC = cvPoint(x,y);
        armX = (y-220)*84.0/48;//armX = y
        armY = (282-x)*62.0/64*-1;//armY = -x
        cout<<"armX:"<<armX<<",armY:"<<armY<<endl;
        arr.data.push_back(100);
        arr.data.push_back(armX);
        arr.data.push_back(armY);
        arr.data.push_back(3330);
        //chatter_pub->publish(arr);
        //ros::spinOnce();
        //loop_rate.sleep();
        //g_ros_wrapper->publish<std_msgs::Float32MultiArray>(arr);
        
    }
}
