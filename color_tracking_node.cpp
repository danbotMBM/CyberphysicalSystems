//~/catkin_ws/src/color_tracking/src/color_tracking_node.cpp
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include "std_msgs/Int16.h"
//#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include <iostream>

//CHANGE
#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>

#include <csignal>
#include <cstdio>
#include <math.h>


using namespace std;
using namespace cv;

//#define MEASURE_TIME 1
#define COLOR_TRACKING 1

#ifdef MEASURE_TIME
clock_t t_begin = 0;
#endif

static int counter = 0;
static int goal = 1000;
static double time_record[] = {};

int detected_count = 0;

void cv_process_img(const Mat& input_img, Mat& output_img)
{
    Mat gray_img;
    cvtColor(input_img, gray_img, CV_RGB2GRAY);
    
    double t1 = 20;
    double t2 = 50;
    int apertureSize = 3;
    
    Canny(gray_img, output_img, t1, t2, apertureSize);
    //cv::imshow("canny edge",output_img);
}

void cv_publish_img(image_transport::Publisher &pub, Mat& pub_img)
{
    //sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pub_img).toImageMsg();
    sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", pub_img).toImageMsg();
    pub.publish(pub_msg);
}

void cv_color_tracking(const Mat& input_img, ros::Publisher &controlPub)
{
    int iLowH = 160;
    int iHighH = 179;
    
    int iLowS = 150;
    int iHighS = 255;
    
    int iLowV = 60;
    int iHighV = 255;

    // Mat gray_img;
    // cvtColor(input_img, gray_img, CV_BGR2GRAY);

    // Mat channels[3];
    // split(input_img, channels);

    
    
    // double t1 = 150;
    // double t2 = 200;
    // int apertureSize = 3;

    // Mat edges;
    
    // Canny(channels[2], edges, t1, t2, apertureSize);

    // cv::imshow("red", channels[2]);

    
    // vector<vector<Point>> contours;
    // findContours(edges, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    // vector<vector<Point>> contours_poly(contours.size());
    // vector<Rect> boundRect(contours.size());
    
    // for(size_t i = 0; i < contours.size(); i++){
    //     approxPolyDP(contours[i], contours_poly[i], 3, true);
    //     boundRect[i] = boundingRect(contours_poly[i]);
    // }
    // Mat drawing = Mat::zeros(edges.size(), CV_8UC3);

    // for(size_t i = 0; i < contours.size(); i++){
    //     Scalar color = Scalar(255, 0,0);
    //     drawContours(drawing, contours, (int)i, color);
    //     rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
    // }
    // cv::imshow("drawing", drawing);
    Mat imgLines = Mat::zeros(input_img.size(), CV_8UC3);    
    Mat imgHSV;
    //convert input image from RGB to HSV
    cvtColor(input_img, imgHSV, CV_BGR2HSV);

    Mat1b mask1;
    Mat1b mask2;
    
    // inRange(imgHSV, Scalar(105, 150, 50), Scalar(125, 255, 255), mask1); // red
    // Mat1b mask = mask1;

    cv::inRange(imgHSV, cv::Scalar(0, 100, 20), cv::Scalar(9, 255, 255), mask1);    // red
    cv::inRange(imgHSV, cv::Scalar(171, 100, 20), cv::Scalar(180, 255, 255), mask2); // red
    cv::Mat1b mask = mask1 | mask2;

    // cv::inRange(imgHSV, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);    //Blue
    // cv::inRange(imgHSV, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2); // Blue
    // cv::Mat1b mask = mask1 | mask2;

    erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    
    //morphological closing (removes small holes from the foreground)
    dilate( mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    
    // //Find the contour, then indentify rectangle, center, radius using openCV build in functions
    // vector<vector<Point> > contours;
    // vector<Vec4i> hierarchy;
    // findContours(mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    // cout << "Contours.size(): " << contours.size() << std::endl;

    //GaussianBlur(mask, mask, Size(9, 9), 2, 2);


    
    
    


    // vector<bool> detects(contours.size(), false);
  

    // size_t num_contours = contours.size();
    // for (size_t i = 0; i < num_contours; i++) {
    //     double area = contourArea(contours[i]);
    //     //cout << "Area:" << area << endl;
    //     if (area > limit && area < upperLimit) {
    //         cout << "Area: " << area << endl;
    //         detects[i] = true;
    //     }
    // }
    
    // // DRAW BOUNDING BOXES

    // findContours(mask, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    // vector<vector<Point>> contours_poly(contours.size());
    // vector<Rect> boundRect(contours.size());

    // for(size_t i = 0; i < contours.size(); i++){
    //     approxPolyDP(contours[i], contours_poly[i], 3, true);
    //     boundRect[i] = boundingRect(contours_poly[i]);
    // }

    
    // for(size_t i = 0; i < contours.size(); i++){
    //     Scalar color = Scalar(255, 0,0);
    //     if (detects[i]){
    //         drawContours(input_img, contours, (int)i, color);
    //         rectangle(input_img, boundRect[i].tl(), boundRect[i].br(), color, 2);
    //     }
    // }


    double limit = 2000;
    double upperLimit = 180000;
    int detected = false;
    std_msgs::Int16 msg;

    //DETECT CIRCLES
    vector<Vec3f> circles;
    HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, mask.rows/8, 100, 20, 0, 0);

    Point circle_center;
    cout << "# of circles detected" << circles.size() << endl;    
    if (circles.size() > 0){
        for(size_t j = 0; j < 1; j++){
            circle_center = Point(std::round(circles[j][0]), std::round(circles[j][1]));
            int radius = std::round(circles[j][2]);
            Scalar green = Scalar(0, 255, 0);
            circle(input_img, circle_center, radius, green, 5);
            //draw x in the middle
            putText(input_img, "X", circle_center, FONT_HERSHEY_COMPLEX, 1, green, 2);
            detected = true;
        }
    }

    int image_width = input_img.cols;
    
    if (detected) {
        cout << image_width << " " << circle_center <<std::endl;
        string location ="NONE";
        if(circle_center.x < image_width / 3){
            location = "LEFT";
        }else if(circle_center.x < image_width * 2 / 3){
            location = "CENTER";
        }else{
            location = "RIGHT";
        }
        cout << "Detected " << location <<std::endl;
        msg.data = 1;
        controlPub.publish(msg);
    }else{
        cout << "Haven't detected" << std::endl;
	    msg.data = 0;
        controlPub.publish(msg);
    }
    cv::imshow("color_tracking_input_image", input_img);
    cv::imshow("red_range", mask);

    waitKey(1);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::Publisher &pub, ros::Publisher &controlPub)
{
    
    auto start = std::chrono::high_resolution_clock::now();
    cv_bridge::CvImageConstPtr cv_ori_img_ptr;
    try{
        Mat cv_ori_img = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat cv_output_img;
        
        cv_process_img(cv_ori_img, cv_output_img);
        
#ifdef COLOR_TRACKING
        cv_color_tracking(cv_ori_img, controlPub);
#endif
        
        cv_publish_img(pub, cv_output_img);
        
#ifdef MEASURE_TIME
        clock_t t_end = clock();
        double delta_time= double(t_end - t_begin) / CLOCKS_PER_SEC;
        //cout << "Delta_t = " << 1/delta_time << "\n";
        //t_begin = t_end;
#endif
        
        //imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //imshow("view", cv_output_img);
        waitKey(30);
        
#ifdef MEASURE_TIME
        t_begin = clock();
#endif
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    // process time measurement
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    time_record[counter] = (double)elapsed.count();
    counter += 1;

    if(counter == goal){
        double sum = 0.0;
        for(int i = 0; i < goal; i ++){
            sum += time_record[i];
        }
        double mean = sum/goal;
        const char *path = "/home/nvidia/catkin_ws/time_test/color_tracking_recording.txt";
        ofstream file;
        file.open(path, ios::app);
        file << "The mean is: " << mean*1000 << "ms" << endl;
        time_t now = time(0);
        char* dt = ctime(&now);
        file << dt << '\n' << '\n';
        counter = 0;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    
    ros::NodeHandle nh;
    
    //namedWindow("Vview");
    startWindowThread();
    image_transport::ImageTransport it(nh);
    
    ros::NodeHandle nh_pub;
    image_transport::ImageTransport itpub(nh_pub);
    image_transport::Publisher pub = itpub.advertise("sample/cannyimg", 1);
    
    ros::NodeHandle node;
    uint32_t queue_size = 500;
    ros::Publisher controlPub = node.advertise<std_msgs::Int16>("imagecontrol", queue_size);
    
    
#ifdef MEASURE_TIME
    t_begin = clock();
#endif
    
    image_transport::Subscriber sub = it.subscribe("rgb/image_rect_color", 1, boost::bind(imageCallback, _1, pub, controlPub));
    //ros::Subscriber controlSub = it.subscribe("rbg/image_rect_color")
    ros::spin();
    

    
    //destroyWindow("view");
    
}

