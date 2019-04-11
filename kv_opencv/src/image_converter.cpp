//
// Created by kevinlinpr on 19-4-10.
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

static const std::string OPENCV_WINDOW = "Image window";
static const char* image_window = "Source Image";
static const char* result_window = "Result window";
class ImageConvertor{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    bool use_mask;
    cv::Mat img;
    cv::Mat templ;
    cv::Mat mask;
    cv::Mat result;
    int match_method;
    //int max_Trackbar = 5;
public:
    ImageConvertor():it_(nh_){
        //Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/kv/camera1/image_raw",1,&ImageConvertor::imageCb,this);
        image_pub_ = it_.advertise("/image_convertor/output_video",1);
        cv::namedWindow(OPENCV_WINDOW);
    }
    ~ImageConvertor(){
        cv::destroyWindow(OPENCV_WINDOW);
    }
    void imageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImageConstPtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }
        //Draw an example circle on the video stream
//        if(cv_ptr->image.rows>60 && cv_ptr->image.cols>60){
//            cv::circle(cv_ptr->image,cv::Point(50,50),10,CV_RGB(255,0,0));
//        }

        cv::Mat gray;
        cv::cvtColor(cv_ptr->image,gray,cv::COLOR_BGR2GRAY);

        cv::GaussianBlur(gray,gray,cv::Size(9,9),2,2);

        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(gray,circles,cv::HOUGH_GRADIENT,1,30,200,50,0,0);
        for (int i = 0; i < circles.size(); ++i) {
            auto c = circles[i];
            auto center = cv::Point(c[0],c[1]);
            circle(cv_ptr->image,center,1,cv::Scalar(0,100,100),3,cv::LINE_AA);
            auto radius = c[2];
            circle(cv_ptr->image,center,radius,cv::Scalar(255,0,255),3,cv::LINE_AA);
        }

        //Update GUI Window
        cv::imshow(OPENCV_WINDOW,cv_ptr->image);
        cv::waitKey(3);

        //Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    void MatchingMethod( int, void* )
    {
        cv::Mat img_display;
        img.copyTo( img_display );
        int result_cols =  img.cols - templ.cols + 1;
        int result_rows = img.rows - templ.rows + 1;
        result.create( result_rows, result_cols, CV_32FC1 );
        bool method_accepts_mask = (CV_TM_SQDIFF == match_method || match_method == CV_TM_CCORR_NORMED);
        if (use_mask && method_accepts_mask)
        { matchTemplate( img, templ, result, match_method, mask); }
        else
        { matchTemplate( img, templ, result, match_method); }
        normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
        double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
        cv::Point matchLoc;
        minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
        if( match_method  == cv::TM_SQDIFF || match_method == cv::TM_SQDIFF_NORMED )
        { matchLoc = minLoc; }
        else
        { matchLoc = maxLoc; }
        rectangle( img_display, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
        rectangle( result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
        imshow( image_window, img_display );
        imshow( result_window, result );
        return;
    }
};

int main(int argc,char** argv){
    ros::init(argc,argv,"image_convertor");
    ImageConvertor ic;
    ros::spin();
    return 0;
}

