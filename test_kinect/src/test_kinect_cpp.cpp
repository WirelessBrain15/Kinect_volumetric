#include "iostream"
#include "ros/ros.h"
// #include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
// #include <sensor_msgs/ChannelFloat32>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";


class volumetric_analysis
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_sub_;
	// flag = true;

public:
	volumetric_analysis()
		: it_(nh_)
	{
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &volumetric_analysis::image_callback, this);
		depth_sub_ = it_.subscribe("/camera/depth/image", 1, &volumetric_analysis::depth_callback, this);
	
		namedWindow(OPENCV_WINDOW, WINDOW_AUTOSIZE);
		// namedWindow("gray", WINDOW_AUTOSIZE);
	}

	~volumetric_analysis()
	{
		destroyWindow(OPENCV_WINDOW);
	}

	double* midpoint(int ptA[], int ptB[])
	{
		static double temp[2] = {(ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5};
		return temp;
	}

	void image_callback(const sensor_msgs::ImageConstPtr& image_msg)
	{
		std_msgs::Header msg_header = image_msg->header;
		std::string frame_id = msg_header.frame_id.c_str();
		cv_bridge::CvImagePtr cv_ptr;
		Mat src, gray, thresh, canny_output;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

//		try
//		{
			// cap >> frame;
			// cvtColor(frame ,frame ,COLOR_BGR2GRAY);
			// cout<<"banana"<<endl;
			
			cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
			// src = cv_ptr->image;
			src = cv_bridge::toCvShare(image_msg, "bgr8")->image;
			// double scaleFactor = 1 / 255.0;
			// double shift = 0.0;
			//src.convertTo(src, CV_16FC3 , scaleFactor,shift);
			// cout << src.type() << endl;
			// src.convertTo(src, CV_8UC3,255.0/65536.0);
			cout << src.type() << endl;
			
			// cvtColor(src, gray, COLOR_GRAY2BGR);
			// cvtColor(gray, gray, COLOR_RGB2BGR);
			cvtColor(src, gray, COLOR_BGR2GRAY);
			//  gray.convertTo(gray,CV_8UC1,255.0/65536.0);
			threshold(gray, thresh, 150, 255, THRESH_BINARY);
			// GaussianBlur(thresh, gray, Size(3,3), 0);
			cout << gray.size() << endl;
			cout << gray.type() << endl;

			// cout<<"marker1"<<endl;

			// Canny(src, canny_output, 50, 150, 3 );

			// cout<<"marker2"<<endl;

			// findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));

			// cout<<"marker3"<<endl;

			// vector<Moments> mu(contours.size());
			// for(int i = 0; i,contours.size(); i++)
			// {
			// 	mu[i] = moments(contours[i], false);
			// }

			// cout<<"marker4"<<endl;

			// Moments m = moments(thresh, true);
			// Point p(m.m10/m.m00, m.m01/m.m00);
			// cout<<Mat(p)<<endl;
			// cout<<src.channels()<<endl;

			// circle(src, Point(50, 50), 10, Scalar(255,0,0));
			// drawMarker(cv_ptr->image, Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  Scalar(0, 0, 255), MARKER_CROSS, 10, 1);
			try 
			{
				// if flag == true
				imshow(OPENCV_WINDOW, thresh);
				// imshow("gray",gray);

			}
			catch(...)
			{

			}
			// flag = false;
			
			cv::waitKey(3);
//		}
		//catch (cv_bridge::Exception& e)
	//	{
	//		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
	//		return;
	//	}
	}

	// void depth_callback(const std_msgs::Float32::ConstPtr& depth_msg)
	void depth_callback(const sensor_msgs::ImageConstPtr& depth_msg)
	{
		cv_bridge::CvImagePtr cv_depth_ptr;
		try
		{
			// cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
			// Mat src = cv_ptr->image;
			cout<<"apple"<<endl;
			// double distance;
			// distance = cv_ptr->image.at<u_int16_t>(cv_ptr->image.rows/2, cv_ptr->image.cols/2);
			// // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
			// cout<<distance<<endl;

			// static std::vector<uint16_t> kdepth( 640 * 480 * 4); 
			// static unsigned int dBufSize = 640 * 480 * 4;
			// int dWidth = 320;
			// int dHeight = 240;
			// Mat mDepth = Mat(dHeight, dWidth, CV_16UC1,&depth[0]).clone();

			// mDepth.convertTo(mDepth,CV_8U, 1.0/255);
			// imshow(OPENCV_WINDOW,mDepth);

			// imshow(OPENCV_WINDOW,cv_depth_ptr->image);
			cv::waitKey(3);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", depth_msg->encoding.c_str());
			return;
		}
	}

};
	
int main(int argc, char** argv)
{
	ros::init(argc, argv, "volumetric_analysis");
	volumetric_analysis va;
	ros::spin();

	// int ptA[2] = {1,2};
	// int ptB[2] = {3,4};
	// double* ans = midpoint(ptA,ptB);
	// cout<<ans<<endl;
	// for(int i = 0; i < 2; i++)
	// 	{
	// 		cout<<ans[i]<<endl;
	// 	}

	return 0;
}


