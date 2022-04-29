#include "iostream"
#include "ros/ros.h"
// #include "std_msgs/String.h"
// #include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>


#include "test.h"

using namespace std;
using namespace cv;

// Driver Code

// static void RGBListener()
// 	{
// 		// ros::init("Image_test");
// 		// ros::NodeHandle node;
// 		// ros::Subscriber sub = node.subscribe("/camera/rgb/image_color", 1, image_test.RGBCallback);
// 		ros::spin();
// 	}

double* midpoint(int ptA[], int ptB[])
	{
		static double temp[2] = {(ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5};
		return temp;
	}

void RGBCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			// cout<<"banana"<<endl;
			cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
			cv::waitKey(30);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
		}
	}
	
int main(int argc, char **argv)
{
	//Log("test start");
	// Mat image ;
	
	ros::init(argc,argv,"Image_test");
	ros::NodeHandle node;
	// cv::medWindow("view",WINDOW_AUTOSIZE);

	image_transport::ImageTransport it(node);

	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, RGBCallback);
	ros::spin();
	//cv::destroyAllWindows();

	int ptA[2] = {1,2};
	int ptB[2] = {3,4};
	double* ans = midpoint(ptA,ptB);
	// cout<<ans<<endl;
	// for(int i = 0; i < 2; i++)
	// 	{
	// 		cout<<ans[i]<<endl;
	// 	}
	// std::cin.get();

	return 0;
}


