#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std;
using namespace cv;
cv::Mat imLeft;	


void chatterCallback(const sensor_msgs::CompressedImageConstPtr& msg){
	cv::Mat img;
	try
	{
		img = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
		cv::waitKey(10);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert to image!");
	}
	//input
    cv::Mat K_l = (Mat_<double>(3,3) << 690.6878510284205, 0.0, 633.4759793568744, 0.0,690.4070560945236, 379.0474946457819, 0.0, 0.0, 1.0);
	cv::Mat P_l = (Mat_<double>(3,4) << 563.7450680031019, 0, 633.4683456420898, 0, 0, 563.7450680031019, 385.850227355957, 0, 0, 0, 1, 0);
	cv::Mat R_l = (Mat_<double>(3,3) << 0.9999346240199585, -0.0009897399284775627, 0.01139158026519184, 0.0009539994069631796, 0.9999946074095437, 0.003142457153890984, -0.01139462905038356, -0.003131384151857443, 0.9999301759933529);
	cv::Mat D_l = (Mat_<double>(1,5) << -0.17133083757747497, 0.026929992494427353, 0.0009495119036639311, -0.00023640686589512599, 0);
	cv::Mat M1l,M2l; //output
	cv::initUndistortRectifyMap(K_l,D_l,R_l, P_l.rowRange(0,3).colRange(0,3),cv::Size(1280, 720), CV_32FC1, M1l, M2l);
 	cv::remap(img, imLeft, M1l, M2l,cv::INTER_LINEAR);
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "undistort_image");  
	ros::NodeHandle n;
    //write subscriber and publisher down here 
	//the callback function(how to undistort image is written)
	

	return 0;
}
