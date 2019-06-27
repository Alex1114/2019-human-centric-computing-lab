/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raul Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"../../../include/System.h"

//add imu
#include "sensor_msgs/Imu.h"
#include <thread>
#include <string>
//ADD ROS PUB
/*
#include "../../../include/MapPoint.h"
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
*/
//add darknet_ros_msgs
//add publish current frame pose
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Core>
using namespace std;
using namespace cv;

//add ros pub
/*
pcl::PointCloud<pcl::PointXYZ>::Ptr lidarcloud(new pcl::PointCloud<pcl::PointXYZ>);
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ROS_INFO("getlidar");
	pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    
    pcl::fromPCLPointCloud2(pcl_pc2,*lidarcloud);
}
void rospub(ros::NodeHandle &n, ORB_SLAM2::System* mpSLAM)
{
	ros::Subscriber lidarsub = n.subscribe("/points_raw2", 100, lidarCallback);
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud2;
	ros::Publisher pub = n.advertise<PointCloud2> ("map", 100);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZ>);

	//add ros loop rate
	cloud_xyzrgb->header.frame_id = "map";
	ros::Rate loop_rate(10);
	//pcl::KdTreeFLANN<pcl::ointXYZ> kdtree;
 	while (n.ok())
    {   
		
		if(mpSLAM->mpTracker->mLastProcessedState==ORB_SLAM2::Tracking::OK && mpSLAM->mpTracker->mLastProcessedState!=ORB_SLAM2::Tracking::NOT_INITIALIZED)
		{	
		//kdtree.setInputCloud (lidarcloud);
		int K=1;
		std::vector<int> pointIdxNKNSearch(K);
  		std::vector<float> pointNKNSquaredDistance(K);

		//const vector<ORB_SLAM2::MapPoint*> &vpRefMPs = mpSLAM->getmap()->GetReferenceMapPoints();
		const vector<ORB_SLAM2::MapPoint*> &vpMPs = mpSLAM->mpTracker->mCurrentFrame.mvpMapPoints;
		


		if(vpMPs.empty())
        	continue;
		if( mpSLAM->mpTracker->mCurrentFrame.mTcw.empty())
			continue;
		cv::Mat Rcw = mpSLAM->mpTracker->mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
    	cv::Mat tcw = mpSLAM->mpTracker->mCurrentFrame.mTcw.rowRange(0,3).col(3);
		
		
		ROS_INFO("hi1");
		for(size_t i=0, iend=mpSLAM->mpTracker->mCurrentFrame.mvKeys.size(); i<iend;i++)
		{
			//cout<<"vpMPs.size()"<<vpMPs.size()<<endl;

		    if( vpMPs[i] )
			{
			if(!mpSLAM->mpTracker->mCurrentFrame.mvbOutlier[i])
			{
			if(vpMPs[i]->Observations()>0)
			{
			
		    cv::Mat pos = vpMPs[i]->GetWorldPos();
			
			pos = Rcw*pos+tcw;
			
			//add ros pub
			pcl::PointXYZ pcl_point_rgb;
			pcl_point_rgb.x = pos.at<float>(0);    // rgb PointCloud 
		    pcl_point_rgb.y = pos.at<float>(1); 
		    pcl_point_rgb.z = pos.at<float>(2); 
			cloud_xyzrgb->push_back(pcl_point_rgb); 
			
			  //if ( kdtree.nearestKSearch (pcl_point_rgb, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			 // {
				//for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
				//  std::cout << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
			 // }

			}
			}
			}
		}
		ROS_INFO("hi2");

    	pcl_conversions::toPCL(ros::Time::now(), cloud_xyzrgb->header.stamp);
       	pub.publish (cloud_xyzrgb);
       	ros::spinOnce ();
       	loop_rate.sleep(); 
		}
    }
	cout<<"end"<<endl;
}
*/

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);
    //add
	
	
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
	
	//add publisher
	ros::Publisher pose_pub;
	geometry_msgs::PoseStamped CameraPath;
};
//add
void readinput(string & tf)
{
	cin>>tf;
	cout<<"get "<<endl;
}

cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
double boxtime;
int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "RGBD");
    ros::start();
	// argc 4 to 5
    if(argc != 5 && argc!=4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;

        ros::shutdown();
        return 1;
    }    
    //add thread choose save map or not
    string torf;
	thread mThread( readinput,  ref(torf) );
	
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true, (bool)atoi(argv[4]) );


    ImageGrabber igb(&SLAM);
    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;
    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        //cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l, P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l), CV_32FC1,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r, P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r), CV_32FC1,igb.M1r,igb.M2r);
    }
    ros::NodeHandle nh;
	//add pub pointcloud
	//thread rosThread( rospub, ref(nh),&SLAM );
	//rosThread.detach();
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
	
	//add imu subscriber
    //message_filters::Subscriber<sensor_msgs::Imu> sub(nh, "/imu/data", 10);
	//add yolov3 bounding box subscriber



	//add publish pose
	igb.pose_pub = nh.advertise<geometry_msgs::PoseStamped>("camera_path", 1);

	ros::spin();


    
    // Stop all threads

	if(torf=="true")
	{
		SLAM.SaveMap();
		cout<<"save map:"<<torf <<endl;
	}
	
	//SLAM.SaveMap(argv[4]);
    
	
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
	//publish image pose
	cv::Mat mTcw=  mpSLAM->GetCurrentFramePose();
	if(!mTcw.empty()){
		cv::Mat mRcw = mTcw.rowRange(0,3).colRange(0,3);
		cv::Mat mRwc = mRcw.t();
		cv::Mat mtcw = mTcw.rowRange(0,3).col(3);
		cv::Mat mOw = -mRcw.t()*mtcw;

		Eigen::Matrix3f camr;
		camr<<  mRwc.at<float>(0,0), mRwc.at<float>(0,1), mRwc.at<float>(0,2),
				mRwc.at<float>(1,0), mRwc.at<float>(1,1), mRwc.at<float>(1,2), 
				mRwc.at<float>(2,0), mRwc.at<float>(2,1), mRwc.at<float>(2,2);
		Eigen::Quaternionf Camr( camr);

		CameraPath.header.frame_id = "/map";
		CameraPath.header.stamp= ros::Time::now();
		CameraPath.pose.position.x = mOw.at<float>(0,0);
		CameraPath.pose.position.y = mOw.at<float>(1,0);
		CameraPath.pose.position.z = mOw.at<float>(2,0);

		CameraPath.pose.orientation.x = Camr.x();
		CameraPath.pose.orientation.y = Camr.y();
		CameraPath.pose.orientation.z = Camr.z();
		CameraPath.pose.orientation.w = Camr.w();

		pose_pub.publish(CameraPath);		
	}

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        
        cv::remap(cv_ptrLeft->image,imLeft, M1l, M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
		/*  save image     
		if(i==1)
        {
        	cv::imwrite( "leftGray_Imagetestn1.jpg", imLeft );
        	cv::imwrite( "leftGray_Imagetestn2.jpg", imRight );
        	i++;
        }
        */
		// cut image
        Rect rect(0, 0, 1280, 605);
		cv::Mat imLeft2, imRight2;
		imLeft(rect).copyTo(imLeft2);
		imRight(rect).copyTo(imRight2);

        mpSLAM->TrackStereo(imLeft2,imRight2,cv_ptrLeft->header.stamp.toSec());

        
        //mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
       
    }
    else
    {
    	//Rect rect(0, 0, 1280, 634);
		//cv::Mat imLeft, imRight;
		//cv_ptrLeft->image(rect).copyTo(imLeft);
		//cv_ptrRight->image(rect).copyTo(imRight);

        mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
        //mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
  
    }
    /*    print speed
    if(countt%10==0  &&countt>50)
    {
    	
		mpSLAM->printspeed();
	}
	countt++;
	*/
}
//add yolo boundingbox

//add IMU
/*
void ImageGrabber::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
  {
	cv::Mat imu=cv::Mat(2, 4, CV_32F);
	
	imu.at<float>(0,0)=msg->linear_acceleration.x;
	imu.at<float>(0,1)=msg->linear_acceleration.y;
	imu.at<float>(0,2)=msg->linear_acceleration.z;

	imu.at<float>(1,0)=msg->orientation.x;
	imu.at<float>(1,1)=msg->orientation.y;
	imu.at<float>(1,2)=msg->orientation.z;
	imu.at<float>(1,3)=msg->orientation.w;

	mpSLAM->getimu(imu, (msg->header.stamp.nsec)/1e9);
}*/


