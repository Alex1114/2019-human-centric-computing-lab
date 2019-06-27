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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

using namespace cv;
namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap, bool bReuseMap):mpMap(pMap)
{
    if (bReuseMap)
        mState=Tracking::LOST;
    else
        mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

	bool showfeamotionmodel=false;
	bool showmovingdetec=false;
	bool showfeatexppt=0;
	bool showdepthcolor=0;
	bool showstaticgrid=0;
	bool showyolocluster=1;
	bool showboundingboxes=1; 
	bool showpredictboundingboxes=1;
    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
    	
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
		
		
		int boxnum=0;
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
            	/*
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
				*/
				
                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    //cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    //cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    //cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    //cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        
        
            
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
				

				//add draw moving object feature
				if(showmovingdetec)
				{
					if(!movingobfd.empty() )
					{
						if(movingobfd[i]==1)
						{
							//scalar blue green red
							cv::rectangle(im,pt1,pt2,cv::Scalar(0,0,255));
		                	cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,0,255),-1);

							std::ostringstream ss;
							ss << CurrentFrame.movedist[i];
							std::string s(ss.str());
							cv::putText(im, s, vCurrentKeys[i].pt, FONT_HERSHEY_PLAIN, 1,  Scalar(255,255,255));
						}
						if (movingobfd[i]==-1)
						{
							cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
		                	cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);

							std::ostringstream ss;
							ss << CurrentFrame.movedist[i];
							std::string s(ss.str());
							cv::putText(im, s, vCurrentKeys[i].pt, FONT_HERSHEY_PLAIN, 1,  Scalar(255,255,255));
						}
						
					}
				}

				//draw optical flow
				//cv scalar blue green red

				//畫出兩個frame的feature matching
/*
				int z=0;
                if( !mtfmp.empty())
                {
					int j= mtfmp[i];
					if(j>0)
					{
				        if(mvLastKeys[i].pt.x<CurrentFrame.mnMinX || mvLastKeys[i].pt.x>CurrentFrame.mnMaxX)				     
				            cout<<"????????????????????????"<<endl;
						if(CurrentFrame.mvDepth[j]<19.2 && CurrentFrame.mvDepth[j]>0)
						{
						if( abs(vCurrentKeys[ j].pt.x - mvLastKeys[i].pt.x)+abs(vCurrentKeys[ j ].pt.y - mvLastKeys[i].pt.y) <150 )
						{
							//cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
							if(CurrentFrame.movedist[j]<0.5)
							{
								cv::circle(im,vCurrentKeys[ j ].pt,3,cv::Scalar(255,0,0),-1);
								cv::line(im, vCurrentKeys[ j ].pt, mvLastKeys[i].pt,Scalar(255,0,0),1);
							}
							else if (CurrentFrame.movedist[j]<1)
							{
								cv::circle(im,vCurrentKeys[ j ].pt,3,cv::Scalar(0,255,0),-1);
								cv::line(im, vCurrentKeys[ j ].pt, mvLastKeys[i].pt,Scalar(0,255,0),1);
							}
							else
							{
								cv::circle(im,vCurrentKeys[ j ].pt,3,cv::Scalar(0,0,255),-1);
								cv::line(im, vCurrentKeys[ j ].pt, mvLastKeys[i].pt,Scalar(0,0,255),1);
							}			
							std::ostringstream ss;
							ss << CurrentFrame.mvDepth[j];
							std::string s(ss.str());
							
							//cv::putText(im, s, vCurrentKeys[ j ].pt, FONT_HERSHEY_PLAIN, 1,  Scalar(255,255,255));
							//cout<<" last pt : "<< mvLastKeys[i].pt
						}
						else
							z++;
						}
						
	
					}
                }
				if(z>2)
					cout<<"hiiiiiiiiiiiii"<<endl;
			*/
				
				if(showfeamotionmodel)
				{
					if(!CurrentFrame.tfmp.empty() )
					{
						int j=CurrentFrame.tfmp[i];
						if( j>0 )
						{
							boxnum++;
							if( boxnum>0)
							{
								if(movingobfd[j]==-1)
								{
									float u=mexpkeypt[j].x;
									float v=mexpkeypt[j].y;

									int nLastOctave = mvLastKeys[i].octave;
									// Search in a window. Size depends on scale
									int th=3;
									float radius = th*CurrentFrame.mvScaleFactors[nLastOctave];
									if(radius>0)
									{
										cv::Point2f pt1,pt2;
										pt1.x=mexpkeypt[j].x-radius;
										pt1.y=mexpkeypt[j].y-radius;
										pt2.x=mexpkeypt[j].x+radius;
										pt2.y=mexpkeypt[j].y+radius;
										cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
								    	cv::circle(im,vCurrentKeys[j].pt,1,cv::Scalar(0,0,255),-1);
									}
							   	}
							}
						
						}
					}
				}
				//show feature have depth or not
/*
				if(CurrentFrame.mvuRight[i]==-1)
				{
					cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(0,0,255),-1);
				}
				else
				{
					cv::circle(im,vCurrentKeys[i].pt,1,cv::Scalar(0,255,0),-1);				
				}
*/
				//根據r和t來預測feature會出現的位置
				if(showfeatexppt)
				{
					if(!mexpkeypt.empty() )
					{
						if(mexpkeypt[i].x>0)
						{
							if( abs(CurrentFrame.mvKeys[i].pt.x - CurrentFrame.expkeypt[i].x)+abs(CurrentFrame.mvKeys[i].pt.y - CurrentFrame.expkeypt[i].y) <200 )
							{
						
								float error=( abs(CurrentFrame.mvKeys[i].pt.x - CurrentFrame.expkeypt[i].x)+abs(CurrentFrame.mvKeys[i].pt.y - CurrentFrame.expkeypt[i].y) )*CurrentFrame.mvDepth[i];

								if(showdepthcolor)
								{
									if(CurrentFrame.mvDepth[i]<20 && CurrentFrame.mvDepth[i]>0)
									{
										//cv::Mat gray = (Mat_<float>(1,1) << (255-CurrentFrame.mvDepth[i]*12.75 ) );
										cv::Mat gray= Mat::ones(1, 1,  CV_8UC1); // , (255-CurrentFrame.mvDepth[i]*12.75 ) );
										gray.at<unsigned char>(0,0)=(255-CurrentFrame.mvDepth[i]*12.75 );
										cv::Mat color;
										// Apply the colormap
										cv::applyColorMap(gray, color, COLORMAP_JET);
										cv::circle(im,vCurrentKeys[i].pt, 3, color.at<Vec3b>(0,0), -1);
										cv::line(im, vCurrentKeys[i].pt, mexpkeypt[i],Scalar(0,0,255),1);
									}
								}
								else if(CurrentFrame.mvDepth[i]<0)
								{
									cv::circle(im,vCurrentKeys[i].pt, 3, Scalar(255,0,0), -1);
									cv::line(im, vCurrentKeys[i].pt, mexpkeypt[i],Scalar(0,0,255),1);
								}
								else if(error<120)
								{
									cv::circle(im,vCurrentKeys[i].pt, 3, Scalar(0,255,0), -1);
									cv::line(im, vCurrentKeys[i].pt, mexpkeypt[i],Scalar(0,0,255),1);
								}
								else 
								{
									cv::circle(im,vCurrentKeys[i].pt, 3, Scalar(0,0,255), -1);
									cv::line(im, vCurrentKeys[i].pt, mexpkeypt[i],Scalar(0,0,255),1);
								}
							}
							else
								cout<<"--error----"<<endl;
						}
					}
				}		


		}

		if(showstaticgrid)
		{
			if(!CurrentFrame.movingob.empty())
			{
				int gridcols=64;
				int gridrows=48;
				for(int i=0; i<gridcols;i++)
				{	
					for (int j=0; j<gridrows;j++)
					{
						float cols= (i+0.5)/ CurrentFrame.mfGridElementWidthInv;
						float rows= (j+0.5)/ CurrentFrame.mfGridElementHeightInv;
						if(CurrentFrame.movingob[ j+ (i*gridrows) ] ==-1)
							cv::circle(im,Point(cols, rows), 5, Scalar(0,255,0), -1);
						else
							cv::circle(im,Point(cols, rows), 5, Scalar(0,0,255), -1);
					}
				}
			}
		}
		if(showyolocluster && !CurrentFrame.yoloclus.empty() ) {
			for(int i=0;i<CurrentFrame.yoloclus.size();i++){
				
				std::ostringstream ss;
				ss << i;
				std::string s(ss.str());
				for(int j=0;j<CurrentFrame.yoloclus[i].size();j++){
					int p= CurrentFrame.feaid[ CurrentFrame.yoloclus[i][j] ];
					cv::putText(im, s, vCurrentKeys[ p ].pt, FONT_HERSHEY_PLAIN, 1,  Scalar(0,0,255));	
					
		            cv::circle(im,vCurrentKeys[ p ].pt,2,cv::Scalar(0,255,0),-1);
				}			
			}
		}
		if(showboundingboxes && !bboxes.empty() )
		{
			for(int i=0;i<bboxes.size();i++){
				cv::rectangle(im, bboxes[i].first, bboxes[i].second, cv::Scalar(0,0,255) );		
			}
		}
		if(showpredictboundingboxes && !predictbboxes.empty() )
		{
			for(int i=0;i<predictbboxes.size();i++){
				cv::rectangle(im, predictbboxes[i].first, predictbboxes[i].second, cv::Scalar(0,255,0) );		
			}
		}
		
		//add show feature grid
		/*
		for(unsigned int i=1; i<109;i++)
		{	
			int cols= round (i/ CurrentFrame.mfGridElementWidthInv);
			cv::line(im, Point(cols, CurrentFrame.mnMinY ), Point(cols, CurrentFrame.mnMaxY),Scalar(0,0,255),1);
		} 
		//rows
		for (unsigned int j=1; j<60;j++)
		{
			int rows= round (j/ CurrentFrame.mfGridElementHeightInv);
			cv::line(im, Point(CurrentFrame.mnMinX, rows), Point( CurrentFrame.mnMaxX, rows),Scalar(0,0,255),1);
		}*/

		
        //cout<<"mnTracked"<<mnTracked<<endl;
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}
int j=0;
int z=-400;
cv::Mat twoim(cv::Size(2560, 1268), CV_8UC3);
//save a video
//VideoWriter video("outcpp.avi",CV_FOURCC('M','J','P','G'),10, Size(2560,1268)); 

void FrameDrawer::Update(Tracking *pTracker)
{
	//add use to moving ob
	CurrentFrame=pTracker->mCurrentFrame;
	movingobfd=pTracker->mCurrentFrame.movingob;
	mvLastKeys.clear();
	mvLastKeys.assign(pTracker->mLastFrame.mvKeys.begin(), pTracker->mLastFrame.mvKeys.end()); 
	mtfmp.clear();
	mtfmp.assign(pTracker->mCurrentFrame.tfmp.begin(), pTracker->mCurrentFrame.tfmp.end());
	mexpkeypt=pTracker->mCurrentFrame.expkeypt;
	//add bounding box
	bboxes=pTracker->boundingboxes;
    predictbboxes=pTracker->predictboundingboxes;
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    
	
	
	Frame Lastframe;
	//draw stereo matches and two frame matches
	bool showmatches=false;
	if(j==0 && z==10)
	{
		pTracker->mImGrayLeft.copyTo(twoim(cv::Rect(  0, 634, 1280, 634)));	
		namedWindow( "Display window");	// WINDOW_NORMAL
		
	}
	if(j==0 && z>10)
	{
		Lastframe=pTracker->mLastFrame;
		//cv::Mat img_1, img_2;
		
		//cvtColor(pTracker->mImGray,img_1,CV_GRAY2BGR);
		//cvtColor(pTracker->mImGrayRight,img_2,CV_GRAY2BGR);
		// Copy small images into big mat
		pTracker->mImGrayLeft.copyTo(twoim(cv::Rect(  0, 0, 1280, 634)));
		pTracker->mImGrayRight.copyTo(twoim(cv::Rect(1280, 0, 1280, 634)));
	}
	
	
    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
    	int countt=0;
    	int countt2=0;
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                    {
                        mvbMap[i]=true;
                        //draw line stereo matches
                        if(j==0 && z>10 && pTracker->mCurrentFrame.mvuRight[i]>0)
                        {
		                    countt++;
		                    Point2f b(pTracker->mCurrentFrame.mvuRight[i]+1280,pTracker->mCurrentFrame.mvKeys[i].pt.y);
		                    if(countt%3==0)
		                		cv::line(twoim,pTracker->mCurrentFrame.mvKeys[i].pt,b,Scalar(0,0,255),1);
		              		if(countt%3==1)
		                		cv::line(twoim,pTracker->mCurrentFrame.mvKeys[i].pt,b,Scalar(0,255,0),1);
		                	if(countt%3==2)
		                		cv::line(twoim,pTracker->mCurrentFrame.mvKeys[i].pt,b,Scalar(255,0,0),1);
		                }
		                
		                
                    }
                    else
                        mvbVO[i]=true;
                }
            }
        }
        
        //draw line two frame matches
        if(j==0 && z>10)
        {
		    for(int i=0; i<Lastframe.N; i++)
			{
				
				MapPoint* pMP = Lastframe.mvpMapPoints[i];
				
				if(pMP)
				{
				    if(!Lastframe.mvbOutlier[i])
				    {
				    	
				    	if(i<pTracker->mCurrentFrame.tfmp.size())
				    	{
				            if(j==0 && z>10 && pTracker->mCurrentFrame.tfmp[i]>0 && pTracker->mCurrentFrame.mvKeys[pTracker->mCurrentFrame.tfmp[i]].pt.x>0)
				            {
				            	countt2++;
				            	Point2f b(Lastframe.mvKeys[i].pt.x, Lastframe.mvKeys[i].pt.y+634);
				                if(countt2%3==0)
				            		cv::line(twoim,pTracker->mCurrentFrame.mvKeys[pTracker->mCurrentFrame.tfmp[i]].pt,b,Scalar(0,0,255),1);
				          		if(countt2%3==1)
				            		cv::line(twoim,pTracker->mCurrentFrame.mvKeys[pTracker->mCurrentFrame.tfmp[i]].pt,b,Scalar(0,255,0),1);
				            	if(countt2%3==2)
				            		cv::line(twoim,pTracker->mCurrentFrame.mvKeys[pTracker->mCurrentFrame.tfmp[i]].pt,b,Scalar(255,0,0),1);	
				            	
				            	//cout<<"LAST:"<<Lastframe.mvpMapPoints[i]->GetWorldPos()<<endl;
		   						//cout<<"NOW:"<<pTracker->mCurrentFrame.mvpMapPoints[ pTracker->mCurrentFrame.tfmp[i] ]->GetWorldPos()<<endl;
		   						cout<<"feature x y dist: "<<pTracker->mCurrentFrame.mvKeys[pTracker->mCurrentFrame.tfmp[i]].pt.x<<" "<<pTracker->mCurrentFrame.mvKeys[pTracker->mCurrentFrame.tfmp[i]].pt.y<<" "<<pTracker->mCurrentFrame.featuredist[pTracker->mCurrentFrame.tfmp[i]]<<endl;
				            }
		                }
		            }
		       	}
		       	
		   	}
		}
			    	
		       
        
        if(j==0 && z>10)
        {
        
		    //cout<<"stereomatches"<<countt<<endl;
		    //cout<<"tfmatches"<<countt2<<endl;
		   	
		    cv::imshow("Display window",twoim);
		    //cv::imwrite( "ORBmatching.jpg", twoim );
		    //save a video
		    //video.write(twoim);
		    cv::waitKey(1000/60); //1000/60
		    pTracker->mImGrayLeft.copyTo(twoim(cv::Rect(  0, 634, 1280, 634)));
		    
		}
		if(showmatches==true)
		{
			z++;
		}
    }
    
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
