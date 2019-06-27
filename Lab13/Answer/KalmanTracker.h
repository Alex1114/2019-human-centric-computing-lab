
// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>

// Output
#include <iostream>

// Vector
#include <vector>

#include "ros/ros.h"


using namespace std;


class KalmanTracker {

private:
	int stateSize, measSize, contrSize;
	unsigned int type;
	int foundCount, number;
    bool found;
	double ticks, preTick;

	cv::KalmanFilter kf;
	cv::Mat state;
	cv::Mat meas;

	ros::Time t_now, t_prev;


public:
	KalmanTracker() {

		found = false;
		foundCount = 0;

		// >>>> Kalman Filter
	    int stateSize = 6;
		int measSize = 6;
		int contrSize = 0;

	    type = CV_64F;
	    kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

	    state = cv::Mat(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
	    meas = cv::Mat(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
	    //cv::Mat procNoise(stateSize, 1, type)
	    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

	    // Transition State Matrix A
	    // Note: set dT at each processing step!
	    // [ 1 0 dT 0  0 0 ]
	    // [ 0 1 0  dT 0 0 ]
	    // [ 0 0 1  0  0 0 ]
	    // [ 0 0 0  1  0 0 ]
	    // [ 0 0 0  0  1 0 ]
	    // [ 0 0 0  0  0 1 ]
	    cv::setIdentity(kf.transitionMatrix);

	    // Measure Matrix H
	    // [ 1 0 0 0 0 0 ]
	    // [ 0 1 0 0 0 0 ]
	    // [ 0 0 1 0 0 0 ]
	    // [ 0 0 0 1 0 0 ]
	    // [ 0 0 0 0 1 0 ]
	    // [ 0 0 0 0 0 1 ]
	    cv::setIdentity(kf.measurementMatrix);

	    // Process Noise Covariance Matrix Q
	    // [ Ex   0   0     0     0    0  ]
	    // [ 0    Ey  0     0     0    0  ]
	    // [ 0    0   Ev_x  0     0    0  ]
	    // [ 0    0   0     Ev_y  0    0  ]
	    // [ 0    0   0     0     Ew   0  ]
	    // [ 0    0   0     0     0    Eh ]
	    cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-1));

	    // Measures Noise Covariance Matrix R
	    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
	}

	void measIn(ros::Time t_now, double x, double y, double w, double h) {
		if (!found) // First detection!
        {
            // >>>> Initialization
            kf.errorCovPre.at<double>(0) = 1e-1;
            kf.errorCovPre.at<double>(7) = 1e-1;
            kf.errorCovPre.at<double>(14) = 1e-1;
            kf.errorCovPre.at<double>(21) = 1e-1;
            kf.errorCovPre.at<double>(28) = 1e-1;
            kf.errorCovPre.at<double>(35) = 1e-1;


            // <<<< Initialize the state vector
            /*----------------------
                    code
            -----------------------*/
			state.at<double>(0) = x;
			state.at<double>(1) = y;
			state.at<double>(2) = 0;
			state.at<double>(3) = 0;
			state.at<double>(4) = w;
			state.at<double>(5) = h;

            kf.statePost = state;

            found = true;
            foundCount = 2;
            this->t_now = t_now;
        }


        else {
		    t_prev = this->t_now;
            this->t_now = t_now;

            ros::Duration diff = t_now - t_prev;
            double dT = diff.toSec();

            // >>>>> Kalman Update


            // Calculate velocity of x and y
			double vx;
			double vy;
			if(dT==0){
				vx = state.at<double>(2);
				vy = state.at<double>(3);
			}
			else{
				vx = (x - state.at<double>(0))/dT;
				vy = (y - state.at<double>(1))/dT;
			}

            // Setup meas vector and update
            meas.at<double>(0) = x;
			meas.at<double>(1) = y;
			meas.at<double>(2) = vx;
			meas.at<double>(3) = vy;
			meas.at<double>(4) = w;
			meas.at<double>(5) = h;

            kf.correct(meas); // Kalman Correction
		    //state = kf.predict();
        }
		cout << "measure" << endl;
	}

	vector<double> predict(ros::Time t_now) {
        vector<double> stateVector;

		t_prev = this->t_now;
        ros::Duration diff = t_now - t_prev;
        this->t_now = t_now;

        double dT = diff.toSec();
        // Put dT into transitionMatrix
		kf.transitionMatrix.at<double>(2) = dT;
		kf.transitionMatrix.at<double>(9) = dT;

        // Setup stateVector to return
		if(found) {
			state = kf.predict();
            stateVector.push_back(state.at<double>(0));
            stateVector.push_back(state.at<double>(1));
            stateVector.push_back(state.at<double>(4));
            stateVector.push_back(state.at<double>(5));

		}

		else {
            stateVector.push_back(0);
            stateVector.push_back(0);
            stateVector.push_back(0);
            stateVector.push_back(0);

	    }
        return stateVector;
	}

    int getFoundCount() { return foundCount;}
    void setFoundCount(int num) { foundCount += num;}
    int getNumber() { return number;}
    void setNumber(int num) { number = num;}
};

