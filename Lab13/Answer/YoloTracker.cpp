#include "yoloTracking/YoloTracker.h"

static const std::string OPENCV_WINDOW = "Image window";

YoloTracker::YoloTracker(): it_(nh_) {
    init = 0;
    number = 0;
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &YoloTracker::pubImage, this);
    cv::namedWindow(OPENCV_WINDOW);
    for (int num = 0; num < 1000; num++)
        randomColor.push_back(cv::Scalar(rand() % 255, rand() % 255,rand() % 255));
}

YoloTracker::~YoloTracker() {
    cv::destroyWindow(OPENCV_WINDOW);
}

void YoloTracker::updateState(ros::Time& time,  vector<vector<double> >& detection, vector<string>& classVector) {
    // Predict the state vector
    vector<vector<double> > predictMat = predictState(time);

    // do data associate
    vector<int> association = getAssociation(detection, predictMat);

    // Update the state vector
    for (int num = 0; num < association.size(); num++) {
        int index = association[num];
        if (index != -1) {
            objectVector[index].tracker.measIn(time,detection[num][0],detection[num][1],detection[num][2],detection[num][3]);
            objectVector[index].cls = classVector[num];
            if (objectVector[index].tracker.getFoundCount() >= 3)
                objectVector[index].tracker.setFoundCount(1);
            else
                objectVector[index].tracker.setFoundCount(2);
        }
        // First in
        else {
            ObjectTracker newObject;
            newObject.serial =  number;
            newObject.cls = classVector[num];
            newObject.tracker.measIn(time,detection[num][0],detection[num][1],detection[num][2],detection[num][3]);
            incNumber();
            objectVector.push_back(newObject);
        }
    }

    // Delete the tracker if it doesn't be detect in several frame
    for (vector<ObjectTracker>::iterator it = objectVector.begin(); it != objectVector.end(); it++) {
        it->tracker.setFoundCount(-1);
        if (it->tracker.getFoundCount() <= 0) {
            objectVector.erase(it);
            it--;
        }
    }

    cout << "update state" <<endl;

}

vector<int> YoloTracker::getAssociation(vector<vector<double> >& yoloDetection, vector<vector<double> >& predictMatrix) {
    vector<vector<double> > lossMatrix(yoloDetection.size());
    vector<int> indexVector;
    if (!init) {
        for (int resultNum = 0; resultNum < yoloDetection.size(); resultNum++)
            indexVector.push_back(-1);
        init = 1;
    }

    else {
        for (int resultNum = 0; resultNum < yoloDetection.size(); resultNum++)
            lossMatrix[resultNum] = getLossVector(yoloDetection[resultNum], predictMatrix);
        
        cout <<"1"<< endl;
        HungarianAlgorithm solver;
        double cost = solver.Solve(lossMatrix, indexVector);
        cout <<"2"<< endl;
        for (int num = 0; num < indexVector.size(); num++) {
            if (indexVector[num] != -1)
                if (lossMatrix[num][indexVector[num]] >= 200)
                    indexVector[num] = -1;
        }
        cout <<"3"<< endl;
        
    }
    cout << "association"<<endl;
    return indexVector;
}

vector<vector<double> > YoloTracker::predictState(ros::Time& time) {
    vector<vector<double> > predictMat;
    for (int num = 0; num < objectVector.size(); num++) {
        predictMat.push_back(objectVector[num].tracker.predict(time));
    }
    return predictMat;
}

vector<double> YoloTracker::getLossVector(vector<double>& yoloDetection, vector<vector<double> >& predictMatrix) {
    vector<double> lossVector;
    for (int num = 0; num < predictMatrix.size(); num++) {
        double dx = predictMatrix[num][0]-yoloDetection[0];
        double dy = predictMatrix[num][1]-yoloDetection[1];
        double w = predictMatrix[num][2];
        double h = predictMatrix[num][3];
        double distance = sqrt(dx*dx+dy*dy);
        double boxErr = abs(yoloDetection[2]*yoloDetection[3]-w*h);
        cout << distance <<endl<<boxErr<<endl;
        double loss = distance + boxErr; // adjustable
        lossVector.push_back(loss);
    }
    cout << "compute loss" <<endl;
    return lossVector;
}

void YoloTracker::yoloCallback(const darknet_ros_msgs::BoundingBoxes& msg) {
    vector<vector<double> > detectionVector;
    vector<string> classVector;
    ros::Time time = msg.header.stamp;
    if (seq == msg.image_header.seq)
        return;
    else {
        seq = msg.image_header.seq;
        for (int num = 0; num < msg.bounding_boxes.size(); num++) {
            darknet_ros_msgs::BoundingBox box = msg.bounding_boxes[num];
            string cls = box.Class;
            double prob = box.probability;
            double x = (box.xmin+box.xmax)/2.;
            double y = (box.ymin+box.ymax)/2.;
            double w = box.xmax-box.xmin;
            double h = box.ymax-box.ymin;
            double vecinit[] = {x, y, w, h};
            vector<double> detection(vecinit, vecinit + sizeof(vecinit) / sizeof(double) );
            detectionVector.push_back(detection);
            classVector.push_back(cls);
        }
        updateState(time, detectionVector, classVector);
    }
}

void YoloTracker::incNumber() {number++;}

void YoloTracker::pubImage(const sensor_msgs::ImageConstPtr& msg){
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        for (int num = 0; num < objectVector.size(); num++) {
            if (objectVector[num].tracker.getFoundCount() < 2)
                continue;
            vector<double> obj = objectVector[num].tracker.predict(msg->header.stamp);
            int colorNum = objectVector[num].serial;
            double x = obj[0];
            double y = obj[1];
            double w = obj[2];
            double h = obj[3];
            cv::rectangle(cv_ptr->image, cv::Rect(x - w/2, y - h/2, w, h), randomColor[colorNum],3,1,0);

            double hh = 20;
            double ww = objectVector[num].cls.size() * 10 + 50;
            cv::rectangle(cv_ptr->image, cv::Rect(x - w/2, y - h/2 - hh, ww, hh), randomColor[colorNum], CV_FILLED,1,0);

            ostringstream str;
            str << objectVector[num].serial;
            string text = objectVector[num].cls + " : " + str.str();
            int font_face = cv::FONT_HERSHEY_TRIPLEX;
            double font_scale = 0.5;
            int thickness = 1.8;
            cv::putText(cv_ptr->image, text, cv::Point(x - w/2, y - h/2 - hh/3), font_face, font_scale, cv::Scalar(0, 0, 0), thickness, 8, 0);
        }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
  }
};
