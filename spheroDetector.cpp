#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
#include <math.h>

using namespace cv;

//Suppress the overlapping features, in favor of the largest nearby feature.
bool supress(vector<KeyPoint>& keypoints, KeyPoint& query)
{
	for(size_t i = 0; i < keypoints.size(); i++){
		int distx = keypoints[i].pt.x - query.pt.x;
		int disty = keypoints[i].pt.y - query.pt.y;
		int dist = sqrt(distx*distx + disty*disty);
		if((keypoints[i].size > query.size) && (dist < keypoints[i].size/2))
			return true;
	}
	return false;
}

int main(int, char**)
{
	string filename = "VIDEO0042.avi";
	VideoCapture cap(filename); // open the file.
	if(!cap.isOpened())  // check if we succeeded
		return -1;
	
	//Initialize variables.
    Mat hsv,blur,thresholded, mser;
	//Create a MSER feature detector
	Ptr<FeatureDetector> defaultDetector;
	defaultDetector = FeatureDetector::create("MSER");
	int min_area = 100;
	int min_thresh = 230;
    namedWindow("hsv",1);
    namedWindow("normal",2);
	namedWindow("blur",3);
	for(;;) {
		defaultDetector->set("minArea",min_area);
		//Read in a frame from video data.		
		Mat frame;
		cap >> frame;
		//Convert to HSV color space.
        cvtColor(frame, hsv, CV_BGR2HSV);
		//Threshold the image to only look for bright spots.
		inRange(hsv, Scalar(0, 0, min_thresh), Scalar(256, 256, 256), thresholded);
		//Blur the image to make detections more robust.
		GaussianBlur(thresholded, blur, Size(9,9), 3, 3);
		//Apply the MSER detector to the blurred image.
		vector<KeyPoint> keypoints
		defaultDetector->detect(blur,keypoints);
		//Display the maximal keypoints over the original image.
		for(size_t i = 0; i < keypoints.size(); i++){
			if(!supress(keypoints,keypoints[i])){		
				Point center = keypoints[i].pt;
				int radius = cvRound(keypoints[i].size/2);
				Point topLeft = center;
				topLeft.x -= radius;
				if(topLeft.x < 0)
					topLeft.x = 0;
				topLeft.y -= radius;
				if(topLeft.y < 0)
					topLeft.y = 0;
				Point botRight = center;
				botRight.y += radius;
				if(botRight.y >= frame.rows)
					botRight.y = frame.rows-1;
				botRight.x += radius;
				if(botRight.x >= frame.cols)
					botRight.x = frame.cols-1;
				int width = botRight.x - topLeft.x;
				int height = botRight.y - topLeft.y;
				//Bounding box of a maximal detection.
				Rect r(topLeft.x,topLeft.y,width,height);
				//Draw the rectangle to the output 
				rectangle(frame,r,Scalar(256,0,0),1,CV_AA);
			}
		}
		//Show all of the frames.
		imshow("hsv",hsv);
        imshow("normal", frame);
		imshow("blur", blur);
		//Wait a pre-slected amount of time to keep framerate consistent if possible.
		if(waitKey(30) >= 0) break;
	}
    return 0;
}
