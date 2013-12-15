/*The frame offset of the two images is:
* HorizontalView.3gp = 100
* VerticalView.mp4 = 20*/
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
	bool ret = false;
	for(size_t i = 0; i < keypoints.size(); i++){
		int distx = keypoints[i].pt.x - query.pt.x;
		int disty = keypoints[i].pt.y - query.pt.y;
		int dist = sqrt(distx*distx + disty*disty);
		if(query.size > keypoints[i].size && (dist < keypoints[i].size)){
			query.response++;
		}
		else if((keypoints[i].size > query.size) && (dist < keypoints[i].size)){
			ret = true;
		}
	}
	return ret;
}

int main(int, char**)
{
	string filename = "HorizontalView.3gp";
	VideoCapture cap(filename); // open the file.
	if(!cap.isOpened())  // check if we succeeded
		return -1;
	cap.set(CV_CAP_PROP_POS_FRAMES, 0);
	const string fileOutName = "HorizontalViewOut2.mp4";
	Size s = Size((int)cap.get(CV_CAP_PROP_FRAME_WIDTH), (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	VideoWriter outputVideo;
	outputVideo.open(fileOutName,CV_FOURCC('M', 'J', 'P', 'G'),30,s,true);
	if(!outputVideo.isOpened())
		return -1;
	//Initialize variables.
    Mat hsv,blur,thresholded, mser, rthresh;
	//Create a MSER feature detector
	Ptr<FeatureDetector> defaultDetector;
	Ptr<FeatureDetector> blobDetector;
	defaultDetector = FeatureDetector::create("MSER");
	blobDetector = FeatureDetector::create("SimpleBlob");
	int min_area = 700;
	int rmin,gmin,bmin,rmax,gmax,bmax = 0;
	int huemin,satmin,valmin,huemin2,satmin2,valmin2 = 0;
	int huemax,satmax,valmax,huemax2,satmax2,valmax2 = 256;
	valmin = 230;
	int colr =  0;
    namedWindow("hsv",1);
    namedWindow("normal",2);
	namedWindow("blur",3);
	//namedWindow("thresh",4);
	createTrackbar("huemin", "blur", &huemin, 256);
	createTrackbar("satmin", "blur", &satmin, 256);
	createTrackbar("valmin", "blur", &valmin, 256);
	createTrackbar("huemax", "blur", &huemax, 256);
	createTrackbar("satmax", "blur", &satmax, 256);
	createTrackbar("valmax", "blur", &valmax, 256);
	/*createTrackbar("rmin", "thresh", &rmin, 256);
	createTrackbar("gmin", "thresh", &gmin, 256);
	createTrackbar("bmin", "thresh", &bmin, 256);
	createTrackbar("rmax", "thresh", &rmax, 256);
	createTrackbar("gmax", "thresh", &gmax, 256);
	createTrackbar("bmax", "thresh", &bmax, 256);*/
	huemin = 123;
	satmin = 32;
	valmin = 218;
	huemax = 188;
	satmax = 256;
	valmax = 256;
	huemin2 = 60;
	satmin2 = 30;
	valmin2 = 134;
	huemax2 = 89;
	satmax2 = 150;
	valmax2 = 256;
	int frame_count = 0;
	for(;;) {
		defaultDetector->set("minArea",min_area);
		//Read in a frame from video data.		
		Mat frame,frameBot,frameTop;
		cap >> frame;
		
		//frameTop = frame.rowRange(Range(0,(int)(frame.rows/2));
		frameBot = frame.rowRange(Range((int)(frame.rows/2),frame.rows));
		//Convert to HSV color space.
        cvtColor(frameBot, hsv, CV_BGR2HSV);
		//Threshold the image to only look for bright red + green spots.
		inRange(hsv, Scalar(huemin,satmin,valmin), Scalar(huemax,satmax,valmax), thresholded);
		inRange(hsv, Scalar(huemin2,satmin2,valmin2), Scalar(huemax2,satmax2,valmax2), rthresh);
		bitwise_or(thresholded,rthresh,thresholded);
		//Blur the image to make detections more robust.
		GaussianBlur(thresholded, blur, Size(9,9), 3, 3);
		//Apply the MSER detector to the blurred image.
		vector<KeyPoint> keypoints;
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
				if(botRight.y >= frameBot.rows)
					botRight.y = frameBot.rows-1;
				botRight.x += radius;
				if(botRight.x >= frameBot.cols)
					botRight.x = frameBot.cols-1;
				int width = botRight.x - topLeft.x;
				int height = botRight.y - topLeft.y;
				//Bounding box of a maximal detection.
				Rect r(topLeft.x,topLeft.y,width,height);
				//Draw the rectangle to the output 
				rectangle(frameBot,r,Scalar(256,0,0),1,CV_AA);
				char c[50];
				int n = sprintf(c,"Response:%f",keypoints[i].response);
				string s(c,n);
				putText(frameBot,s,topLeft,FONT_HERSHEY_COMPLEX,1.0,Scalar(0,255,0));
				//circle( frameBot, center, 3, Scalar(0,255,0), -1, 8, 0 );
				//circle( frameBot, center, radius, Scalar(255,0,0), 3, 8, 0 );
			}
		}
	//	drawKeypoints( blur, keypoints, blur, Scalar::all(-1));
		//Show all of the frames.
		//frame.rowRange(Range((int)(frame.rows/2),frame.rows)) = frameBot;
		imshow("hsv",hsv);
        imshow("normal", frameBot);
		imshow("blur", blur);
	//	std::cout<<frame_count<<"\n";
	//	frame_count++;
		//outputVideo << frame;
		//Wait a pre-slected amount of time to keep framerate consistent if possible.
		//if(waitKey(0) >= 0) break;
		waitKey(0);
	}
    return 0;
}
