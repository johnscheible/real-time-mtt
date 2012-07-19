#ifndef _ROIHOG_H_
#define _ROIHOG_H_
#include <opencv/cv.h>
#include <opencv/cvaux.hpp>
#ifdef HAVE_CUDA
#include "opencv2/gpu/gpu.hpp"
#endif

using namespace cv;
using namespace std;

#define HOG_PART_SCORE
#ifdef HOG_PART_SCORE
#define HOG_PART_NUM 4
#endif

#define SCORE_OUTOFIMG	-3.5f
struct DetectionROI
{
	double scale;
	vector<cv::Point> locations;
	vector<double> confidences;
#ifdef HOG_PART_SCORE
	vector<double> part_score[HOG_PART_NUM];
#endif
};
/**/ 

/**/
#ifdef HAVE_CUDA
struct ROIHOGDetector // 
{
public:
	ROIHOGDetector():impl_(cv::Size(96,88)),winSize(impl_.win_size)
	{
	};

	ROIHOGDetector(cv::Size win_size):impl_(win_size),winSize(impl_.win_size)
	{
	};

	~ROIHOGDetector(){};

	void detectMultiScaleROI(const cv::Mat& img, 
							CV_OUT std::vector<cv::Rect>& foundLocations,
							std::vector<DetectionROI>& locations,
							double hitThreshold=0,
							int groupThreshold=2);
	
	void readALTModel(std::string filename);
	
	void setSVMDetector(vector<float> &detector)
	{
		impl_.setSVMDetector(detector);
		detector_=detector;
	};

	vector<float> detector_;
	cv::gpu::HOGDescriptor impl_;
	// aliases
	cv::Size &winSize;
};
#else
class ROIHOGDetector : public cv::HOGDescriptor
{
public:
	ROIHOGDetector(){};
	virtual ~ROIHOGDetector(){};

	CV_WRAP virtual void detectROI(const cv::Mat& img, const vector<cv::Point> &locations, 
											CV_OUT std::vector<cv::Point>& foundLocations, CV_OUT std::vector<double>& confidences, 
											double hitThreshold=0, cv::Size winStride=cv::Size(),
											cv::Size padding=cv::Size()) const;
#ifdef HOG_PART_SCORE
	CV_WRAP void detectROI(const cv::Mat& img, const vector<cv::Point> &locations, 
										CV_OUT std::vector<cv::Point>& foundLocations, CV_OUT std::vector<double>& confidences, CV_OUT std::vector<double> part_scores[], 
										double hitThreshold, cv::Size winStride,
										cv::Size padding) const;
#endif
	CV_WRAP virtual void detectMultiScaleROI(const cv::Mat& img, 
																CV_OUT std::vector<cv::Rect>& foundLocations,
																std::vector<DetectionROI>& locations,
																double hitThreshold=0,
																int groupThreshold=2) const;
	
	void tempCropUpperBodyDetector();
	void showModel();

	void readALTModel(std::string filename);
private:
	void drawOneCell(cv::Mat &img, const float* w, const float minval, const float maxval);
};
#endif
#endif
