/*
 * Software License Agreement (BSD License)
 * 
 * Copyright (c)  2012, Wongun Choi
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies, 
 * either expressed or implied, of the FreeBSD Project.
 */

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
