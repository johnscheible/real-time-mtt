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

#ifndef FEAT_TRACKER_H_
#define FEAT_TRACKER_H_

#include <iostream>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/nonfree/nonfree.hpp>

namespace people {
	void draw_key_points(cv::Mat &img, std::vector<cv::KeyPoint> &points);
	
	typedef struct _imfeat {
		cv::Point2f		pt_;
		double			timestamp_;
	}imfeat;
#if 1
	typedef struct _imfeat_track {
		std::vector<imfeat> feats_;
		float 	response_;
	}imfeat_track;
#else
	typedef std::vector<imfeat> imfeat_track;
#endif
	class FeatTracker
	{
	public:
		FeatTracker();
		virtual ~FeatTracker();
		
		void setVideoFile(const std::string &video_file);
		void showImage(const bool bshow) { show_image_ = bshow; }

		void setDetectorType(const std::string &type);
		void setNewImage(cv::Mat &image, double timestamp);
		void processTracking();

		void get_features(double timestamp, std::vector<cv::Point2f> &pts, std::vector<float> &responses, std::vector<int> &index);
	protected:
		bool isValidGroundFeature(cv::Point2f pt);
		std::vector<cv::KeyPoint> detect_features(cv::Mat &image);
	protected:
		cv::Mat prev_image_;
		cv::Mat current_image_;

		double prev_timestamp_;
		double current_timestamp_;

		int max_features_;
		float max_flow_dist_;
		bool show_image_;

		std::string detector_type_;
		
		std::vector<imfeat_track> feat_tracks_;

		cv::VideoWriter 	video_;
	};
};
#endif
