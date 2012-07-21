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

#ifndef MEAN_SHIFT_TRACKER_H
#define MEAN_SHIFT_TRACKER_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ctype.h>

namespace people {
	bool buildHistogram(const cv::Mat &image, const cv::Rect &rt, bool init, const cv::MatND &hist_in, cv::MatND &hist_out, float alpha);
	float computeSim(const cv::MatND &hist1, const cv::MatND &hist2);

	class MSTarget {
	public:
		MSTarget():count_(0),hratio_(1.0f) {};
		virtual ~MSTarget() {};
		// update color histogram model
		bool 			updateHistogram(cv::Mat &image, cv::Rect rt, float hratio, float alpha);
		// run actual meanshift without scale search
		float			runMeanShift(cv::Mat &image, cv::Rect &out, cv::Rect init = cv::Rect(0, 0, 0, 0));
		// run meanshift with scale search
		float			runScaleMeanShift(cv::Mat &image, cv::Rect &max_out, cv::Rect init = cv::Rect(0, 0, 0, 0));

		// 
		inline cv::Rect getRect() 					{ return rect_; }
		inline int			getTrackedCount() 	{ return count_; }
		// force the MS tracker to be initialized, if sim value is too low..
		// reinitialize the histogram
		inline void			resetModel()				{ count_ = 0; }

		inline float		getHeightRatio() { return hratio_; }
	protected:
		cv::MatND 	hist_;
		cv::Rect		rect_;
		int					count_;
		float			hratio_;
	};
#if 0
	class KernelMeanShift
	{
	public:
		KernelMeanShift();
		virtual ~KernelMeanShift();
	protected:
		cv::Mat runMeanShift(cv::Mat &image_color, std::vector<cv::MatND> &hist_ms, std::vector<cv::Rect> &rect_ms);
	};
#endif
};

#endif
