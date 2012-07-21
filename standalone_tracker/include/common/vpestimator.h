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

#ifndef VP_ESTIMATOR_H_
#define VP_ESTIMATOR_H_

#include <iostream>
#include <string>
#include <opencv/cv.h>
#include <common/lines.h>

namespace people {
	class VPEstimator {
	public:
		VPEstimator();
		virtual ~VPEstimator();

		void 		preprocess(cv::Mat &image, double timestamp);

		double 		getHorizonConfidence(int hor);
		bool 		readPreprocessedFile(const std::string &filename);

		cv::Mat 	getVPConfImage(int width);
		cv::Point2f findVanishingPoint(int horizon, float delta = 0.05);

		std::vector<longline> getLinesIntersectingPoint(const cv::Point2f &pt, const float epstheta = M_PI / 18);
		std::vector<longline> getAllLines() { return lines_; }
	protected:
		void detectLines() { assert(0); };
		double getMaxHorizonVote(int idx);
		float getVotingAngle(const longline &line, const cv::Point2f &pt);
	protected:
		double 					weight_;
		std::vector<longline> 	lines_;
		// later, we may not need to precompute this, but we can compute only the confidence for each new sample?
		// with caching (faster)
		double 					votestep_;
		cv::Mat					vmap_;
	};
};

#endif
