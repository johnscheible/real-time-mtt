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

#ifndef _GFEAT_STATE_H_
#define _GFEAT_STATE_H_
#include <common/states.h>

namespace people {
	class StaticFeatureState : public FeatureState
	{
	public:
		StaticFeatureState():x_(0.0),y_(0.0),z_(0.0),FeatureState()
		{
			state_type_ = "feature_static";
		};

		virtual ~StaticFeatureState() {};

		// copy state
		inline virtual void copyTo(FeatureStatePtr dst)
		{
			dst->setElement(0, x_);
			dst->setElement(1, y_);
			dst->setElement(2, z_);

			dst->setTS(timesec_);
			dst->setConfidence(confidence_);
		}
	
		// clone state
		inline virtual FeatureStatePtr clone() 
		{
			FeatureStatePtr ret(new StaticFeatureState);
			copyTo(ret);
			return ret;
		}

		// for debug, show state
		inline virtual void print()
		{
			std::cout << "ts : " << setprecision(6) << timesec_ - floor(timesec_ / 10000) * 10000 << std::endl;
			std::cout << "loc  : [ x : " << x_ << " y : " << y_<< " z : " << z_ << "]" << std::endl;
			std::cout << "conf : " << confidence_ << std::endl;
		}

		// draw a new sample
		virtual FeatureStatePtr drawSample(double timesec, const std::vector<double> &params)
		{
			assert(0);
		}

		// draw a new sample
		virtual FeatureStatePtr perturbState(const std::vector<double> &params, double c = 1.0)
		{
			FeatureStatePtr ret = clone();
			// random sampling
			double temp;
			for(size_t i = 0 ; i < numElement(); i++) {
				temp = ret->getElement(i) + c * g_rng.gaussian(params[i]);
				ret->setElement(i, temp);
			}
			return ret;
		}

		virtual double computeLogPrior(FeatureStatePtr state, double timesec, const std::vector<double> &params) 
		{
			assert(0);
		}

		inline virtual size_t numElement() {	return 3;	}

		inline virtual double getElement(int idx)
		{
			switch(idx) {
				case 0:
				return x_;
				case 1:
				return y_;
				case 2:
				return z_;
				otherwise:
				assert(0);
				break;
			}
			return 0.0;
		}

		inline virtual void setElement(int idx, double val)
		{
			switch(idx) {
				case 0:
				x_ = val;
				break;
				case 1:
				y_ = val;
				break;
				case 2:
				z_ = val;
				break;
				otherwise:
				assert(0);
				break;
			}
		}

		// get cv:::Mat representation
		inline virtual cv::Mat getMatrix() {
			cv::Mat ret(3, 1, CV_64F);
			ret.at<double>(0, 0) = x_;
			ret.at<double>(1, 0) = y_;
			ret.at<double>(2, 0) = z_;
			return ret;
		}

	protected:
		double x_;
		double y_;
		double z_;
	};
};
#endif // _GFEAT_STATE_H_
