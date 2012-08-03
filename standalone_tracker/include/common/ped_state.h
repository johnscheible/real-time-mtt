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

#ifndef _PED_STATE_H_
#define _PED_STATE_H_
#include <string.h>
#include <common/states.h>

namespace people {
	class ObjectStateLoc: public ObjectState
	{
	public:
		ObjectStateLoc():x_(0.0),y_(0.0),z_(0.0),ObjectState()
		{
			state_type_ = "object_location";
		};

		virtual ~ObjectStateLoc() {};
		// copy state
		inline virtual void copyTo(ObjectStatePtr dst)
		{
			dst->setElement(0, x_);
			dst->setElement(1, y_);
			dst->setElement(2, z_);

			dst->setTS(timesec_);
			dst->setConfidence(confidence_);
		}
	
		// clone state
		inline virtual ObjectStatePtr clone() 
		{
			ObjectStatePtr ret(new ObjectStateLoc);
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

		// predict location of the state in timesec
		inline virtual ObjectStatePtr predict(double timesec)
		{
			ObjectStatePtr ret = clone();
			return ret;
		}

		// draw a new sample
		inline virtual ObjectStatePtr drawSample(double timesec, const std::vector<double> &params)
		{
			ObjectStatePtr ret = clone();

			// random sampling
			double dt = timesec - timesec_; // uncertainty over time
			assert(dt > 0);

			double temp;
			for(size_t i = 0 ; i < 3; i++) {
				temp = ret->getElement(i) + g_rng.gaussian(params[i] * dt);
				ret->setElement(i, temp);
			}
			ret->setTS(timesec);

			return ret;
		}

		inline virtual ObjectStatePtr perturbState(const std::vector<double> &params, double c = 1.0)
		{
			ObjectStatePtr ret = clone();
			// random sampling
			double temp;
			for(size_t i = 0 ; i < 3; i++) {
				temp = ret->getElement(i) + c * g_rng.gaussian(params[i]);
				ret->setElement(i, temp);
			}
			return ret;
		}

		// draw a new sample
		virtual double computeLogPrior(ObjectStatePtr state, double timesec, const std::vector<double> &params)
		{
			double ret = 0, dt = timesec - timesec_;
			assert(dt > 0);
			for(size_t i = 0 ; i < 3; i++) {
				ret += log_gaussian_prob(getElement(i), state->getElement(i), params[i] * dt);
			}
			return ret;
		}

		inline virtual size_t numElement() { return 3; }
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

	class ObjectStateVel: public ObjectState
	{
	public:
		ObjectStateVel():x_(0.0), y_(0.0), z_(0.0),vx_(0.0),vy_(0.0),vz_(0.0),ObjectState()
		{
			obj_type_ = g_objtype;
			state_type_ = "object_location_velocity";
		};

		virtual ~ObjectStateVel() {};

		// copy state
		inline virtual void copyTo(ObjectStatePtr dst)
		{
			dst->setElement(0, x_);
			dst->setElement(1, y_);
			dst->setElement(2, z_);
			dst->setElement(3, vx_);
			dst->setElement(4, vy_);
			dst->setElement(5, vz_);

			dst->setTS(timesec_);
			dst->setConfidence(confidence_);
		}
	
		// clone state
		inline virtual ObjectStatePtr clone() 
		{
			ObjectStatePtr ret(new ObjectStateVel);
			copyTo(ret);
			return ret;
		}

		// for debug, show state
		inline virtual void print()
		{
			std::cout << "ts : " << setprecision(6) << timesec_ - floor(timesec_ / 10000) * 10000 << std::endl;
			std::cout << "loc  : [ x : " << x_ << " y : " << y_<< " z : " << z_ << "]" <<std::endl;
			std::cout << "vel  : [ vx : " << vx_ << " vy : " << vy_<< " vz : " << vz_ << "]" <<std::endl;
			std::cout << "conf : " << confidence_ << std::endl;
		}

		inline virtual ObjectStatePtr predict(double timesec)
		{
			ObjectStatePtr ret = clone();
			double dt = timesec - timesec_;
			assert(dt >= 0);
			for(size_t i = 0 ; i < 3; i++) { // x, y, z
				ret->setElement(i, ret->getElement(i) + ret->getElement(i + 3) * dt);
			}
			// ret->setElement(0, x_ + vx_ * (timesec - timesec_));
			// ret->setElement(1, y_ + vy_ * (timesec - timesec_));
			// ret->setElement(2, z_ + vz_ * (timesec - timesec_));
			ret->setTS(timesec);

			return ret;
		}

		// draw a new sample
		inline virtual ObjectStatePtr drawSample(double timesec, const std::vector<double> &params)
		{
			ObjectStatePtr ret = clone();
			// random sampling
			double dt = timesec - timesec_; // uncertainty over time
			assert(dt > 0);
			double temp;
			for(size_t i = 0 ; i < 6; i++) {
				temp = ret->getElement(i) + g_rng.gaussian(params[i] * dt);
				ret->setElement(i, temp);
			}
			return ret->predict(timesec);
		}

		// draw a new sample
		inline virtual ObjectStatePtr perturbState(const std::vector<double> &params, double c = 1.0)
		{
			ObjectStatePtr ret = clone();
			// random sampling
			double temp;
			for(size_t i = 0 ; i < 6; i++) {
				temp = ret->getElement(i) + c * g_rng.gaussian(params[i]);
				ret->setElement(i, temp);
			}
			return ret;
		}

		// draw a new sample
		virtual double computeLogPrior(ObjectStatePtr state, double timesec, const std::vector<double> &params)
		{
			double ret = 0, dt = timesec - timesec_;
			assert(dt > 0);

			double temp = 0;
			for(size_t i = 0 ; i < 3; i++) { // x, y, z
				// x - dt * vx
				temp = getElement(i) - dt * (getElement(i + 3));
				ret += log_gaussian_prob(temp, state->getElement(i), params[i] * dt);
			}
			for(size_t i = 3 ; i < 6; i++) { // vx, vy, vz
				ret += log_gaussian_prob(getElement(i), state->getElement(i), params[i] * dt);
			}
			return ret;
		}

		inline virtual size_t numElement() { return 6; }
		inline virtual double getElement(int idx)
		{
			switch(idx) {
				case 0:	return x_;			break;
				case 1:	return y_;			break;
				case 2:	return z_;			break;
				case 3:	return vx_;			break;
				case 4:	return vy_;			break;
				case 5:	return vz_;			break;
				otherwise:	assert(0);	break;
			}
			return 0.0;
		}

		inline virtual void setElement(int idx, double val)
		{
			switch(idx) {
				case 0:	x_ = val;	break;
				case 1:	y_ = val;	break;
				case 2:	z_ = val;	break;
				case 3:	vx_ = val;	break;
				case 4:	vy_ = val;	break;
				case 5:	vz_ = val;	break;
				otherwise: assert(0);	break;
			}
		}

		// get cv:::Mat representation
		inline virtual cv::Mat getMatrix() {
			cv::Mat ret(6, 1, CV_64F);

			ret.at<double>(0, 0) = x_;
			ret.at<double>(1, 0) = y_;
			ret.at<double>(2, 0) = z_;
			ret.at<double>(3, 0) = vx_;
			ret.at<double>(4, 0) = vy_;
			ret.at<double>(5, 0) = vz_;

			return ret;
		}
	protected:
		double x_;
		double y_;
		double z_;
		double vx_;
		double vy_;
		double vz_;
	};
};
#endif // _PED_STATE_H_
