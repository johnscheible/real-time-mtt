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

#ifndef _CAM_STATE_H_
#define _CAM_STATE_H_

#include <common/util.h>
#include <common/states.h>

namespace people {
	template <class O, class F>
	class SimplifiedCameraState : public CameraState
	{
	public:
		SimplifiedCameraState():x_(0.0), y_(0.0), z_(0.0), yaw_(0.0), v_(0.0), horizon_(0.0), CameraState() 
		{
			state_type_ = "simplified_camera";
		};
		virtual ~SimplifiedCameraState(){};

		CameraStatePtr clone() 
		{
			CameraStatePtr ret(new SimplifiedCameraState);
		
			ret->setElement(0, f_);
			ret->setElement(1, xcenter_);
			ret->setElement(2, x_);
			ret->setElement(3, y_);
			ret->setElement(4, z_);
			ret->setElement(5, yaw_);
			ret->setElement(6, v_);
			ret->setElement(7, horizon_);
			ret->setTS(timesec_);

			return ret;
		}
		
		inline virtual size_t numElement() {	return 8;	}
		virtual double getElement(int idx)
		{
			switch(idx) {
				case 0:	return f_;	break;
				case 1:	return xcenter_;	break;
				case 2:	return x_;	break;
				case 3:	return y_;	break;
				case 4:	return z_;	break;
				case 5:	return yaw_;	break;
				case 6:	return v_;	break;
				case 7:	return horizon_;	break;
				otherwise: assert(0);	break;
			}
		}

		virtual void setElement(int idx, double val)
		{
			switch(idx) {
				case 0:	f_ = val;	break;
				case 1:	xcenter_ = val;	break;
				case 2:	x_ = val;	break;
				case 3:	y_ = val;	break;
				case 4:	z_ = val;	break;
				case 5:	yaw_ = val;	break;
				case 6:	v_ = val;	break;
				case 7:	horizon_ = val;	break;
				otherwise: assert(0);	break;
			}
		}

		CameraStatePtr predict(double timestamp)
		{
			CameraStatePtr ret = clone();

			double dt = timestamp - timesec_;
			// z direction is the viewing direction!! v * 
 			// transition of the camera
			ret->setElement(2, x_ + v_ * sin(yaw_) * dt);
			ret->setElement(4, z_ + v_ * cos(yaw_) * dt);
			ret->setTS(timestamp);

			return ret;
		}

		// draw a new sample
		virtual CameraStatePtr drawSample(double timesec, const std::vector<double> &params)
		{
			CameraStatePtr ret = clone();
			// random sampling
			double dt = timesec - timesec_; // uncertainty over time
			assert(dt > 0);
			double temp;
			for(size_t i = 0 ; i < numElement(); i++) {
				temp = ret->getElement(i) + g_rng.gaussian(params[i] * dt);
				ret->setElement(i, temp);
			}
			return ret->predict(timesec);
		}

		// draw a new sample
		inline virtual CameraStatePtr perturbState(const std::vector<double> &params, double c = 1.0)
		{
			CameraStatePtr ret = clone();
			// random sampling
			double temp;
			for(size_t i = 0 ; i < numElement(); i++) {
				temp = ret->getElement(i) + c * g_rng.gaussian(params[i]);
				ret->setElement(i, temp);
			}
			return ret;
		}

		// draw a new sample
		virtual double computeLogPrior(CameraStatePtr state, double timesec, const std::vector<double> &params)
		{
			double ret = 0, dt = timesec - timesec_;
			double dx, dz;

			assert(dt > 0);

			dx = -v_ * sin(yaw_) * dt;
			dz = v_ * cos(yaw_) * dt;

			ret = log_gaussian_prob(state->getElement(2) - dx, x_, params[2] * dt);
			ret += log_gaussian_prob(state->getElement(4) - dz, z_, params[4] * dt);
			for(size_t i = 5; i < 8; i++)
				ret += log_gaussian_prob(state->getElement(i), getElement(i), params[i] * dt);

			// avoid negative velocity
			if(state->getElement(6) < 0) ret -= 100;

			return ret;
		}

		cv::Point3f project(FeatureStatePtr state)
		{
			cv::Mat iR(2, 2, CV_32FC1, cv::Scalar(0.0f));
			cv::Mat loc(2, 1, CV_32FC1); 
			cv::Rect rt;
			///////////////////////////////////////////////////////////////
			// Transform to camera coordinate system
			// set rotation matrix
			iR.at<float>(0,0) = cos(yaw_);
			iR.at<float>(0,1) = sin(yaw_);
			iR.at<float>(1,0) = -sin(yaw_);
			iR.at<float>(1,1) = cos(yaw_);
			// set XZ location in camera system
			loc.at<float>(0,0) = state->getElement(0) - x_;
			loc.at<float>(1,0) = state->getElement(2) - z_;
			// rotate, and it will be located in camera reference system
			loc = iR * loc;
			///////////////////////////////////////////////////////////////
			
			///////////////////////////////////////////////////////////////////
			// project onto image
			cv::Point3f ret;
			// focal / z * x + xcenter_
			ret.x = f_ / loc.at<float>(1, 0) * loc.at<float>(0,0) + xcenter_;
			// focal / z * cam_height + horizon
			ret.y = f_ / loc.at<float>(1, 0) * y_ + horizon_;
			///////////////////////////////////////////////////////////////////

			return ret;
		};

		FeatureStatePtr iproject(const cv::Point2f &pt, double depth = 0)
		{
			// FeatureStatePtr state = createFeatureState();
			FeatureStatePtr state(new F);

			cv::Mat R(2, 2, CV_32FC1, cv::Scalar(0.0f));
			cv::Mat loc(2, 1, CV_32FC1); 
			// no information about the velocity
			//////////////////////////////////////////////////////////////////
			// location in camera coordinate
			loc.at<float>(1, 0) = f_ * y_ / (pt.y - horizon_);
			loc.at<float>(0, 0) = (pt.x - xcenter_) * loc.at<float>(1, 0) / f_;
			////////////////////////////////////////////////////////////////////
			// set rotation matrix get location in world
			R.at<float>(0,0) = cos(yaw_);
			R.at<float>(0,1) = -sin(yaw_);
			R.at<float>(1,0) = sin(yaw_);
			R.at<float>(1,1) = cos(yaw_);
			loc = R * loc;

			state->setElement(0, loc.at<float>(0, 0) + x_);
			state->setElement(2, loc.at<float>(1, 0) + z_);
			state->setElement(1, 0.0);
			////////////////////////////////////////////////////////////////////
			return state;
		}
		
		cv::Rect project(ObjectStatePtr state)
		{
			cv::Mat iR(2, 2, CV_32FC1, cv::Scalar(0.0f));
			cv::Mat loc(2, 1, CV_32FC1); 
			cv::Rect rt;

			///////////////////////////////////////////////////////////////
			// Transform to camera coordinate system
			// set rotation matrix
			iR.at<float>(0,0) = cos(yaw_);
			iR.at<float>(0,1) = sin(yaw_);
			iR.at<float>(1,0) = -sin(yaw_);
			iR.at<float>(1,1) = cos(yaw_);
			// set XZ location in camera system
			loc.at<float>(0,0) = state->getElement(0) - x_;
			loc.at<float>(1,0) = state->getElement(2) - z_;
			// rotate, and it will be located in camera reference system
			loc = iR * loc;
			///////////////////////////////////////////////////////////////
			
			///////////////////////////////////////////////////////////////////
			// project onto image
			double feetx, feety;
			// focal / z * x + xcenter_
			feetx = f_ / loc.at<float>(1, 0) * loc.at<float>(0,0) + xcenter_;
			// focal / z * cam_height + horizon
			feety = f_ / loc.at<float>(1, 0) * y_ + horizon_;
			// height in image
			rt.height = f_ / loc.at<float>(1, 0) * state->getElement(1);
			///////////////////////////////////////////////////////////////////
			if(state->getObjType() == ObjPerson) {
				rt.width = rt.height / WH_PERSON_RATIO;
			}
			else if(state->getObjType() == ObjCar) {
				if(state->getSubType() == 0) {
					rt.width = rt.height / WH_CAR_RATIO0;
				}
				else if(state->getSubType() == 1) {
					rt.width = rt.height / WH_CAR_RATIO1;
				}
				else {
					state->print();
					printf("%d\n", state->getSubType() );
					assert(0);
				}
			}
			else {
				assert(0);
			}
			rt.x = feetx - rt.width / 2;
			rt.y = feety - rt.height;
			///////////////////////////////////////////////////////////////////

			return rt;
		};

		ObjectStatePtr iproject(const cv::Rect &rt)
		{
			// ObjectStatePtr state = createObjectState();
			ObjectStatePtr state(new O);

			cv::Mat R(2, 2, CV_32FC1, cv::Scalar(0.0f));
			cv::Mat loc(2, 1, CV_32FC1); 
			//////////////////////////////////////////////////////////////////
			double feetx, feety;
			feetx = rt.x + rt.width / 2;
			feety = rt.y + rt.height;
			////////////////////////////////////////////////////////////////////
			// location in camera coordinate
			loc.at<float>(1, 0) = f_ * y_ / (feety - horizon_);
			loc.at<float>(0, 0) = (feetx - xcenter_) * loc.at<float>(1, 0) / f_;
			state->setElement(1, rt.height * loc.at<float>(1, 0) / f_);
			////////////////////////////////////////////////////////////////////
			// set rotation matrix get location in world
			R.at<float>(0,0) = cos(yaw_);
			R.at<float>(0,1) = -sin(yaw_);
			R.at<float>(1,0) = sin(yaw_);
			R.at<float>(1,1) = cos(yaw_);
			loc = R * loc;

			state->setElement(0, loc.at<float>(0, 0) + x_);
			state->setElement(2, loc.at<float>(1, 0) + z_);

			return state;
		}

		void print()
		{
			std::cout << setprecision(5) << "camera at " << timesec_ << " : " 
						<< " x " << x_
						<< " y " << y_
						<< " z " << z_
						<< " yaw " << yaw_
						<< " velocity " << v_
						<< " horizon " << horizon_ 
						<< " f " << f_ 
						<< " xcenter " << xcenter_ 
						<< std::endl;
		};

		inline void set(const double &x, const double &y, const double &z, const double &v, const double &yaw, const double &hor, const double &f, const double &xc, const double &ts) {x_ = x; y_ = y; z_ = z; v_ = v; yaw_ = yaw; horizon_ = hor; f_ = f; xcenter_ = xc; timesec_ = ts;};
	protected:
		//
		double x_;				// x location
		double z_;				// z location
		double yaw_;			// yaw angle
		double v_;				// velocity of the camera
		double horizon_;	// horizon position

		// not being estimated
		double y_; 				// camera height
		double f_; 				// focal length
		double xcenter_; 	// image x center
	};
};
#endif // _CAM_STATE_H_
