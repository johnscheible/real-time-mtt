#ifndef _CAM_STATE_H_
#define _CAM_STATE_H_

#include <common/ped_state.h>
#include <common/gfeat_state.h>
// #include <LinearMath/btTransform.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define WH_RATIO	2.3

namespace people {
	class CamState;
	typedef boost::shared_ptr<CamState> CamStatePtr;

	class CamState
	{
	public:
		CamState():x_(0.0), y_(0.0), z_(0.0), yaw_(0.0), v_(0.0), horizon_(0.0), timesec_(0.0) {};
		virtual ~CamState(){};
#if 0
		btMatrix3x3 getRot() 
		{
			btMatrix3x3 ret;
			// ret.setRPY(roll_, pitch_, yaw_);
			return ret;
		};

		btVector3 getTrans() 
		{
			btVector3 ret(x_, y_, z_);
			return ret;
		};
#endif
		inline virtual CamStatePtr clone() 
		{
			CamStatePtr ret = boost::make_shared<CamState>(CamState());
			ret->x_ = x_; 	
			ret->z_ = z_;
			ret->yaw_ = yaw_;	
			ret->v_ = v_;	
			ret->horizon_ = horizon_;

			ret->y_ = y_; 	
			ret->f_ = f_; 	
			ret->xcenter_ = xcenter_; 	

			ret->timesec_ = timesec_;

			return ret;
		}
		
		CamStatePtr predict(double timestamp)
		{
			CamStatePtr ret = clone();
			double dt = timestamp - timesec_;
			// z direction is the viewing direction!! v * 
 			// transition of the camera
			ret->x_ += -ret->v_ * sin(ret->yaw_) * dt;
			ret->z_ += ret->v_ * cos(ret->yaw_) * dt;
			ret->timesec_ = timestamp;

			return ret;
		}

		cv::Point2f project(GFeatStatePtr state)
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
			loc.at<float>(0,0) = state->getX() - x_;
			loc.at<float>(1,0) = state->getZ() - z_;
			// rotate, and it will be located in camera reference system
			loc = iR * loc;
			///////////////////////////////////////////////////////////////
			
			///////////////////////////////////////////////////////////////////
			// project onto image
			cv::Point2f ret;
			// focal / z * x + xcenter_
			ret.x = f_ / loc.at<float>(1, 0) * loc.at<float>(0,0) + xcenter_;
			// focal / z * cam_height + horizon
			ret.y = f_ / loc.at<float>(1, 0) * y_ + horizon_;
			///////////////////////////////////////////////////////////////////

			return ret;
		};

		GFeatStatePtr iproject(const cv::Point2f &pt)
		{
			GFeatStatePtr state = boost::make_shared<GFeatState>();
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

			state->setX(loc.at<float>(0, 0) + x_);
			state->setZ(loc.at<float>(1, 0) + z_);
			state->setY(0.0);
			////////////////////////////////////////////////////////////////////
#if 0
			std::cout << pt.x << " " << pt.y << std::endl;
			state->print();
			print();
			cv::waitKey();
#endif
			return state;
		}
		
		cv::Rect project(PeopleStatePtr state)
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
			loc.at<float>(0,0) = state->getX() - x_;
			loc.at<float>(1,0) = state->getZ() - z_;
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
			rt.height = f_ / loc.at<float>(1, 0) * state->getY();
			///////////////////////////////////////////////////////////////////
			rt.width = rt.height / WH_RATIO;
			rt.x = feetx - rt.width / 2;
			rt.y = feety - rt.height;
			///////////////////////////////////////////////////////////////////

			return rt;
		};

		PeopleStatePtr iproject(const cv::Rect &rt)
		{
			PeopleStatePtr state = boost::make_shared<PeopleState>();
			cv::Mat R(2, 2, CV_32FC1, cv::Scalar(0.0f));
			cv::Mat loc(2, 1, CV_32FC1); 
			
			// no information about the velocity
			state->setVX(0.0);
			state->setVY(0.0);
			state->setVZ(0.0);
			//////////////////////////////////////////////////////////////////
			double feetx, feety;
			feetx = rt.x + rt.width / 2;
			feety = rt.y + rt.height;
			////////////////////////////////////////////////////////////////////
			// location in camera coordinate
			loc.at<float>(1, 0) = f_ * y_ / (feety - horizon_);
			loc.at<float>(0, 0) = (feetx - xcenter_) * loc.at<float>(1, 0) / f_;
			state->setY(rt.height * loc.at<float>(1, 0) / f_);
			////////////////////////////////////////////////////////////////////
			// set rotation matrix get location in world
			R.at<float>(0,0) = cos(yaw_);
			R.at<float>(0,1) = -sin(yaw_);
			R.at<float>(1,0) = sin(yaw_);
			R.at<float>(1,1) = cos(yaw_);
			loc = R * loc;

			state->setX(loc.at<float>(0, 0) + x_);
			state->setZ(loc.at<float>(1, 0) + z_);
			////////////////////////////////////////////////////////////////////
#if 0
			std::cout << rt.x << " " << rt.y << " " << rt.width << " " << rt.height << std::endl;
			state->print();
			print();
			cv::waitKey();
#endif
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

		inline double getX() { return x_; }
		inline double getY() { return y_; }
		inline double getZ() { return z_; }
		inline double getV() { return v_; }
		inline double getYaw() { return yaw_; }
		inline double getHorizon() { return horizon_; }
		inline double getFocal() { return f_; }
		inline double getXcenter() { return xcenter_; }
		inline double getTS() { return timesec_; }

		inline void setX(const double &x) { x_ = x; }
		inline void setY(const double &y) { y_ = y; }
		inline void setZ(const double &z) { z_ = z; }
		inline void setV(const double &v) { v_ = v; }
		inline void setYaw(const double &yaw) { yaw_ = yaw; }
		inline void setHorizon(const double &hor) { horizon_ = hor; }
		inline void setFocal(const double &f) { f_ = f; }
		inline void setXcenter(const double &xc) { xcenter_ = xc; }
		inline void setTS(const double &ts) { timesec_ = ts; }

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

		double timesec_;
	};
};
#endif // _CAM_STATE_H_
