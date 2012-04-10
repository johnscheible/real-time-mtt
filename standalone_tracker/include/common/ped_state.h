#ifndef _PED_STATE_H_
#define _PED_STATE_H_

#include <iostream>
#include <iomanip>
#include <string.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <common/global.h>
// #include <LinearMath/btTransform.h>

#define NUM_PERSON_SUBTYPE 		1
#define WH_PERSON_RATIO			3
#define MEAN_PERSON_HEIGHT 		1.7
#define STD_PERSON_HEIGHT		0.1

#define NUM_CAR_SUBTYPE		2
#define WH_CAR_RATIO1		0.5
#define WH_CAR_RATIO0		0.75
#define MEAN_CAR_HEIGHT 	1.2
#define STD_CAR_HEIGHT		0.2

namespace people {
	class PeopleStateLoc;
	typedef boost::shared_ptr<PeopleStateLoc> PeopleStateLocPtr;
	
	using namespace std;

	class PeopleStateLoc
	{
	public:
		PeopleStateLoc():x_(0.0),y_(0.0),z_(0.0),timesec_(0),confidence_(0) {};
		virtual ~PeopleStateLoc() {};

		// copy state
		inline virtual void copyTo(PeopleStateLocPtr dst)
		{
			dst->x_ = x_;
			dst->y_ = y_;
			dst->z_ = z_;
			dst->timesec_ = timesec_;
			dst->confidence_ = confidence_;
		}
	
		// clone state
		inline virtual PeopleStateLocPtr clone() 
		{
			PeopleStateLocPtr ret = boost::make_shared<PeopleStateLoc>(PeopleStateLoc());
			copyTo(ret);
			return ret;
		}

		// for debug, show state
		inline virtual void print()
		{
			std::cout << "ts : " << setprecision(6) << timesec_ - floor(timesec_ / 10000) * 10000 << std::endl;
			std::cout << "loc  : [ x : " << x_ << " y : " << y_<< " z : " << z_ << "]" <<std::endl;
			std::cout << "conf : " << confidence_ << std::endl;
		}

		// predict location of the state in timesec
		inline virtual PeopleStateLocPtr predict(double timesec)
		{
			PeopleStateLocPtr ret = clone();
			// if velocity modelled, predict location
			return ret;
		}

		// get Point3d representation
		inline virtual cv::Point3d getPoint() {	return cv::Point3d(x_, y_, z_);	}
		
		// get cv:::Mat representation
		inline virtual cv::Mat getMatrix() {
			cv::Mat ret(3, 1, CV_64F);

			ret.at<double>(0, 0) = x_;
			ret.at<double>(1, 0) = y_;
			ret.at<double>(2, 0) = z_;

			return ret;
		}
#if 0
		inline virtual btVector3 getBtVector3()
		{
			return btVector3(x_, y_, z_);
		}
#endif
		inline double getX() { return x_; }
		inline double getY() { return y_; }
		inline double getZ() { return z_; }
		inline double getTS() { return timesec_; }
		inline double getConfidence() { return confidence_; }

		inline void setX(const double &x) { x_ = x; }
		inline void setY(const double &y) { y_ = y; }
		inline void setZ(const double &z) { z_ = z; }
		inline void setTS(const double &ts) { timesec_ = ts; }
		inline void setConfidence(const double &confidence) { confidence_ = confidence; }
	protected:
		double x_;
		double y_;
		double z_;
		double timesec_;
		double confidence_;
	};
#if 0
	class PeopleStateVel;
	typedef boost::shared_ptr<PeopleStateVel> PeopleStateVelPtr;
	class PeopleStateVel
	{
	public:
		PeopleStateVel():x_(0.0), y_(0.0), z_(0.0),vx_(0.0),vy_(0.0),vz_(0.0),timesec_(0.0),confidence_(0.0){};
		virtual ~PeopleStateVel() {};

		// copy state
		inline virtual void copyTo(PeopleStateVelPtr dst)
		{
			dst->x_ = x_;
			dst->y_ = y_;
			dst->z_ = z_;
			dst->vx_ = vx_;
			dst->vy_ = vy_;
			dst->vz_ = vz_;
			dst->timesec_ = timesec_;
			dst->confidence_ = confidence_;
		}
	
		// clone state
		inline virtual PeopleStateVelPtr clone() 
		{
			PeopleStateVelPtr ret = boost::make_shared<PeopleStateVel>(PeopleStateVel());
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

		inline virtual PeopleStateVelPtr predict(double timesec)
		{
			PeopleStateVelPtr ret = clone();

			ret->x_ = ret->x_ + ret->vx_ * (timesec - timesec_);
			ret->y_ = ret->y_ + ret->vy_ * (timesec - timesec_);
			ret->z_ = ret->z_ + ret->vz_ * (timesec - timesec_);

			ret->timesec_ = timesec;

			return ret;
		}

		inline double getX() { return x_; }
		inline double getY() { return y_; }
		inline double getZ() { return z_; }
		inline double getVX() { return vx_; }
		inline double getVY() { return vy_; }
		inline double getVZ() { return vz_; }
		inline double getTS() { return timesec_; }
		inline double getConfidence() { return confidence_; }

		inline void setX(const double &x) { x_ = x; }
		inline void setY(const double &y) { y_ = y; }
		inline void setZ(const double &z) { z_ = z; }
		inline void setVX(const double &vx) { vx_ = vx; }
		inline void setVY(const double &vy) { vy_ = 0.0; /*vy;*/ } // no velocity on y direction (vertical)
		inline void setVZ(const double &vz) { vz_ = vz; }
		inline void setTS(const double &ts) { timesec_ = ts; }
		inline void setConfidence(const double &confidence) { confidence_ = confidence; }
	protected:
		double x_;
		double y_;
		double z_;
		double vx_;
		double vy_;
		double vz_;
		double timesec_;
		double confidence_;
	};
#else
	class ObjStateVel;
	typedef boost::shared_ptr<ObjStateVel> ObjStateVelPtr;
	class ObjStateVel
	{
	public:
		ObjStateVel():x_(0.0), y_(0.0), z_(0.0),vx_(0.0),vy_(0.0),vz_(0.0),timesec_(0.0),confidence_(0.0),obj_type_(ObjPerson),sub_type_(0){
			obj_type_ = g_objtype;
		};
		
		ObjStateVel(ObjectType type):x_(0.0), y_(0.0), z_(0.0),vx_(0.0),vy_(0.0),vz_(0.0),timesec_(0.0),confidence_(0.0),obj_type_(type),sub_type_(0){ };

		virtual ~ObjStateVel() {};

		// copy state
		inline virtual void copyTo(ObjStateVelPtr dst)
		{
			dst->x_ = x_;
			dst->y_ = y_;
			dst->z_ = z_;
			dst->vx_ = vx_;
			dst->vy_ = vy_;
			dst->vz_ = vz_;
			dst->timesec_ = timesec_;
			dst->confidence_ = confidence_;
		}
	
		// clone state
		inline virtual ObjStateVelPtr clone() 
		{
			ObjStateVelPtr ret = boost::make_shared<ObjStateVel>(ObjStateVel());
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

		inline virtual ObjStateVelPtr predict(double timesec)
		{
			ObjStateVelPtr ret = clone();

			ret->x_ = ret->x_ + ret->vx_ * (timesec - timesec_);
			ret->y_ = ret->y_ + ret->vy_ * (timesec - timesec_);
			ret->z_ = ret->z_ + ret->vz_ * (timesec - timesec_);

			ret->timesec_ = timesec;

			return ret;
		}

		inline double getX() { return x_; }
		inline double getY() { return y_; }
		inline double getZ() { return z_; }
		inline double getVX() { return vx_; }
		inline double getVY() { return vy_; }
		inline double getVZ() { return vz_; }
		inline double getTS() { return timesec_; }
		inline double getConfidence() { return confidence_; }

		inline void setX(const double &x) { x_ = x; }
		inline void setY(const double &y) { y_ = y; }
		inline void setZ(const double &z) { z_ = z; }
		inline void setVX(const double &vx) { vx_ = vx; }
		inline void setVY(const double &vy) { vy_ = 0.0; /*vy;*/ } // no velocity on y direction (vertical)
		inline void setVZ(const double &vz) { vz_ = vz; }
		inline void setTS(const double &ts) { timesec_ = ts; }
		inline void setConfidence(const double &confidence) { confidence_ = confidence; }
		
		void setObjType(ObjectType type) { obj_type_ = type; }
		ObjectType getObjType() { return obj_type_; }

		void setSubType(int type) { sub_type_ = type; }
		int getSubType() { return sub_type_; }
	protected:
		double x_;
		double y_;
		double z_;
		double vx_;
		double vy_;
		double vz_;
		double timesec_;
		double confidence_;

		ObjectType 	obj_type_;
		int			sub_type_;
	};
#endif
#ifdef VEL_STATE
	// typedef PeopleStateVel PeopleState;
	// typedef PeopleStateVelPtr PeopleStatePtr;
	typedef ObjStateVel PeopleState;
	typedef ObjStateVelPtr PeopleStatePtr;
#define get_vel_factor(frames) (4 - (double)min(frames, 6) / 2)
// #define get_vel_factor(frames) 1 
// #define get_vel_factor(frames) (frames == 1) ? 10.0 : 1.0
// #define get_vel_factor(frames) 15.0 / ((double)min(frames, 10) + 5)
#else
	typedef PeopleStateLoc PeopleState;
	typedef PeopleStateLocPtr PeopleStatePtr;
#endif
};

#endif // _PED_STATE_H_
