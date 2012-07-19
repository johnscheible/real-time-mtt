#ifndef _SKIN_NODE_H_
#define _SKIN_NODE_H_

#include <observation/ObservationNode.h>

namespace people {
	class SkinNode : public ObservationNode 
	{
	public:
		SkinNode();
		virtual ~SkinNode();

		virtual void setParameter(const std::string &name, const std::string &value);
		virtual void setData(const void *data, const std::string &type);
		virtual double getConfidence(const cv::Rect &rt, double depth = 0);

		virtual void preprocess();

		cv::Mat getSkin(){return img_skin_;}
	protected:
		cv::Mat img_hsv_;
		cv::Mat img_depth_;
		cv::Mat img_skin_;

		unsigned char hval_lb_, hval_ub_;
		unsigned char sval_lb_, sval_ub_;
		unsigned char vval_lb_, vval_ub_;
		float dth_;
	};
}; // Namespace

#endif
