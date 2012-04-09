#ifndef _OBSERVATION_NODE_H_
#define _OBSERVATION_NODE_H_

#include <opencv/cv.h>
#include <common/util.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/signals2/mutex.hpp>

namespace people {
	class ConfMap
	{
	public:
		ConfMap(){};
		ConfMap(const ConfMap &map)
		{
			scale_ = map.scale_;
			height_ = map.height_;
			confidences_ = map.confidences_;
			pts_ = map.pts_;
			roi_ = map.roi_;
			one_row_size_ = map.one_row_size_;
		}

		float 									height_;
		float 									scale_;
		cv::Rect								roi_;
		// 
		int											one_row_size_;
		std::vector<float> 			confidences_;
		std::vector<cv::Point> 	pts_;
	};

	// not sure what will be the best...
	// think!
	const double obs_out_of_image = -1.5;

	class ObservationNode
	{
	public:
		ObservationNode(){ init(); };
		virtual ~ObservationNode() { release(); };
		
		inline std::string getType() { return node_type_; }

		virtual void setParameter(const std::string &name, const std::string &value) { };
		virtual void setData(const void *data, const std::string &type) {};
		virtual double getConfidence(const cv::Rect &rt, double depth = 0) { return 0.0;};

		virtual void init(){ weight_ = 1.0; };
		virtual void release(){};

		virtual void preprocess(){ /* Do nothing */ };
		virtual std::vector<cv::Rect> getDetections(){ std::vector<cv::Rect> empty; return empty; };
		virtual void quaryData(const std::string &name, void *data) {};

		virtual void setObjType(ObjectType type) {obj_type_ = type;}
	protected:
		std::string 	node_type_;
		double			weight_;
		ObjectType 		obj_type_;
		static boost::signals2::mutex	preprocess_mutex_; // gpu cannot be shared...
	};
}; // Namespace
#endif // _OBSERVATION_NODE_H_
