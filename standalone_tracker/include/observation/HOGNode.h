#ifndef _HOG_NODE_H_
#define _HOG_NODE_H_

#include <observation/ObservationNode.h>
#include <observation/roihog.h>

namespace people {
	class HOGNode : public ObservationNode 
	{
	public:
		HOGNode();
		virtual ~HOGNode();

		virtual void setParameter(const std::string &name, const std::string &value);
		virtual void setData(const void *data, const std::string &type);
		virtual double getConfidence(const cv::Rect &rt, double depth = 0);

		virtual void preprocess();
		virtual std::vector<cv::Rect> getDetections();

		void quaryData(const std::string &name, void *data);
	protected:
		cv::Mat img_mono_;

		// ROIHOGDetector hog_;
		std::vector<cv::Rect> found_;
		std::vector<cv::Rect> ub_found_;
		// std::vector<cv::Rect> full_found_;
		std::vector<ConfMap> confidences_;

		// model file
		std::string ub_model_file_;
		// parameters
		double hit_threshold_;
		int group_threshold_;
		double det_scale_;
	};
}; // Namespace

#endif
