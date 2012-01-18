#ifndef _FACE_NODE_H_
#define _FACE_NODE_H_

#include <observation/ObservationNode.h>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/objdetect/objdetect.hpp>

namespace people {
	class FaceNode : public ObservationNode 
	{
	public:
		FaceNode();
		virtual ~FaceNode();

		virtual void init();
		virtual void setParameter(const std::string &name, const std::string &value);
		virtual void setData(const void *data, const std::string &type);
		virtual double getConfidence(const cv::Rect &rt, double depth = 0);

		virtual void preprocess();
		virtual std::vector<cv::Rect> getDetections();

		void quaryData(const std::string &name, void *data);
	protected:
		cv::Mat img_mono_;
		// cv::gpu::CascadeClassifier_GPU impl_;
		cv::CascadeClassifier impl_cpu_;

		std::vector<cv::Rect> found_;
		std::vector<cv::Rect> org_found_;

		std::string face_model_file_;
		// parameters
		int group_threshold_;
	};
}; // Namespace

#endif
