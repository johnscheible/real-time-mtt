#ifndef _DETECTION_READIN_NODE_H_
#define _DETECTION_READIN_NODE_H_

#include <observation/ObservationNode.h>

namespace people {
	class DetectionReadinConfidence
	{
	public:
		DetectionReadinConfidence() {};
		DetectionReadinConfidence(const DetectionReadinConfidence &src) 
		{
			size_ = src.size_;
			size_ratio_ = src.size_ratio_;
			step_ = src.step_;
			minx_ = src.minx_;
			miny_ = src.miny_;
			map_ = src.map_;
		};

		virtual ~DetectionReadinConfidence() {};

		void showMap();

		float size_;
		float size_ratio_;
		float step_;
		float minx_;
		float miny_;

		cv::Mat map_;
	};

	class DetectionReadinNode : public ObservationNode 
	{
	public:
		DetectionReadinNode();
		virtual ~DetectionReadinNode();

		virtual void setParameter(const std::string &name, const std::string &value);
		virtual void setData(const void *data, const std::string &type);
		virtual double getConfidence(const cv::Rect &rt, double depth = 0);

		virtual void preprocess();
		virtual std::vector<cv::Rect> getDetections();

		void quaryData(const std::string &name, void *data);
	private:
		bool readDetectionResult(const std::string filename);
		void dbgShowConfImage(DetectionReadinConfidence &conf);
		double detectionOberlap(const cv::Rect &rt);
	protected:
		// std::string 					prefix_;
		std::string 					conf_file_;
		double 								time_sec_;
		// detections
		std::vector<cv::Rect> found_;
		std::vector<double> responses_;
		// 
		std::vector<DetectionReadinConfidence> confidences_;
		// parameters
		double hit_threshold_;
		double det_scale_;

		double det_std_x_;
		double det_std_y_;
		double det_std_h_;
		
		cv::Size imsize_;
		double pos_threshold_;
	};
}; // Namespace

#endif
