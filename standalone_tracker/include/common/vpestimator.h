#ifndef VP_ESTIMATOR_H_
#define VP_ESTIMATOR_H_

#include <iostream>
#include <string>
#include <opencv/cv.h>

namespace people {
	typedef struct _longline {
		float x1_;
		float x2_;
		float y1_;
		float y2_;
		float angle_;
		float length_;
	}longline;
	
	void print_line(const longline &line);

	class VPEstimator {
	public:
		VPEstimator();
		virtual ~VPEstimator();

		void 		preprocess(cv::Mat &image, double timestamp);

		double 		getHorizonConfidence(int hor);
		bool 		readPreprocessedFile(const std::string &filename);

		cv::Mat 	getVPConfImage(int width);
		cv::Point2f findVanishingPoint(int horizon, float delta = 0.05);

		std::vector<longline> getLinesIntersectingPoint(const cv::Point2f &pt, const float epstheta = M_PI / 18);
		std::vector<longline> getAllLines() { return lines_; }
	protected:
		void detectLines() { assert(0); };
		double getMaxHorizonVote(int idx);
		float getVotingAngle(const longline &line, const cv::Point2f &pt);
	protected:
		double 					weight_;
		std::vector<longline> 	lines_;
		// later, we may not need to precompute this, but we can compute only the confidence for each new sample?
		// with caching (faster)
		double 					votestep_;
		cv::Mat					vmap_;
	};
};

#endif
