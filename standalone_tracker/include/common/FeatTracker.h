#ifndef FEAT_TRACKER_H_
#define FEAT_TRACKER_H_

#include <iostream>
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

namespace people {
	void draw_key_points(cv::Mat &img, std::vector<cv::KeyPoint> &points);
	
	typedef struct _imfeat {
		cv::Point2f		pt_;
		double			timestamp_;
	}imfeat;
#if 1
	typedef struct _imfeat_track {
		std::vector<imfeat> feats_;
		float 	response_;
	}imfeat_track;
#else
	typedef std::vector<imfeat> imfeat_track;
#endif
	class FeatTracker
	{
	public:
		FeatTracker();
		virtual ~FeatTracker();
		
		void setVideoFile(const std::string &video_file);
		void showImage(const bool bshow) { show_image_ = bshow; }

		void setDetectorType(const std::string &type);
		void setNewImage(cv::Mat &image, double timestamp);
		void processTracking();

		void get_features(double timestamp, std::vector<cv::Point2f> &pts, std::vector<float> &responses, std::vector<int> &index);
	protected:
		bool isValidGroundFeature(cv::Point2f pt);
		std::vector<cv::KeyPoint> detect_features(cv::Mat &image);
	protected:
		cv::Mat prev_image_;
		cv::Mat current_image_;

		double prev_timestamp_;
		double current_timestamp_;

		int max_features_;
		float max_flow_dist_;
		bool show_image_;

		std::string detector_type_;
		
		std::vector<imfeat_track> feat_tracks_;

		cv::VideoWriter 	video_;
	};
};
#endif
