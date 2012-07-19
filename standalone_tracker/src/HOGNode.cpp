#include <iostream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <observation/HOGNode.h>
#include <boost/lexical_cast.hpp>

namespace people {
boost::signals2::mutex	ObservationNode::preprocess_mutex_; // gpu cannot be shared...

HOGNode::HOGNode() // :gpu_mutex_(boost::interprocess::open_or_create_t(), "gpu_hog")
{
	node_type_ = "hog_ub_detector";
	ub_model_file_ = "";

	weight_ = 1.0;
	hit_threshold_ = 0.0;
	group_threshold_ = 2;
	det_scale_ = 1.05;
#if 0
	assert(hog_.winSize.width == 96);
	assert(hog_.winSize.height == 88); 
	hog_.readALTModel(std::string("/u/wchoi/ros/people_experimental/ped_tracker/data/model_4BiSVMLight.H90.alt"));
#endif
	init();
}

HOGNode::~HOGNode()
{
}

void HOGNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "hog_weight")
		weight_ = boost::lexical_cast<double>(value) / 1.5;
	if(name == "hog_hit_threshold")
		hit_threshold_ = boost::lexical_cast<double>(value) - 0.5;
	if(name == "hog_group_threshold")
		group_threshold_ = boost::lexical_cast<int>(value);
	if(name == "hog_det_scale")
		det_scale_ = boost::lexical_cast<double>(value);
	if(name == "hog_ub_model_file")
		ub_model_file_ = value;
}

void HOGNode::setData(const void *data, const std::string& type)
{
	if(type == "image_mono")
		img_mono_ = *(cv::Mat*)data;
}

void HOGNode::quaryData(const std::string &name, void *data)
{
	if(name == "hog_ub_detection")
		*((std::vector<cv::Rect>*)data) = ub_found_;
	else if(name == "hog_detection")
		*((std::vector<cv::Rect>*)data) = found_;
}

void HOGNode::preprocess()
{
	std::vector<DetectionROI> location;
	DetectionROI det_roi;
	int min_x, min_y;
	int max_x, max_y;
	int stride_size = 8 ;

	preprocess_mutex_.lock();
	/////////////////////////////////////
	{
		found_.clear();
		confidences_.clear();

		// pass 1 detect upper bodies
		ROIHOGDetector hog_ub;
		assert(hog_ub.winSize.width == 96);
		assert(hog_ub.winSize.height == 88); 
		// hog_ub.readALTModel(std::string("/u/wchoi/ros/people_experimental/ped_tracker/data/model_4BiSVMLight.H90.alt"));
		if(ub_model_file_ == "") {
			std::cout << "HOG Upper body model file is not set!! set parameter hog_ub_model_file to HOGNode" << std::endl;
			exit(1);
		}
		hog_ub.readALTModel(ub_model_file_);

		location.clear();
		
		for(double scale = 1.0; scale < 5.0; scale *= det_scale_) {
			det_roi.scale = scale; // 100 by 90
			min_x = 0 - 8;
			min_y = 0;
			max_x = (int)floor((float)img_mono_.cols / scale) - hog_ub.winSize.width + 8;
			max_y = (int)floor((float)img_mono_.rows / scale) - hog_ub.winSize.height + 8;

			for(int j = min_y; j < max_y; j += 8)
				for(int i = min_x; i < max_x; i += 8)
					det_roi.locations.push_back(cv::Point(i, j));

			location.push_back(det_roi);
			det_roi.locations.clear();
		}

		hog_ub.detectMultiScaleROI(img_mono_, ub_found_, location, hit_threshold_, 2);
		for(size_t i = 0; i < location.size(); i++) 
		{
			ConfMap map;
			map.scale_ = location[i].scale;
			map.height_ = hog_ub.winSize.height * location[i].scale;

			map.one_row_size_ = floor(floor((img_mono_.cols) / map.scale_ - hog_ub.winSize.width) / stride_size ) + 1;
			CV_Assert(location[i].locations[map.one_row_size_ - 1].y == location[i].locations[map.one_row_size_].y - stride_size);

			for(size_t j = 0; j < location[i].confidences.size(); j++) {
				cv::Point pt = location[i].locations[j];
				map.confidences_.push_back(location[i].confidences[j] - hit_threshold_);
				map.pts_.push_back(pt);
			}
			
			confidences_.push_back(map);
			// std::cout << "ub scale " << map.scale_ << " height " << map.height_ <<  std::endl;
		}
		found_ = ub_found_;
	}
	preprocess_mutex_.unlock();
#if 0
	// pass two detect full body
	// not the best way!
	ROIHOGDetector hog_full(cv::Size(64, 128));
	assert(hog_full.winSize.width == 64);
	assert(hog_full.winSize.height == 128); 
	std::vector<float> default_detector = cv::HOGDescriptor::getDefaultPeopleDetector();
	hog_full.setSVMDetector(default_detector);

	// HACK!! 
	// adjust aspect ratio of the bounding box to upper body compatible
	const float ub_full_ratio = (hog_ub.winSize.width * hog_full.winSize.height) / (hog_full.winSize.width * hog_ub.winSize.height);
	location.clear();
	// min size of ub detector
	double max_size = ( hog_ub.winSize.height * ub_full_ratio ) / ( hog_full.winSize.height );
	for(double scale = 1.0; scale < max_size; scale *= det_scale_) {
		det_roi.scale = scale; // 64 by 128
		min_x = 0 - 8;
		min_y = 0;
		max_x = (int)floor((float)img_mono_.cols / scale) - hog_full.winSize.width + 8;
		max_y = (int)floor((float)img_mono_.rows / scale) - hog_full.winSize.height + 8;

		for(int j = min_y; j < max_y; j += 8)
			for(int i = min_x; i < max_x; i += 8)
				det_roi.locations.push_back(cv::Point(i, j));

		location.push_back(det_roi);
		det_roi.locations.clear();
	}
	hog_full.detectMultiScaleROI(img_mono_, full_found_, location, hit_threshold_, 0);
	
	for(size_t i = 0; i < location.size(); i++) 
	{
		ConfMap map;
		map.scale_ = ( location[i].scale * hog_full.winSize.height ) / ( hog_ub.winSize.height * ub_full_ratio );
		map.height_ = hog_ub.winSize.height * map.scale_; // hog_ub.winSize.height * location[i].scale;

		map.one_row_size_ = floor(floor((img_mono_.cols) / location[i].scale - hog_full.winSize.width) / stride_size ) + 1;
		CV_Assert(location[i].locations[map.one_row_size_ - 1].y == location[i].locations[map.one_row_size_].y - stride_size);
		for(size_t j = 0; j < location[i].confidences.size(); j++) {
			cv::Point pt = location[i].locations[j];
			map.confidences_.push_back(location[i].confidences[j] - hit_threshold_);
			map.pts_.push_back(pt);
		}
		confidences_.push_back(map);
		// std::cout << "full scale " << map.scale_ << " height " << map.height_ << " org hieght " <<  location[i].scale * hog_full.winSize.height << std::endl;
	}
	// found_ = full_found;
	for(size_t i = 0; i < full_found_.size(); i++)
	{
		cv::Rect rt = full_found_[i];
		rt.y += rt.height / 8;
		rt.height /= ub_full_ratio;
		found_.push_back(rt);
	}
	// group all of the bbs
	cv::groupRectangles(found_, group_threshold_, 0.2);
#endif

#if 0
	{
		cv::Mat temp_image = img_mono_.clone();
		for(size_t i = 0; i < ub_found.size(); i++)
			cv::rectangle(temp_image, ub_found[i].tl(), ub_found[i].br(), cv::Scalar(0, 0, 0), 3);
		for(size_t i = 0; i < full_found.size(); i++)
			cv::rectangle(temp_image, full_found[i].tl(), ub_found[i].br(), cv::Scalar(0, 0, 0), 3);
		cv::imshow("hogs", temp_image);
		cv::waitKey(10);
	}
#endif
}

std::vector<cv::Rect> HOGNode::getDetections()
{
	return found_;
}

double HOGNode::getConfidence(const cv::Rect &rt, double depth)
{
	cv::Point pt(rt.x, rt.y);
	int stride_size = 8 ;

	double ret = std::max(4.0 - getMinDist2Dets(found_, rt, 0.15, 0.2, 0.2), 0.0) / 1;
	// minimum size
	if(rt.height < 88)
		return ret;

	// printf("confidences count %d : searching for [%d,%d,%d,%d]\n", confidences_.size(),rt.x, rt.y, rt.width, rt.height);
	for(size_t i = 0; i < confidences_.size(); i++) {
		// printf("confidence%d height %d\n", i, (int)confidences_[i].height_);
		if(rt.height >= (confidences_[i].height_ * (1 + 1 / det_scale_) / 2) 
			&& rt.height <= (confidences_[i].height_ * (1 + det_scale_) / 2)) {
			// printf("confidence size %d\n", (int)confidences_[i].height_);
			int xidx = floor((float)(rt.x / confidences_[i].scale_ + stride_size / 2) / stride_size);
			int yidx = floor((float)(rt.y / confidences_[i].scale_ + stride_size / 2) / stride_size);
#if 0
			if(confidences_[i].scale_ < 1.0)
			{
				assert(confidences_[i].scale_ >= 1.0);
				// image padding in full body detector;;
				yidx -= 2;
			}
#endif
			if((xidx < 0) || (yidx < 0) || (xidx >= confidences_[i].one_row_size_)) break;
			int ptidx = yidx * confidences_[i].one_row_size_ + xidx;
			if(ptidx >= (int)confidences_[i].confidences_.size()) break;

			CV_Assert(abs(rt.x / confidences_[i].scale_- confidences_[i].pts_[ptidx].x) <= 4);
			if(confidences_[i].scale_ >= 1.0) {
				CV_Assert(abs(rt.y / confidences_[i].scale_- confidences_[i].pts_[ptidx].y) <= 4);
			}

			return (ret + confidences_[i].confidences_[ptidx]) * weight_;
		}
	}

	return obs_out_of_image * weight_;
}

}; // Namespace
