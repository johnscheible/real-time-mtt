#include <iostream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <observation/DetectionReadinNode.h>
#include <boost/lexical_cast.hpp>
#include <common/util.h>

namespace people {
DetectionReadinNode::DetectionReadinNode()
{
	node_type_ = "detection_readin_node";
	
	conf_file_ = "";
	time_sec_ = 0.0;
	weight_ = 1.0;
	hit_threshold_ = 0.0;
	det_scale_ = 1.0718;

	det_std_x_ = 0.05;
	det_std_y_ = 0.1;
	det_std_h_ = 0.1;

	init();
}

DetectionReadinNode::~DetectionReadinNode()
{
}

void DetectionReadinNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "detection_readin_weight")
		weight_ = boost::lexical_cast<double>(value);
	else if(name == "detection_readin_det_scale")
		det_scale_ = boost::lexical_cast<double>(value);
	else if(name == "detection_readin_threshold")
		hit_threshold_ = boost::lexical_cast<double>(value);
	else if(name == "detection_std_x")
		det_std_x_ = boost::lexical_cast<double>(value);
	else if(name == "detection_std_y")
		det_std_y_ = boost::lexical_cast<double>(value);
	else if(name == "detection_std_h")
		det_std_h_ = boost::lexical_cast<double>(value);
}

void DetectionReadinNode::setData(const void *data, const std::string& type)
{
	if(type == "image_mono") {
		cv::Mat image = *(cv::Mat*)data;
		imsize_.width = image.cols;
		imsize_.height = image.rows;
	}
	else if(type == "time_sec") {
		time_sec_ = *(double*)data;
	}
	else if(type == "detection_readin_conf_file") {
		conf_file_ = *(std::string*)data;
	}
}

void DetectionReadinNode::quaryData(const std::string &name, void *data)
{
	if(name == "detection_readin_detection")
		*((std::vector<cv::Rect>*)data) = found_;
}

void DetectionReadinNode::preprocess()
{
	assert(readDetectionResult(conf_file_));
	// dbgShowConfImage(confidences_[10]);
}

void DetectionReadinNode::dbgShowConfImage(DetectionReadinConfidence &conf)
{
	cv::Mat confidence_image(imsize_.height, imsize_.width, CV_8U);
	confidence_image = cv::Scalar(obs_out_of_image);

	float minval = 100000000.0f;
	float maxval = -10000000.0f;
#if 0
	for(int i = 0; i < conf.map_.rows; i++) {
		for(int j = 0; j < conf.map_.cols; j++) {
			if(minval > conf.map_.at<float>(i, j)) {
				minval = conf.map_.at<float>(i, j);
			}
			if(maxval < conf.map_.at<float>(i, j)) {
				maxval = conf.map_.at<float>(i, j);
			}
		}
	}
#endif
	minval = -5;
	maxval = 5;

	float scale = 255 / (maxval - minval);

	for(int i = 0; i < conf.map_.rows; i++) {
		for(int j = 0; j < conf.map_.cols; j++) {
			float temp = std::max(conf.map_.at<float>(i, j), minval);
			temp = std::min(temp, maxval) - minval;

			unsigned char value = floor(temp * scale);
			int half_step = ceil(conf.step_ / 2);

			for(int di = -half_step; di < half_step; di++) {
				for(int dj = -half_step; dj < half_step; dj++) {
					int x = conf.minx_ + dj + floor(conf.step_ * j);
					int y = conf.miny_ + di + floor(conf.step_ * i);

					if((x >= 0) && (y >= 0) && (x < 640) && (y < 480))
						confidence_image.at<unsigned char>(y, x) = value;
				}
			}
		}
	}
	std::cout << std::endl;
	std::cout << "size : " << conf.size_ << std::endl;
	std::cout << "size_ratio : " << conf.size_ratio_ << std::endl;
	std::cout << "minx : " << conf.minx_ << std::endl;
	std::cout << "miny : " << conf.miny_ << std::endl;
	std::cout << "step : " << conf.step_ << std::endl;
	std::cout << "map size : [width " << conf.map_.cols << ", height " << conf.map_.rows << "]" << std::endl;

	cv::imshow("confidence", confidence_image);
	cv::waitKey();
}

bool DetectionReadinNode::readDetectionResult(const std::string filename)
{
	FILE *fp;
	size_t nread;

	std::cout << "read " << filename << std::endl;
	fp = fopen(filename.c_str(), "r");
	if(fp == NULL) {
		std::cout << "ERROR :Cannot read detection confidence file!" << std::endl;
		fclose(fp);
		return false;
	}
	found_.clear();
#ifdef USE_DET_RESPONSE
	responses_.clear();
#endif
	confidences_.clear();
	
	char header[4];
	nread = fread(header, sizeof(char), 4, fp);
	assert(nread == 4);
	if(!(header[0] == 'C'	&& header[1] == 'O'
		&& header[2] == 'N'	&& header[3] == 'F')) {
		std::cout << "ERROR : invalid header format!" << std::endl;
		fclose(fp);
		return false;
	}

	unsigned int nums;
	// IMPROTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// changed the detection format!!!!!!!!!!!!!!!!!!!!!!!!
	// it include x, y, w, h, th, subtype
	float det[6];

	assert(sizeof(unsigned int) == 4);

	nread = fread(&nums, sizeof(unsigned int), 1, fp);
	assert(nread == 1);
	for(size_t i = 0; i < nums; i++) {
		nread = fread(det, sizeof(float), 6, fp);
		assert(nread == 6);
#ifdef USE_DET_RESPONSE
		if(det[4] > 1.0) {
			responses_.push_back(max((double)det[4], 0.0));
			// responses_.push_back(det[4] + 0.5);
#else
		if(det[4] > .5) {
#endif
			cv::Rect rt(det[0], det[1], det[2], det[3]); 
			
			rt.width = rt.height / WH_RATIO;
			// trick to use nms in opencv
			found_.push_back(rt);
			// found_.push_back(rt);
		}
	}
#ifdef USE_DET_RESPONSE
#if 0
	std::vector<cv::Rect> found2 = found_;
	std::vector<double> resps;

	cv::groupRectangles(found2, 1, 0.2);
	for(size_t i = 0; i < found2.size(); i ++) {
		for(size_t j = 0; j < found_.size(); j++) {

			if(found_[j].x == found2[i].x && 
				found_[j].y == found2[i].y &&
				found_[j].width == found2[i].width &&
				found_[j].height == found2[i].height) {
				
				resps.push_back(max( responses_[j], 0.0 ) );

				break;
			}
			assert(j != found_.size() - 1);
		}
	}
	found_ = found2;
	responses_ = resps;

	std::cout << found_.size() << " " << resps.size() << std::endl; 
	assert(found_.size() == responses_.size());
#endif
#else
	cv::groupRectangles(found_, 1, 0.2);
#endif

	nread = fread(&nums, sizeof(unsigned int), 1, fp);
	assert(nread == 1);
	// float prev_size = 0.0;
	for(size_t i = 0; i < nums; i++) {
		DetectionReadinConfidence conf;
		float *data;
		float map_size[2];	int imap_size[2];

		nread = fread(&conf.size_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(&conf.size_ratio_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(&conf.step_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(&conf.minx_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(&conf.miny_, sizeof(float), 1, fp);
		assert(nread == 1);
		nread = fread(map_size, sizeof(float), 2, fp); // my mistake....should have used integer....
		assert(nread == 2);

		imap_size[0] = round(map_size[0]);
		imap_size[1] = round(map_size[1]);

		data = new float [ imap_size[0] * imap_size[1] ];
		nread = fread(data, sizeof(float), imap_size[0] * imap_size[1], fp);
		
		conf.map_ = cv::Mat(imap_size[0], imap_size[1], CV_32F);
		for(int row = 0; row < imap_size[0]; row++) {
			for(int col = 0; col < imap_size[1]; col++) {
				conf.map_.at<float>(row, col) = data[ row + col * imap_size[0] ] - hit_threshold_;
			}
		}
		delete data;

		confidences_.push_back(conf);
	}
	fclose(fp);

	return true;
}

std::vector<cv::Rect> DetectionReadinNode::getDetections()
{
	return found_;
}

/*
*/
double DetectionReadinNode::getConfidence(const cv::Rect &rt, double depth)
{
	double overlap = 0.0; // detectionOberlap(rt);
#ifdef USE_DET_RESPONSE
	// weight_  = 0.5;
	int idx = 0;
	overlap = getMinDist2Dets(found_, idx, rt, det_std_x_, det_std_y_, det_std_h_);
	if(overlap < 4.0) { // within range
		overlap = (4.0 - overlap) * responses_[idx] / 2;
	}
	else { // too far
		overlap = 0.0;
	}
#else
	overlap = std::max(4.0 - getMinDist2Dets(found_, rt, det_std_x_, det_std_y_, det_std_h_), 0.0) * 1 / 2;
#endif
	// overlap = getDist2AllDets(found_, rt, det_std_x_, det_std_y_, det_std_h_, 4.0) * 1 / 2;
	// overlap = getOverlap2AllDets(found_, rt, 0.4) * 2 / 0.6;
	cv::Point pt(rt.x, rt.y);
	for(size_t i = 0; i < confidences_.size(); i++) {
		if(rt.height >= (confidences_[i].size_ * (1 + 1 / det_scale_) / 2) 
			&& rt.height <= (confidences_[i].size_ * (1 + det_scale_) / 2)) {
			int x = floor((pt.x - confidences_[i].minx_) / confidences_[i].step_);
			int y = floor((pt.y - confidences_[i].miny_) / confidences_[i].step_);

			// check whether point is in image
			if((x < 0) || (y < 0) || (x > confidences_[i].map_.cols) || (y > confidences_[i].map_.rows)) {
				return (overlap + obs_out_of_image) * weight_;
			}
			return (overlap + confidences_[i].map_.at<float>(y, x)) * weight_;
		}
	}
	return (overlap + obs_out_of_image) * weight_;
}
}; // Namespace
