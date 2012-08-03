/*
 * Software License Agreement (BSD License)
 * 
 * Copyright (c)  2012, Wongun Choi
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met: 
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer. 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution. 
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies, 
 * either expressed or implied, of the FreeBSD Project.
 */

#include <iostream>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <observation/DetectionReadinNode.h>
#include <boost/lexical_cast.hpp>
#include <common/util.h>

namespace people {

void DetectionReadinConfidence::showMap()
{
	cv::Mat image(map_.rows, map_.cols, CV_8U);
	
	std::cout << " size_ " << size_ ;
	std::cout << " size_ratio_ " << size_ratio_ ;
	std::cout << " step_ " << step_ ;
	std::cout << " minx_ " << minx_ ;
	std::cout << " miny_ " << miny_ ;
	std::cout << std::endl;

	float minval = -3.0; 	
	float maxval = 0.0;
	float scale = 255 / (maxval - minval);
	for(int i = 0; i < map_.rows; i++) {
		for(int j = 0; j < map_.cols; j++) {
			float temp = std::max(map_.at<float>(i, j), minval);
			temp = std::min(temp, maxval) - minval;
			unsigned char value = floor(temp * scale);
			image.at<unsigned char>(i, j) = value;
		}
	}
	cv::imshow("conf", image);
	cv::waitKey(20);
}

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
	//
	obj_type_ = g_objtype; // ObjPerson;
	if(obj_type_ == ObjPerson)		pos_threshold_ = 0.0;
	else if(obj_type_ == ObjCar)	pos_threshold_ = -0.85;
	else							assert(0);

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
}

void DetectionReadinNode::dbgShowConfImage(DetectionReadinConfidence &conf)
{
	cv::Mat confidence_image(imsize_.height, imsize_.width, CV_8U);
	confidence_image = cv::Scalar(obs_out_of_image);

	float minval = 100000000.0f;
	float maxval = -10000000.0f;
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
	if(fp < 0) {
		std::cout << "ERROR :Cannot read detection confidence file!" << std::endl;
		exit(1);
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

	// int version = 0;
	if((header[0] == 'C'	&& header[1] == 'O'
		&& header[2] == 'N'	&& header[3] == 'F')) {
		version_ = 1;
	}
	else if((header[0] == 'C'    && header[1] == 'O'
		     && header[2] == 'N' && header[3] == '2')) {
		version_ = 2;
	}
	else if((header[0] == 'C'    && header[1] == 'O'
		     && header[2] == 'N' && header[3] == '3')) {
		version_ = 3;
	}
	else {
		std::cout << "ERROR : invalid header format!" << std::endl;
		fclose(fp);
		return false;
	}

	unsigned int nums;
	float det[6];
	assert(sizeof(unsigned int) == 4);

	nread = fread(&nums, sizeof(unsigned int), 1, fp);
	assert(nread == 1);
	for(size_t i = 0; i < nums; i++) {
		if(version_ == 1) {
			// it include x, y, w, h, th
			nread = fread(det, sizeof(float), 5, fp);
			det[5] = 1;
			assert(nread == 5);
		}
		else if(version_ == 2 || version_ == 3) {
			// IMPROTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			// changed the detection format!!!!!!!!!!!!!!!!!!!!!!!!
			// it include x, y, w, h, th, subtype
			nread = fread(det, sizeof(float), 6, fp);
			assert(nread == 6);
		}
		else {
			assert(0);
		}

#ifdef USE_DET_RESPONSE
		if( det[4] > pos_threshold_ ) {
			responses_.push_back(max((double)det[4] - pos_threshold_, 0.0));
			// responses_.push_back(det[4] + 0.5);
#else
		if( det[4] > .5 ) {
#endif
			cv::Rect rt(det[0], det[1], det[2], det[3]); 

			if(version_ < 3){ 
				if(obj_type_ == ObjPerson) {
					rt.width = rt.height / WH_PERSON_RATIO;
				}
				else if(obj_type_ == ObjCar) {
					if(det[5] == 1) {
						rt.width = rt.height / WH_CAR_RATIO1;
					}
					else if(det[5] == 2) {
						rt.width = rt.height / WH_CAR_RATIO0;
					}
					else {
						// only two type defined now..
						assert(0);
					}
				}
			}

			// trick to use nms in opencv
			found_.push_back(rt);
			// found_.push_back(rt);
		}
	}
#ifdef USE_DET_RESPONSE
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

double DetectionReadinNode::getConfidence(const cv::Rect &rt, double depth)
{
	double overlap = 0.0; // detectionOberlap(rt);
	int idx = 0;

	overlap = getMinDist2Dets(found_, idx, rt, det_std_x_, det_std_y_, det_std_h_);
	if(overlap < 4.0) { // within range
		if(obj_type_ == ObjPerson)
			overlap = (4.0 - overlap) * responses_[idx];
		else if(obj_type_ == ObjCar)
			overlap = (4.0 - overlap) * responses_[idx];
	}
	else { // too far
		overlap = 0.0;
	}

	if(version_ == 3) {
		if(rt.height > 180) {
			return -10.0;
		}

		// reference point is at the feet
		cv::Point pt(rt.x + rt.width / 2, rt.y + rt.height);
		for(size_t i = 0; i < confidences_.size(); i++) {
			if( rt.height >= (confidences_[i].size_ * (1 + 1 / det_scale_) / 2) 
				&& rt.height <= (confidences_[i].size_ * (1 + det_scale_) / 2)) {

				int x = floor((pt.x - confidences_[i].minx_) / confidences_[i].step_);
				int y = floor((pt.y - confidences_[i].miny_) / confidences_[i].step_);

				// check whether point is in image
				if((x < 0) || (y < 0) || (x > confidences_[i].map_.cols) || (y > confidences_[i].map_.rows)) {
					return (overlap + obs_out_of_image) * weight_;
				}
#if 0				
				std::cout << x << " " 
							<< y << " : "
							<< pt.x << " "
							<< pt.y << " size : " 
							<< confidences_[i].size_ << " ov : " << overlap << " conf: " 
							<< confidences_[i].map_.at<float>(y, x) << std::endl << std::endl;

				cv::waitKey();
#endif
				return (overlap + confidences_[i].map_.at<float>(y, x)) * weight_;
			}
		}
	}
	else {
		cv::Point pt(rt.x, rt.y);
		double whratio = (double)rt.width / (double)rt.height;
		for(size_t i = 0; i < confidences_.size(); i++) {
			if( abs(confidences_[i].size_ratio_ - whratio) < 0.1 * whratio
				&& rt.height >= (confidences_[i].size_ * (1 + 1 / det_scale_) / 2) 
				&& rt.height <= (confidences_[i].size_ * (1 + det_scale_) / 2)) {

				if(obj_type_ == ObjCar) assert(i % 2 == 1); // not ready!
				int x = floor((pt.x - confidences_[i].minx_) / confidences_[i].step_);
				int y = floor((pt.y - confidences_[i].miny_) / confidences_[i].step_);
				// check whether point is in image
				if((x < 0) || (y < 0) || (x > confidences_[i].map_.cols) || (y > confidences_[i].map_.rows)) {
					return (overlap + obs_out_of_image) * weight_;
				}
#if 0		
				std::cout << "(" << pt.x << " - " << confidences_[i].minx_ << ") / " << confidences_[i].step_ << std::endl;
				std::cout << "x : " << x << " y : " << y << std::endl;

				std::cout << " " << weight_ << " " << confidences_[i].map_.at<float>(y, x) << " " << overlap << " ";
				std::cout << "conf : " << (overlap + confidences_[i].map_.at<float>(y, x)) * weight_ << " ";

				for(int ii = -5; ii < 5; ii++) {
					for(int jj = -5; jj < 5; jj++) {
						std::cout << confidences_[i].map_.at<float>(y + ii, x + jj) << " ";
					}
					std::cout << std::endl;
				}

				cv::Rect rtt = rt;
				print_rect(rtt);
				std::cout << "idx " << i;
				confidences_[i].showMap(); 
				cv::waitKey();
#endif

				return (overlap + confidences_[i].map_.at<float>(y, x)) * weight_;
			}
		}
	}

	return (overlap + obs_out_of_image) * weight_;
}
}; // Namespace
