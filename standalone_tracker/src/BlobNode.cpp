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
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <observation/BlobNode.h>
#include <boost/lexical_cast.hpp>
#include <common/util.h>
#include <math.h>

namespace people {

BlobNode::BlobNode()
{
	node_type_ = "blob_node";
	
	color_image_ = NULL;
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

	default_detector_ = cv::FeatureDetector::create("MSER");
	default_detector_->set("minArea", 100);
	thresh_vals.push_back(123);
	thresh_vals.push_back(32);
	thresh_vals.push_back(218);
	thresh_vals.push_back(188);
	thresh_vals.push_back(256);
	thresh_vals.push_back(256);
	thresh_vals.push_back(60);
	thresh_vals.push_back(30);
	thresh_vals.push_back(134);
	thresh_vals.push_back(89);
	thresh_vals.push_back(150);
	thresh_vals.push_back(256);

	init();
}

BlobNode::~BlobNode()
{
}

void BlobNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "blob_weight")
		weight_ = boost::lexical_cast<double>(value);
	else if(name == "blob_det_scale")
		det_scale_ = boost::lexical_cast<double>(value);
	else if(name == "blob_threshold")
		hit_threshold_ = boost::lexical_cast<double>(value);
	else if(name == "blob_detector_area")
		default_detector_->set("minArea", boost::lexical_cast<int>(value));
	else if(name == "detection_std_x")
		det_std_x_ = boost::lexical_cast<double>(value);
	else if(name == "detection_std_y")
		det_std_y_ = boost::lexical_cast<double>(value);
	else if(name == "detection_std_h")
		det_std_h_ = boost::lexical_cast<double>(value);
}

void BlobNode::setData(const void *data, const std::string& type)
{
	if(type == "image_mono") {
		cv::Mat image = *(cv::Mat*)data;
		imsize_.width = image.cols;
		imsize_.height = image.rows;
	}
	else if(type == "time_sec") {
		time_sec_ = *(double*)data;
	}
	else if(type == "image_color") {
		color_image_ = (cv::Mat*)data;
	}
}

void BlobNode::quaryData(const std::string &name, void *data)
{
	if(name == "blob_detection")
		*((std::vector<cv::Rect>*)data) = found_;
}

void BlobNode::preprocess()
{
	assert(detectBlobs());
}

bool BlobNode::supress(std::vector<cv::KeyPoint>& keypoints,
	                     cv::KeyPoint& query) {
	bool ret = false;
	for (std::vector<cv::KeyPoint>::iterator kpi = keypoints.begin(),
		   kpe = keypoints.end(); kpi != kpe; ++kpi) {
		cv::KeyPoint kp = *kpi;
		int distx = kp.pt.x - query.pt.x;
		int disty = kp.pt.y - query.pt.y;
		int dist = sqrt(distx*distx + disty*disty);
		if(query.size > kp.size && (dist < kp.size)){
			query.response++;
		} else if((kp.size > query.size) && (dist < kp.size)) {
			ret = true;
		}
	}
	return ret;
}

bool BlobNode::detectBlobs()
{
	if (color_image_ == NULL) {
		std::cout << "ERROR: color image is null!" << std::endl;
		return false;
	}

	// Clear the results
	found_.clear();
	confidence_.clear();

	// Initialize variables.
	cv::Mat hsv, blur, thresholded, thresholdedb, thresholdedr, frameBot;
	//Only consider the bottom half of the image.
	frameBot = color_image_->cv::Mat::rowRange(cv::Range((int)((color_image_->rows)/2), color_image_->rows));
	//Convert the image to HSV.
	cv::cvtColor(frameBot, hsv, CV_BGR2HSV);
	//Threshold the image to help track the balls.
	cv::inRange(hsv, cv::Scalar(thresh_vals[0], thresh_vals[1], thresh_vals[2]), 
			cv::Scalar(thresh_vals[3], thresh_vals[4], thresh_vals[5]), thresholdedb);
	cv::inRange(hsv, cv::Scalar(thresh_vals[6], thresh_vals[7], thresh_vals[8]), 
			cv::Scalar(thresh_vals[9], thresh_vals[10], thresh_vals[11]), thresholdedr);
	// int huemin,satmin,valmin,huemin2,satmin2,valmin2 = 0;
	// int huemax,satmax,valmax,huemax2,satmax2,valmax2 = 256;
	// huemin = 123;
	// satmin = 32;
	// valmin = 218;
	// huemax = 188;
	// satmax = 256;
	// valmax = 256;
	// huemin2 = 60;
	// satmin2 = 30;
	// valmin2 = 134;
	// huemax2 = 89;
	// satmax2 = 150;
	// valmax2 = 256;
	// inRange(hsv, cv::Scalar(huemin,satmin,valmin), cv::Scalar(huemax,satmax,valmax), thresholdedb);
	// inRange(hsv, cv::Scalar(huemin2,satmin2,valmin2), cv::Scalar(huemax2,satmax2,valmax2), thresholdedr);

	cv::bitwise_or(thresholdedb,thresholdedr,thresholded);
	cv::imshow("thresh",thresholded);
	//Blur the image to improve detection results.
	GaussianBlur(thresholded, blur, cv::Size(9, 9), 3, 3);
	cv::imshow("blur",blur);

	// Get detections
	std::vector<cv::KeyPoint> keypoints;
	default_detector_->detect(blur, keypoints);

	std::cerr << "------------------\n";

	for(size_t i = 0; i < keypoints.size(); i++){
			if(!supress(keypoints,keypoints[i])){		
				cv::Point center = keypoints[i].pt;
				int radius = cvRound(keypoints[i].size/2);
				cv::Point topLeft = center;
				topLeft.x -= radius;
				if(topLeft.x < 0)
					topLeft.x = 0;
				topLeft.y -= radius;
				if(topLeft.y < 0)
					topLeft.y = 0;
				cv::Point botRight = center;
				botRight.y += radius;
				if(botRight.y >= frameBot.rows)
					botRight.y = frameBot.rows-1;
				botRight.x += radius;
				if(botRight.x >= frameBot.cols)
					botRight.x = frameBot.cols-1;
				int width = botRight.x - topLeft.x;
				int height = botRight.y - topLeft.y;
				//Bounding box of a maximal detection.
				cv::Rect r(topLeft.x,topLeft.y + frameBot.rows,width,height);
				cv::rectangle(*color_image_,r,cv::Scalar(256,0,0),1,CV_AA);
				std::cerr << "x: " << r.x << " | y:" << r.y
											<< " | width: " << r.width << " | height: " << r.height << std::endl;
				found_.push_back(r);
				confidence_[&(found_.back())] = keypoints[i].response;
			}
		}
		cv::imshow("frameBot",frameBot);
	// Convert detections to Rects and push them into found
/*	for (std::vector<cv::KeyPoint>::iterator kpi = keypoints.begin(),
		   kpe = keypoints.end(); kpi != kpe; ++kpi) {
		cv::KeyPoint kp = *kpi;
		if (!supress(keypoints, kp)) {

			cv::Point center = kp.pt;
			int radius = cvRound(kp.size / 2);
			
			cv::Point topLeft = center;
			double new_x = topLeft.x - radius;
			double new_y = topLeft.y - radius;
			topLeft.x = new_x < 0 ? 0 : new_x;
			topLeft.y = new_y < 0 ? 0 : new_y;

			cv::Point botRight = center;
			new_x = botRight.x + radius;
			new_y = botRight.y + radius;
			botRight.x = new_x >= frameBot.rows ? frameBot.rows - 1 : new_x;
			botRight.y = new_y >= frameBot.cols ? frameBot.cols - 1 : new_y;

			int width = botRight.x - topLeft.x;
			int height = botRight.y - topLeft.y;

			cv::Rect r(topLeft.x, topLeft.y, width, height);
			found_.push_back(r);
			confidence_[&(found_.back())] = kp.response;
		}
	}*/

	return true;
}

std::vector<cv::Rect> BlobNode::getDetections()
{
	return found_;
}

//TODO
double BlobNode::getConfidence(const cv::Rect &rt, double depth)
{		
	return 100;
	// return confidence_[&rt];
	// double overlap = 0.0; // detectionOberlap(rt);
	// int idx = 0;

	// overlap = getMinDist2Dets(found_, idx, rt, det_std_x_, det_std_y_, det_std_h_);
	// if(overlap < 4.0) { // within range
	// 	if(obj_type_ == ObjPerson)
	// 		overlap = (4.0 - overlap) * responses_[idx];
	// 	else if(obj_type_ == ObjCar)
	// 		overlap = (4.0 - overlap) * responses_[idx];
	// }
	// else { // too far
	// 	overlap = 0.0;
	// }

	// if(version_ == 3) {
	// 	if(rt.height > 180) {
	// 		return -10.0;
	// 	}

	// 	// reference point is at the feet
	// 	cv::Point pt(rt.x + rt.width / 2, rt.y + rt.height);
	// 	for(size_t i = 0; i < confidences_.size(); i++) {
	// 		if( rt.height >= (confidences_[i].size_ * (1 + 1 / det_scale_) / 2) 
	// 			&& rt.height <= (confidences_[i].size_ * (1 + det_scale_) / 2)) {

	// 			int x = floor((pt.x - confidences_[i].minx_) / confidences_[i].step_);
	// 			int y = floor((pt.y - confidences_[i].miny_) / confidences_[i].step_);

	// 			// check whether point is in image
	// 			if((x < 0) || (y < 0) || (x > confidences_[i].map_.cols) || (y > confidences_[i].map_.rows)) {
	// 				return (overlap + obs_out_of_image) * weight_;
	// 			}

	// 			return (overlap + confidences_[i].map_.at<float>(y, x)) * weight_;
	// 		}
	// 	}
	// }
	// else {
	// 	cv::Point pt(rt.x, rt.y);
	// 	double whratio = (double)rt.width / (double)rt.height;
	// 	for(size_t i = 0; i < confidences_.size(); i++) {
	// 		if( abs(confidences_[i].size_ratio_ - whratio) < 0.1 * whratio
	// 			&& rt.height >= (confidences_[i].size_ * (1 + 1 / det_scale_) / 2) 
	// 			&& rt.height <= (confidences_[i].size_ * (1 + det_scale_) / 2)) {

	// 			if(obj_type_ == ObjCar) assert(i % 2 == 1); // not ready!
	// 			int x = floor((pt.x - confidences_[i].minx_) / confidences_[i].step_);
	// 			int y = floor((pt.y - confidences_[i].miny_) / confidences_[i].step_);
	// 			// check whether point is in image
	// 			if((x < 0) || (y < 0) || (x > confidences_[i].map_.cols) || (y > confidences_[i].map_.rows)) {
	// 				return (overlap + obs_out_of_image) * weight_;
	// 			}

	// 			return (overlap + confidences_[i].map_.at<float>(y, x)) * weight_;
	// 		}
	// 	}
	// }

	// return (overlap + obs_out_of_image) * weight_;
}
}; // Namespace
