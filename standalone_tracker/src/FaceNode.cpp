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

#include <observation/FaceNode.h>
#include <boost/lexical_cast.hpp>
#include <iostream>

namespace people {
FaceNode::FaceNode()
{
	node_type_ = "face_detector";
	// cascadeName_ = "/u/wchoi/ros/people_experimental/ped_tracker/data/haarcascade_frontalface_default.xml";
	// cascadeName_ = "/u/wchoi/ros/people_experimental/ped_tracker/data/haarcascade_frontalface_alt.xml";
	face_model_file_ = "";
	group_threshold_ = 2;
	weight_ = 1.0;
}

FaceNode::~FaceNode()
{
}

void FaceNode::init()
{
	ObservationNode::init();
	if(face_model_file_ == "")
	{
		std::cout << "Face model file is not set!! set parameter face_model_file to FaceNode" << std::endl;
		exit(1);
	}
	if(!impl_cpu_.load(face_model_file_))
		assert(0);
}
void FaceNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "face_model_file") {
		face_model_file_ = value;
		init();
	}
	else if(name == "face_weight")
		weight_ = boost::lexical_cast<double>(value);
	else if(name == "face_group_threshold")
		group_threshold_ = boost::lexical_cast<int>(value);
}

void FaceNode::setData(const void *data, const std::string& type)
{
	if(type == "image_mono")
		img_mono_ = *(cv::Mat*)data;
}

void FaceNode::preprocess()
{
	impl_cpu_.detectMultiScale(img_mono_, found_, 1.2, group_threshold_, CV_HAAR_SCALE_IMAGE, cv::Size(20, 20));
	org_found_ = found_;
#if 0
	return;
#endif
	double cx, cy;
	std::vector<cv::Rect>::iterator it;
	for(it = found_.begin(); it < found_.end(); it++) 
	{
		// make it compatible with ub detection
		cx = (*it).x + (double)(*it).width / 2;
		cy = (*it).y;
		
		(*it).width *= 3.25;
		(*it).height *= 3.25;
		(*it).x = cx - (*it).width / 2;
		(*it).y -= (*it).height / 10;
	}
#if 0
	if(found_.size() > 0) {
		std::cout << "Face detected!! " << found_.size() << std::endl;
		cv::Mat img = img_mono_.clone();

		int i;
		for(i = 0; i < found_.size(); i++) {
			cv::rectangle(img, found_[i].tl(), found_[i].br(), cv::Scalar(255, 255, 255), 2);
			cv::rectangle(img, org_found_[i].tl(), org_found_[i].br(), cv::Scalar(55, 55, 55), 4);
		}

		show_image(img, "faces", 600);
		cv::waitKey();
	}
	else {
		std::cout << "No face detected!!" << std::endl;
	}
#endif
}

void FaceNode::quaryData(const std::string &name, void *data)
{
	if(name == "face_detection")
		*((std::vector<cv::Rect>*)data) = org_found_;
	if(name == "face_ub_detection")
		*((std::vector<cv::Rect>*)data) = found_;
}

std::vector<cv::Rect> FaceNode::getDetections()
{
	return found_;
}

double FaceNode::getConfidence(const cv::Rect &rt, double depth)
{
	double ret = 0.0, temp;

	std::vector<cv::Rect>::iterator it;
	for(it = found_.begin(); it < found_.end(); it++) 
	{
		cv::Rect orect = rt & (*it);
		cv::Rect urect = rt | (*it);

		temp = (double)(orect.width * orect.height) / (urect.width * urect.height);
		if(temp > ret)
			ret = temp;
	}

	return ret * weight_;
}

}; // Namespace
