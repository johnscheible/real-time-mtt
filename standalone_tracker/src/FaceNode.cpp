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
