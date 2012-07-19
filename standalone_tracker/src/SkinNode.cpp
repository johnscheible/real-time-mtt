#include <observation/SkinNode.h>
#include <boost/lexical_cast.hpp>
#include <opencv/highgui.h>
namespace people {
SkinNode::SkinNode()
{
	hval_lb_ = 2;
	hval_ub_ = 15;
	sval_lb_ = 60;
	sval_ub_ = 200;
	vval_lb_ = 40;
	vval_ub_ = 200;
	weight_ = 1.0;
	dth_ = 0.2;

	node_type_ = "skin_detector";

	init();
}

SkinNode::~SkinNode()
{
}

void SkinNode::setParameter(const std::string &name, const std::string &value)
{
	if(name == "skin_weight") weight_ = boost::lexical_cast<double>(value);
	if(name == "skin_hval_lb")	hval_lb_ = boost::lexical_cast<unsigned char>(value);
	if(name == "skin_hval_ub")	hval_ub_ = boost::lexical_cast<unsigned char>(value);
	if(name == "skin_sval_lb")	sval_lb_ = boost::lexical_cast<unsigned char>(value);
	if(name == "skin_sval_ub")	sval_ub_ = boost::lexical_cast<unsigned char>(value);
	if(name == "skin_vval_lb")	vval_lb_ = boost::lexical_cast<unsigned char>(value);
	if(name == "skin_vval_ub")	vval_ub_ = boost::lexical_cast<unsigned char>(value);
}

void SkinNode::setData(const void *data, const std::string& type)
{
	if(type == "image_depth")
		img_depth_ = *(cv::Mat*)data;
	if(type == "image_hsv")
		img_hsv_ = *(cv::Mat*)data;
}

void SkinNode::preprocess()
{
	cv::Mat temp_mat(img_hsv_.rows, img_hsv_.cols, CV_8U);
	for(int i = 0; i < img_hsv_.rows; i++) {
		for(int j = 0; j < img_hsv_.cols; j++) {
			unsigned char hval = img_hsv_.at<char>(i, 3 * j);
			unsigned char sval = img_hsv_.at<char>(i, 3 * j + 1);
			unsigned char vval = img_hsv_.at<char>(i, 3 * j + 2);

			if((hval >= hval_lb_) && (hval <= hval_ub_) && (sval >= sval_lb_) && (sval <= sval_ub_) && (vval >= vval_lb_) && (vval <= vval_ub_))
				temp_mat.at<unsigned char>(i, j) = 1;
			else
				temp_mat.at<unsigned char>(i, j) = 0;
		}
	}
	cv::medianBlur(temp_mat, img_skin_, 3);
#if 0
	cv::Mat temp = img_skin_.clone() * 100;
	cv::imshow("skin pixels", temp);
#endif
}

double SkinNode::getConfidence(const cv::Rect &rt, double depth)
{
	double ret = 0;
	cv::Rect roi(rt.x + rt.width / 3, rt.y + rt.height / 8, rt.width / 3, rt.height / 2.5); // face region

	float lb = depth - dth_, ub = depth + dth_;
	float stepx = (float)(roi.width-1) / 10;
	float stepy = (float)(roi.height-1) / 10;

	// already normalized
	for(float x = roi.x, i = 0; i < 10 ; x += stepx, i++) {
		for(float y = roi.y, j = 0; j < 10 ; y += stepy, j++) {
			int ix = floor(x), iy = floor(y);
			if((ix < 0) || (iy < 0) || (ix >= img_depth_.cols) || (iy >= img_depth_.rows)) continue;

			if((img_depth_.at<float>(iy, ix) > lb) && (img_depth_.at<float>(iy, ix) < ub))
			{
				ret += (float)img_skin_.at<unsigned char>(iy, ix) / 50;
			}
		}
	}

	return ret * weight_;
}

}; // Namespace
