#include <common/vpestimator.h>
#include <opencv/highgui.h>

namespace people {

VPEstimator::VPEstimator()
{
	weight_ = 1;
	votestep_ = 15;
	vmap_ = cv::Mat();
	lines_.clear();
}

VPEstimator::~VPEstimator()
{
}

void VPEstimator::preprocess(cv::Mat &image, double timestamp)
{
	lines_.clear();
}

bool VPEstimator::readPreprocessedFile(const std::string &filename)
{
	FILE *fp;
	size_t nread;
	
	if(filename == "") {
		vmap_ = cv::Scalar(0);
		lines_.clear();
		return true;
	}

	std::cout << "read vpfile : " << filename << std::endl;
	fp = fopen(filename.c_str(), "r");
	if(fp == NULL) {
		std::cout << "ERROR :Cannot read voting file!" << std::endl;
		fclose(fp);
		return false;
	}
	vmap_ = cv::Scalar(0);

	char header[4];
	nread = fread(header, sizeof(char), 4, fp);
	assert(nread == 4);
	if(!(header[0] == 'V'	&& header[1] == 'P'
		&& header[2] == '0'	&& header[3] == '0')) {
		std::cout << "ERROR : invalid header format!" << std::endl;
		fclose(fp);
		return false;
	}

	float temp[2];
	float vstep;
	float *data;
	////////////////////////////////////////////////////////////////////////////
	nread = fread(&temp, sizeof(float), 2, fp);
	assert(nread == 2);
	nread = fread(&vstep, sizeof(float), 1, fp);
	assert(nread == 1);
	votestep_ = vstep;

	int vsize[2];
	vsize[0] = round(temp[0]);
	vsize[1] = round(temp[1]);

	data = new float [ vsize[0] * vsize[1] ];
	nread = fread(data, sizeof(float), vsize[0] *vsize[1], fp);
	
	vmap_ = cv::Mat(vsize[0], vsize[1], CV_32F);
	for(int row = 0; row < vsize[0]; row++) {
		for(int col = 0; col < vsize[1]; col++) {
			vmap_.at<float>(row, col) = data[ row + col * vsize[0] ];
		}
	}
	delete data;
	////////////////////////////////////////////////////////////////////////////
	lines_.clear();
	unsigned int numlines = 0;
	nread = fread(&numlines, sizeof(unsigned int), 1, fp);
	assert(nread == 1);

	longline oneline;
	for(unsigned int i = 0; i < numlines; i++) {
		nread = fread(&oneline, sizeof(float), 6, fp);
		assert(nread == 6);
		lines_.push_back(oneline);
		// std::cout << oneline.x1_ << " " << oneline.x2_ << " " << oneline.y1_ << " " << oneline.y2_ << std::endl;
	}
	////////////////////////////////////////////////////////////////////////////
#if 0
	cv::Mat image(vmap_.rows, vmap_.cols, CV_8U);
	for(int row = 0; row < vsize[0]; row++) {
		for(int col = 0; col < vsize[1]; col++) {
			image.at<char>(row, col) = (vmap_.at<float>(row, col) > 255.0) ? 255 : floor(vmap_.at<float>(row, col));
		}
	}
	cv::imshow("test", image);
	cv::waitKey();
#endif
	fclose(fp);

	return true;
}

double VPEstimator::getHorizonConfidence(int hor)
{
	double ret = 0;

	if(lines_.size() == 0)
		return ret;

	int yidx = floor((double)hor / votestep_);
	for(int i = 0; i < vmap_.cols; i++) {
		if(ret < vmap_.at<float>(yidx, i)) {
			ret = (double)vmap_.at<float>(yidx, i);
		}
	}
	return ret * weight_;
}
};
