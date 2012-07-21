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

#include <common/vpestimator.h>
#include <opencv/highgui.h>

namespace people {

VPEstimator::VPEstimator()
{
	weight_ = .1;
	votestep_ = 15;
	vmap_ = cv::Mat();
	lines_.clear();
}

VPEstimator::~VPEstimator()
{
}

void VPEstimator::preprocess(cv::Mat &image, double timestamp)
{
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
#if 1
	double offset = (double)hor - (votestep_ * (double)yidx);
	// std::cout << hor << " " << yidx << " " << offset << " ";
	// std::cout << (1.0 - (offset / votestep_)) << " " << (offset / votestep_)  << " ";
	// std::cout << getMaxHorizonVote(yidx) << " ";
	// std::cout << getMaxHorizonVote(yidx + 1) <<  " ";
	// std::cout << std::endl;
	ret = (1.0 - (offset / votestep_)) * getMaxHorizonVote(yidx);
	ret += (offset / votestep_) * getMaxHorizonVote(yidx + 1);
	return ret;
#else
	for(int i = 0; i < vmap_.cols; i++) {
		if(ret < vmap_.at<float>(yidx, i)) {
			ret = (double)vmap_.at<float>(yidx, i);
		}
	}
	return ret * weight_;
#endif
}

double VPEstimator::getMaxHorizonVote(int idx)
{
	double ret = 0.0;
	if(idx >= vmap_.rows) return 0.0;
	assert(vmap_.rows > idx);
	for(int i = 0; i < vmap_.cols; i++) {
		if(ret < vmap_.at<float>(idx, i)) {
			ret = (double)vmap_.at<float>(idx, i);
		}
	}
	return ret * weight_;
}

cv::Mat VPEstimator::getVPConfImage(int width)
{
	float scale = 255.0/1000.0;

	cv::Mat image(1,1,CV_8U);
	if(lines_.size() == 0) { // no info
		return image;
	}

	int ratio = floor((double)width/vmap_.cols);
	image = cv::Mat(vmap_.rows * ratio, vmap_.cols * ratio, CV_8U);
	
	for(int i = 0; i < vmap_.rows; i++) {
		for(int j = 0; j < vmap_.cols; j++) {
			float val = std::min(scale * vmap_.at<float>(i, j), (float)255.0);
			for(int k = 0; k < ratio; k++) {
				for(int l = 0; l < ratio; l++) {
					assert(i * ratio + k < image.rows);
					assert(j * ratio + l < image.cols);

					image.at<unsigned char>(i * ratio + k, j * ratio + l) = (unsigned char)floor(val);
				}
			}
		}
	}

	return image;
}

cv::Point2f VPEstimator::findVanishingPoint(int horizon, float delta)
{
	int yidx = floor((double)horizon / votestep_);
	int xidx = 0;
	double max_vote = 0.0;

	for(int x = 0; x < vmap_.cols; x++) {
		if(max_vote < vmap_.at<float>(yidx, x)) {
			max_vote = vmap_.at<float>(yidx, x);
			xidx = x;
		}
	}

	return cv::Point2f(votestep_ * xidx, horizon);
}

// refer to Varsha Hedau ICCV09
float VPEstimator::getVotingAngle(const longline &line, const cv::Point2f &pt)
{
	float alpha;
	float mx = (line.x1_ + line.x2_) / 2;
	float my = (line.y1_ + line.y2_) / 2;
	
	cv::Point2f v1(pt.x - mx, pt.y - my);
	cv::Point2f v2(line.x2_ - line.x1_, line.y2_ - line.y1_);
	
	float len = norm(v1);
	v1.x = v1.x / len;
	v1.y = v1.y / len;
	len = norm(v2);
	v2.x = v2.x / len;
	v2.y = v2.y / len;

	alpha = acosf(v1.x * v2.x + v1.y * v2.y);
	if(alpha > M_PI / 2) {
		alpha = M_PI - alpha;
	}
	// mpt = [(line(1) + line(2)) / 2, (line(3) + line(4)) / 2]; 
	// equ2 = null([pt, 1; mpt, 1])';
	// alpha = atan2(abs(equ(1) * equ2(2) - equ2(1) * equ(2)), abs(equ(1) * equ2(1) + equ(2) * equ2(2)));
	return alpha;
}

std::vector<longline> VPEstimator::getLinesIntersectingPoint(const cv::Point2f &pt, const float epstheta)
{
	std::vector<longline> lines;
	for(size_t i = 0; i < lines_.size(); i++) {
		// std::cout << " angle : " << getVotingAngle(lines_[i], pt) << std::endl;
		if(getVotingAngle(lines_[i], pt) < epstheta && lines_[i].y1_ > pt.y && lines_[i].y2_ > pt.y) {
			// print_line(lines_[i]);
			lines.push_back(lines_[i]);
		}
	}
	return lines;
}

};
