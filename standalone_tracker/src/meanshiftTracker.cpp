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

#include <common/meanshiftTracker.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <ctype.h>

#define PI 3.14159
#define nBits 4
#define eps pow(2,-52)
typedef unsigned char uchar;

using namespace people;
using namespace std;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// meanshift functions
float buildKernel(int ht, int wd, float *kernel, float *xs, float *ys);
std::vector<cv::Mat> normalizeRoi(cv::Mat &roi);
cv::MatND buildHist(float *kernel, float ksum, std::vector<cv::Mat> &Qc);
float computeW(float *w, std::vector<cv::Mat> &Qc, cv::MatND &p, cv::MatND &q, int ht, int wd);
cv::Point updatePos(float *w, float *xs, float *ys, float wsum, int ht, int wd);
cv::MatND initHist();
cv::MatND updateHist(const cv::MatND &hist_pre, const cv::MatND &hist, double alpha);

float buildKernel(int ht, int wd, float *kernel, float *xs, float *ys)
{
	float dx = 2.0 / (wd-1.0);
	float dy = 2.0 / (ht-1.0);
	//std::cout << dx << " " << dy << std::endl;
	float ksum = 0.0;
	//std::cout << "kernel: ";
	for (int i = 0; i < ht; i++) {
		for (int j = 0; j < wd; j++) {
			xs[i + j*ht] = -1.0 + j*dx;
			//std::cout << xs[i + j*ht] << " ";
		}
	}

	for (int i = 0; i < ht; i++) {
		for (int j = 0; j < wd; j++) {
			ys[i + j*ht] = -1.0 + i*dy;
			//std::cout << ys[i + j*ht] << " ";
		}
	}

	for (int i = 0; i < ht; i++) {
		for (int j = 0; j < wd; j++) {
			kernel[i + j*ht] = pow(xs[i + j*ht],2) + pow(ys[i + j*ht],2);
			if(kernel[i + j*ht] > 1.0) kernel[i + j*ht] = 1.0;
			kernel[i + j*ht] = 2.0/PI * (1.0-kernel[i + j*ht]);
			//std::cout << kernel[i + j*ht] << " ";
			ksum += kernel[i + j*ht];
		}
	}
	//std::cout << "ksum: " << ksum << std::endl;

	return ksum;
}

std::vector<cv::Mat> normalizeRoi(cv::Mat &roi)
{
	//cv::Mat roi;
	//roi2.convertTo(roi,CV_8U);
	//std::cout << "type: " << roi.type() << " " << roi.depth() << " " << roi.channels() << std::endl;
	std::vector<cv::Mat> Qc;
	cv::split(roi,Qc);
	//std::cout << "size: " << Qc.size() << std::endl;
	//std::cout << "value: ";
	int shift_bit = 8-nBits;
#if 0
	for (int i = 0; i < roi.cols; i++) {
		for (int j = 0; j < roi.rows; j++) {
			std::cout << (int)Qc[0].at<uchar>(j,i) << " ";
		}
	}
#endif
	for (int i = 0; i < roi.rows; i++) {
		for (int j = 0; j < roi.cols; j++) {
			for(int c = 0; c < 3; c++)
			{
				Qc[c].at<uchar>(i,j) = Qc[c].at<uchar>(i,j) >> shift_bit;
				//std::cout << (int)Qc[0].at<uchar>(i,j) << " ";
				//Qc[i + j*roi.rows + c*roi.rows*roi.cols] = Qc_temp[c].at<uchar>(i,j);
				//std::cout << (int)Qc[i + j*roi.rows + c*roi.rows*roi.cols] << " ";
			}
		}
	}
#if 0
	for (int i = 0; i < roi.cols; i++) {
		for (int j = 0; j < roi.rows; j++) {
			std::cout << (int)Qc[0].at<uchar>(j,i) << " ";
		}
	}
#endif
	return Qc; // values are between 0~32	
}

cv::MatND buildHist(float *kernel, float ksum, std::vector<cv::Mat> &Qc)
{
	cv::MatND hist = initHist();

 	for (int i = 0; i < Qc[0].rows; i++) {
		for (int j = 0; j < Qc[0].cols; j++) {
			uchar x = Qc[0].at<uchar>(i,j);
			uchar y = Qc[1].at<uchar>(i,j);
			uchar z = Qc[2].at<uchar>(i,j);

			float temp = hist.at<float>(x, y, z);
			*((float*)hist.ptr(x, y, z)) = temp + kernel[i + j * Qc[0].rows] / ksum;
		}
	}
#if 0
	int bins = 1<<nBits;	

	float temp2 = 0;
	for(int r = 0; r < bins; r++) {
		for(int g = 0; g < bins; g++) {
			for(int b = 0; b < bins; b++) {
				temp2 += hist.at<float>(r, g, b);				
			}
		}
	}
	std::cout << "sum: " << temp2 << std::endl;
#endif

	return hist;
}

float computeW(float *w, std::vector<cv::Mat> &Qc, cv::MatND &p, cv::MatND &q, int ht, int wd)
{
	cv::MatND qp = p.clone();
	int bins = 1<<nBits;
	for(int r = 0; r < bins; r++) {
        	for(int g = 0; g < bins; g++) {
			for(int b = 0; b < bins; b++) {
				float temp_p = p.at<float>(r, g, b);
				float temp_q = q.at<float>(r, g, b);
				*((float*)qp.ptr(r, g, b)) = ( temp_p > 0.0 ) ? sqrt(temp_q/temp_p) : 0.0;
				//std::cout << qp.at<float>(r, g, b) << " ";
			}
		}
	}

	//std::cout << "w: " << std::endl;
	float wsum = 0;
	for (int i = 0; i < ht; i++) {
		for (int j = 0; j < wd; j++) {
			uchar x = Qc[0].at<uchar>(i,j);
			uchar y = Qc[1].at<uchar>(i,j);
			uchar z = Qc[2].at<uchar>(i,j);
			w[i + j*ht] = qp.at<float>(x, y, z);
			//std::cout << w[i + j*ht] << " ";
			wsum += w[i + j*ht];
		}
  	}
	//std::cout << "wsum: " << wsum << std::endl;
	return wsum;
}

cv::Point updatePos(float *w, float *xs, float *ys, float wsum, int ht, int wd)
{
	cv::Point pos;
	// float pos_x, pos_y;	
	float sum_x = 0;
	float sum_y = 0;
	for (int i = 0; i < ht; i++) {
		for (int j = 0; j < wd; j++) {
			sum_x += xs[i + j*ht]*w[i + j*ht];
			sum_y += ys[i + j*ht]*w[i + j*ht];			
		}
	}
	sum_x = sum_x/(wsum+eps)*wd/2.0;
	sum_y = sum_y/(wsum+eps)*ht/2.0;

	//std::cout << "sum: " << sum_x << " " << sum_y << std::endl;
	// pos_x = floor(sum_x+0.6);
	// pos_y = floor(sum_y+0.6);
	//std::cout << "pos: " << pos_x << " " << pos_y << std::endl;
	
	pos.x = round(sum_x + 0.1);
	pos.y = round(sum_y + 0.1);

	return pos;
}

cv::MatND initHist()
{
	int bins[3];
	for(int i = 0; i < 3; i++) { bins[i] = 1 << nBits; }
	return cv::MatND(3, bins, CV_32FC1, cv::Scalar(0.0f));
#if 0
	int histSize[] = {bins, bins, bins};
    	float rranges[] = { 0, 255 };
    	float granges[] = { 0, 255 };
    	float branges[] = { 0, 255 };
    	const float* ranges[] = {rranges, granges, branges};
    	cv::MatND hist;
   	int channels[] = {0, 1, 2};
	cv::calcHist(&image, 1, channels, cv::Mat(), hist, 3, histSize, ranges, true, false);

	//std::cout << "hist: " << std::endl;
	for(int r = 0; r < bins; r++) {
		for(int g = 0; g < bins; g++) {
			for(int b = 0; b < bins; b++) {
				*((float*)hist.ptr(r, g, b)) = 0.0;
				//std::cout << hist.at<float>(r, g, b) << " ";
			}
		}
	}
	return hist;
#endif
}

cv::MatND updateHist(const cv::MatND &hist_pre, const cv::MatND &hist, double alpha)
{
	int bins = 1<<nBits;
	cv::MatND out = hist;
	for(int r = 0; r < bins; r++) {
		for(int g = 0; g < bins; g++) {
			for(int b = 0; b < bins; b++) {
				*((float*)out.ptr(r, g, b)) = alpha*hist_pre.at<float>(r, g, b) + (1.0-alpha)*hist.at<float>(r, g, b);
				//std::cout << out.at<float>(r, g, b) << " ";
			}
		}
	}

	return out;
}

float people::computeSim(const cv::MatND &hist1, const cv::MatND &hist2)
{
	int bins = 1 << nBits;
	float sim = 0;

	for(int r = 0; r < bins; r++) {
		for(int g = 0; g < bins; g++) {
			for(int b = 0; b < bins; b++) {
				sim += sqrt(hist1.at<float>(r, g, b) * hist2.at<float>(r, g, b));
			}
		}
	}

	return sim;
}
//////////////////////Meanshift utility functions end

/////////////////////////////////MSTarget begins
/////////////////////////////////////////////////////////////
bool MSTarget::updateHistogram(cv::Mat &image, cv::Rect rt, float hratio, float alpha)
{
#if 1
	// unreliable if the bbs are around image border
	if(rt.x < 0 || rt.y < 0 
		|| rt.x + rt.width >= image.cols || rt.y + rt.height >= image.rows
		|| rt.width <= 0 || rt.height <= 0) {
		count_ = 0;
		return false;
	}
#else
	rt &= cv::Rect(0, 0, image.cols, image.rows);
	if(rt.width <= 0 || rt.height <= 0) {
		count_ = 0;
		return false;
	}
#endif
	cv::Mat image_roi = image(rt);
	// build new histogram
	float *kernel = new float [rt.height * rt.width];
	float *xs = new float [rt.height * rt.width];
	float *ys = new float [rt.height * rt.width];
	float ksum = buildKernel(rt.height, rt.width, kernel, xs, ys);
	std::vector<cv::Mat> Qc;
	Qc = normalizeRoi(image_roi);
	cv::MatND hist_new = buildHist(kernel, ksum, Qc);
	delete kernel; 	delete xs; 	delete ys;
	/////////////////////////////////////////
	// update hist
	if(count_ == 0 || hratio_ != hratio) { // initial creation
		hist_ = hist_new;
	}
	else { // update
		assert(hist_.cols == hist_new.cols);
		assert(hist_.rows == hist_new.rows);
		hist_ = updateHist(hist_, hist_new, alpha);
	}
	hratio_ = hratio;
	rect_ = rt; // rectangle location
	/////////////////////////////////////////
	count_++;
	/////////////////////////////////////////
	return true;
}

float MSTarget::runMeanShift(cv::Mat &image, cv::Rect &out, cv::Rect init)
{
	float sim = eps;
	if(count_ == 0) {
		out = cv::Rect(0,0,0,0);
		return sim; // nothings to be done
	}

	cv::MatND hist_new;
	// cv::Rect rt;
	cv::Mat imgroi;

	float pos_x, pos_y;
	int wd, ht;
	int iter = 20; // maximum number of iterations
	std::vector<cv::Mat> Qc; // quantized color values
	// initialize the position if not given
	if(init.x == 0 && init.y == 0 && init.width == 0 && init.height == 0) {
		init = rect_;
	}
	pos_x = init.x + init.width / 2;
	pos_y = init.y + init.height / 2;
	wd = init.width; 	ht = init.height;

	// build kernel
	float *kernel = new float [ht * wd];
	float *xs = new float [ht * wd];
	float *ys = new float [ht * wd];

	float ksum = buildKernel(ht, wd, kernel, xs, ys);
	float wsum;
	cv::Point pos;

	// run meanshift
	// MeanShift step	
	float *w = new float [ht * wd];
	for(int j = 0; j < iter; j++) {
		out = cvRect(pos_x - wd / 2.0, pos_y - ht / 2.0, wd, ht);
		if(out.x < 0 || out.y < 0 || out.x + out.width > image.cols || out.y + out.height > image.rows) {
			// failed to track 
			delete w;  delete kernel; 	delete xs;	delete ys;	
			return eps;
		}

		imgroi = image(out);
		Qc = normalizeRoi(imgroi);
		hist_new = buildHist(kernel, ksum, Qc);
		wsum = computeW(w, Qc, hist_new, hist_, ht, wd);
		pos = updatePos(w, xs, ys, wsum, ht, wd);

		// convergence, no need to move
		if(pos.x == 0 && pos.y == 0) break;
		pos_x += pos.x;
		pos_y += pos.y;
	}
	delete w; delete kernel; 	delete xs; 	delete ys;	
	sim = computeSim(hist_new, hist_);

	return sim;
}

// run meanshift with scale search
float MSTarget::runScaleMeanShift(cv::Mat &image, cv::Rect &max_out, cv::Rect init)
{
	float max_sim = 0.0f;
	float scale_step = 1.05;
	int max_step = 3; // 1.05^3 = 1.16
	
	if(init.x == 0 && init.y == 0 && init.width == 0 && init.height == 0) {
		init = rect_;
	}
	
	init.x += init.width / 2;
	init.y += init.height / 2;

	cv::Rect irect;
	cv::Rect out;
	float sim;

	for(int s = -max_step; s <= max_step; s++) {
		// for each scale space, use different initial BB
		irect = init;
		irect.width *= pow(scale_step, s);
		irect.height *= pow(scale_step, s);
		irect.x -= irect.width / 2;
		irect.y -= irect.height / 2;
		// run fixed scale MS
		sim = runMeanShift(image, out, irect);
		// pick the best matching result
		if(sim > max_sim) {
			max_sim = sim;
			max_out = out;
		}
	}

	return max_sim;
}

bool people::buildHistogram(const cv::Mat &image, const cv::Rect &rt, bool init, const cv::MatND &hist_in, cv::MatND &hist_out, float alpha)
{
	// unreliable if the bbs are around image border
	if(rt.x < 0 || rt.y < 0 
		|| rt.x + rt.width >= image.cols || rt.y + rt.height >= image.rows
		|| rt.width <= 0 || rt.height <= 0) {
		return false;
	}
	cv::Mat image_roi = image(rt);

	// build new histogram
	float *kernel = new float [rt.height * rt.width];
	float *xs = new float [rt.height * rt.width];
	float *ys = new float [rt.height * rt.width];
	float ksum = buildKernel(rt.height, rt.width, kernel, xs, ys);
	std::vector<cv::Mat> Qc;
	Qc = normalizeRoi(image_roi);
	cv::MatND hist_new = buildHist(kernel, ksum, Qc);
	delete kernel; 	delete xs; 	delete ys;

	/////////////////////////////////////////
	// update hist
	if(!init) { // initial creation
		hist_out = hist_new;
	}
	else { // update
		hist_out = updateHist(hist_in, hist_new, alpha);
	}
	return true;
}

#if 0
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
KernelMeanShift::KernelMeanShift()
{
}	

KernelMeanShift::~KernelMeanShift()
{
}

cv::Mat KernelMeanShift::runMeanShift(cv::Mat &image_color, std::vector<cv::MatND> &hist_ms, std::vector<cv::Rect> &rect_ms)
{
	cv::Mat image_draw = image_color.clone();
#if 0
	cv::Scalar color;		
	std::vector<cv::Rect> rect_draw;
	
	assert(hist_ms.size() == rect_ms.size());

	cv::Rect rt, win;
	cv::Mat imgroi;
	cv::MatND hist, hist_init;
	std::vector<cv::Mat> Qc;	
	std::vector<float> pos;
	float ksum, wsum;	
	float pos_x, pos_y;
	int wd, ht;

	int iter = 20;
	int len = 10;     // update initial rectangle every (len) frames
	double alpha = 1; // higher, consider previous histogram more
	bool show = 0;

	for(size_t i = 0; i < targets_.size(); i++) {
		// exist tracking target
		if(count_ms[i] > 1) {                                                 
		// initialization
		if((count_ms[i]-2) % len == 0) {
		rt = targets_[i]->getRect(count_ms[i]-2);                                       					
		rt &= cv::Rect(0, 0, image_draw.cols, image_draw.rows);
		ht = rt.height - rt.height % 2;
		wd = rt.width - rt.width % 2;

		pos_x = rt.x + wd/2.0;
		pos_y = rt.y + ht/2.0;
		rt = cvRect(rt.x, rt.y, wd, ht);				
		}

		else {
		rt = rect_ms[i];
		ht = rt.height;
		wd = rt.width;
		pos_x = rt.x + wd/2.0;
		pos_y = rt.y + ht/2.0;						
		}

		float *kernel = new float [ht*wd];
		float *xs = new float [ht*wd];
		float *ys = new float [ht*wd];
		//std::cout << "init pos: " << pos_x << " " << pos_y << std::endl;

		//std::cout << "size: " << ht <<"," << wd << std::endl;
		std::cout << "size: " << rt.x << "," << rt.y << "," << rt.width <<"," << rt.height << std::endl;

		ksum = meanshift_.buildKernel(ht, wd, kernel, xs, ys);

		// initialization
		if((count_ms[i]-2) % len == 0) {                                          
		imgroi = image_draw(rt);
		Qc = meanshift_.normalizeRoi(imgroi);
		//std::cout << "size: " << ht <<"," << wd << std::endl;
		//std::cout << "init pos: " << pos_x << " " << pos_y << std::endl;
		//std::cout << "size: " << rt.x <<"," << rt.y << std::endl;

		hist_init = meanshift_.buildHist(kernel, ksum, Qc, imgroi);				
		}

		// load previous histogram
		else hist_init = hist_ms[i];                                    

		// MeanShift step	
		for(int j = 0; j < iter; j++) {
		float *w = new float [ht*wd];
		rt = cvRect(pos_x - wd/2.0, pos_y - ht/2.0, wd, ht);
		if(rt.x < 0 || rt.y < 0 || rt.x+rt.width > image_draw.cols || rt.y+rt.height > image_draw.rows) {
		hist = hist_init;
		break;
		}
		//rt &= cv::Rect(0, 0, image_draw.cols, image_draw.rows);
		imgroi = image_draw(rt);
		Qc = meanshift_.normalizeRoi(imgroi);

		hist = meanshift_.buildHist(kernel, ksum, Qc, imgroi);

		wsum = meanshift_.computeW(w, Qc, hist, hist_init, ht, wd);

		pos = meanshift_.updatePos(w, xs, ys, wsum, ht, wd);
		//std::cout << "pos: " << pos[0] << " " << pos[1] << std::endl;
		if(pos[0] == 0 && pos[1] == 0) break;
		pos_x += pos[0];
		pos_y += pos[1];
		delete[] w;						
		}

		// update histogram
		hist_ms[i] = meanshift_.updateHist(hist_init, hist, alpha);       
		//std::cout << "end pos: " << pos_x << " " << pos_y << std::endl;
		win = cvRect(pos_x - wd/2.0, pos_y - ht/2.0, wd, ht);
		rect_draw.push_back(win);
		rect_ms[i] = win;					

		delete[] kernel;
		delete[] xs;
		delete[] ys;		
		}
	}

	if(show) {
	int idx = 0;
	for(size_t i = 0; i < targets_.size(); i++)
	{				
	if(count_ms[i] > 1) {
	color = get_target_color(idx);
	cv::rectangle(image_draw, rect_draw[idx].tl(), rect_draw[idx].br(), color, 3, CV_AA);
	idx++;
	}
	}
	}
#endif
	return image_draw;
}
#endif
