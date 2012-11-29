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

#include <stdio.h>
#include <stdlib.h>
#include <common/util.h>
#include <common/global.h>
#include <common/ped_state.h>
#include <common/gfeat_state.h>
#include <common/cam_state.h>

namespace people{
	void show_image(cv::Mat &im, const std::string &name, int max_height)
	{
		cv::Size imsz(im.cols, im.rows);
		cv::Mat resized;
		if(max_height < imsz.height) {
			double ratio = (double)max_height / (double)imsz.height;
			imsz.height *= ratio;
			imsz.width *= ratio;
			cv::resize(im, resized, imsz);
		}
		else {
			resized = im;			
		}

		cv::imshow(name, resized);
		cv::waitKey(20);
	}

	bool in_any_rect(const std::vector<cv::Rect> &rts, const cv::Point2f &pt)
	{
		for(size_t i = 0; i < rts.size(); i++) {
			if(inrect(rts[i], pt)) return true;
		}
		return false;
	}

	bool inrect(const cv::Rect &rt, const cv::Point2f &pt)
	{
		bool ret = false;

		ret = (rt.x < pt.x) 
					&& (rt.y < pt.y)
					&& (rt.x + rt.width - 1 > pt.x)
					&& (rt.y + rt.height - 1 > pt.y);

		return ret;
	}

	float bb_overlap(const cv::Rect& rt1, const cv::Rect& rt2)
	{
		float ret = 0;

		float temp1 = max(rt1.x, rt2.x);
		float temp2 = max(rt1.y, rt2.y);

		float temp3 = min(rt1.x + rt1.width
		, rt2.x + rt2.width);

		float temp4 = min(rt1.y + rt1.height
		, rt2.y + rt2.height);

		if((temp1 < temp3) && (temp2 < temp4)) {
			ret = 2 * (temp3 - temp1) * (temp4 - temp2);
			ret /= (rt1.width * rt1.height) + (rt2.width * rt2.height);
		}

		return ret;
	}

	float bb_intunion(const cv::Rect& rt1, const cv::Rect& rt2)
	{
		float intersect = 0;
		float ret = 0;

		float x1 = max(rt1.x, rt2.x);
		float y1 = max(rt1.y, rt2.y);
		float x2 = min(rt1.x + rt1.width, rt2.x + rt2.width);
		float y2 = min(rt1.y + rt1.height, rt2.y + rt2.height);

		if((x1 < x2) && (y1 < y2)) {
			intersect = (x2 - x1) * (y2 - y1);
			ret = intersect / ((rt1.width * rt1.height) + (rt2.width * rt2.height) - intersect);
		}
		else {
			ret = 0;
		}

		return ret;
	}

	double state_ground_dist(ObjectStatePtr a, ObjectStatePtr b)
	{
		return sqrt(pow(a->getElement(0) - b->getElement(0), 2) + 
								pow(a->getElement(2) - b->getElement(2), 2));
	}

	double state_dist(ObjectStatePtr a, ObjectStatePtr b)
	{
		return sqrt(pow(a->getElement(0) - b->getElement(0), 2) 
								+ pow(a->getElement(1) - b->getElement(1), 2) 
								+ pow(a->getElement(2) - b->getElement(2), 2));
	}

	double feat_state_dist(FeatureStatePtr a, FeatureStatePtr b)
	{
		assert(a->getElement(1) == 0.0); assert(b->getElement(1) == 0.0);
		return sqrt(pow(a->getElement(0) - b->getElement(0), 2) 
							+ pow(a->getElement(1) - b->getElement(1), 2) 
							+ pow(a->getElement(2) - b->getElement(2), 2));
	}

	double state_ground_vel_diff(ObjectStatePtr a, ObjectStatePtr b)
	{
		return sqrt(pow(a->getElement(3) - b->getElement(3), 2) + 
								pow(a->getElement(5) - b->getElement(5), 2));
		// return sqrt(pow(a->getVX() - b->getVX(), 2) + pow(a->getVZ() - b->getVZ(), 2));
	}

	void getPairIndex(unsigned int min_idx, unsigned int max_idx, unsigned int &pair_index)
	{
		assert(min_idx < max_idx);
		pair_index = max_idx * (max_idx - 1) / 2 + min_idx;
		assert(pair_index < 10000);
	}

	void getPairFromIndex(unsigned int &min_idx, unsigned int &max_idx, unsigned int num_states, unsigned int pair_index)
	{
		unsigned int temp = 1;
		max_idx = 1;
		while(1) {
			if(pair_index < temp) {
				break;
			}
			pair_index -= temp;
			max_idx++;
			temp++;
		}
		min_idx = pair_index;

		if(!(min_idx < max_idx)) {
			std::cout << "min_idx:" << min_idx << " ";
			std::cout << "max_idx:" << max_idx << " ";
			std::cout << "num_states:" << num_states << " ";
			std::cout << "pair_index:" << pair_index << " ";
			std::cout << std::endl;
			my_assert(0);
		}

		if(!(max_idx < num_states)){
			std::cout << "min_idx:" << min_idx << " ";
			std::cout << "max_idx:" << max_idx << " ";
			std::cout << "num_states:" << num_states << " ";
			std::cout << "pair_index:" << pair_index << " ";
			std::cout << std::endl;
			my_assert(0);
		}
	}

	double getMinDist2Dets(const std::vector<cv::Rect> &dets, int &idx, const cv::Rect &rt, const double &sx, const double &sy, const double &sh)
	{
		double ret = 1000.0f;
		double x1, y1, h1;
		double x2, y2, h2;
		double sx2, sy2, sh2;
		double dist;

		x1 = rt.x + rt.width / 2;
		h1 = rt.height;
		y1 = rt.y + h1 / 2;

		sx2 = h1 * sx;
		sy2 = h1 * sy;
		sh2 = h1 * sh;

		idx = -1;
		for(size_t i = 0; i < dets.size(); i++) {
			x2 = dets[i].x + dets[i].width / 2;
			h2 = dets[i].height;
			y2 = dets[i].y + h2 / 2;
			dist = pow((x1 - x2) / sx2, 2);
			// std::cout << x1 << " - " << x2 << " " << dist << std::endl;
			dist += pow((y1 - y2) / sy2, 2);
			// std::cout << y1 << " - " << y2 << " " << dist << std::endl;
			dist += pow((h1 - h2) / sh2, 2);
			// std::cout << h1 << " - " << h2 << " " << dist << std::endl;
			if(dist < ret) {
				ret = dist;
				idx = i;
			}
		}

		return ret;

	}

	double getMinDist2Dets(const std::vector<cv::Rect> &dets, const cv::Rect &rt, const double &sx, const double &sy, const double &sh) {
		double ret = 1000.0f;
		double x1, y1, h1;
		double x2, y2, h2;
		double sx2, sy2, sh2;
		double dist;

		x1 = rt.x + rt.width / 2;
		h1 = rt.height;
		y1 = rt.y + h1 / 2;

		sx2 = h1 * sx;
		sy2 = h1 * sy;
		sh2 = h1 * sh;

		for(size_t i = 0; i < dets.size(); i++) {
			x2 = dets[i].x + dets[i].width / 2;
			h2 = dets[i].height;
			y2 = dets[i].y + h2 / 2;
			
			dist = pow((x1 - x2) / sx2, 2);
			// std::cout << x1 << " - " << x2 << " " << dist << std::endl;
			dist += pow((y1 - y2) / sy2, 2);
			// std::cout << y1 << " - " << y2 << " " << dist << std::endl;
			dist += pow((h1 - h2) / sh2, 2);
			// std::cout << h1 << " - " << h2 << " " << dist << std::endl;

			if(dist < ret) {
				ret = dist;
			}
		}

		return ret;
	}

	double getDist2AllDets(const std::vector<cv::Rect> &dets, const cv::Rect &rt, const double &sx, const double &sy, const double &sh, const double &th) {
		double ret = 0.0;
		double x1, y1, h1;
		double x2, y2, h2;
		double sx2, sy2, sh2;
		double dist;

		x1 = rt.x + rt.width / 2;
		h1 = rt.height;
		y1 = rt.y + h1 / 2;

		sx2 = h1 * sx;
		sy2 = h1 * sy;
		sh2 = h1 * sh;

		for(size_t i = 0; i < dets.size(); i++) {
			x2 = dets[i].x + dets[i].width / 2;
			h2 = dets[i].height;
			y2 = dets[i].y + h2 / 2;
			
			dist = pow((x1 - x2) / sx2, 2);
			dist += pow((y1 - y2) / sy2, 2);
			dist += pow((h1 - h2) / sh2, 2);

			ret += std::max(th - dist, 0.0);
		}

		return ret;
	}

	double getOverlap2AllDets(const std::vector<cv::Rect> &dets, const cv::Rect &rt, const double &th)
	{
		double ret = 0.0;
		double ob;

		for(size_t i = 0; i < dets.size(); i++) {
			ob = bb_overlap(dets[i], rt);
			ret += std::max(ob - th, 0.0);
		}

		return ret;
	}

	double soft_max(double x, double scale)
	{
		double ret = 0;
		ret = x / scale;
		ret = scale * ret / sqrt(1.0f + ret * ret);
		return ret;
	}
#ifdef MYDEBUG
	static FILE *dbg_fp = NULL;

	void open_dbg_file(const std::string &filename)
	{
		if(dbg_fp) close_dbg_file();
		dbg_fp = fopen(filename.c_str(), "w");
	}

	void print_dbg_file(const char *msg)
	{
		if(dbg_fp) fprintf(dbg_fp, "%s", msg);
		else	   printf("%s", msg);
	}

	void print_dbg_file(const std::string &msg)
	{
		if(dbg_fp) fprintf(dbg_fp, "%s", msg.c_str());
		else	   printf("%s", msg.c_str());
	}

	void close_dbg_file()
	{
		fclose(dbg_fp);
		dbg_fp = NULL;
	}
#endif

	std::vector<std::string> read_file_list(const std::string &filename)
	{
		std::vector<std::string> filelist;
		
		FILE *fp = fopen(filename.c_str(), "r");
		if(fp == NULL) {
			return filelist;
		}

		char line[2000], txt[2000];
		
		int count = 0;
		while(fgets(line, 2000, fp)) {
			sscanf(line, "%s\n", txt);
			filelist.push_back(std::string(txt));
		}
		fclose(fp);

		return filelist;
	}

	void print_rect(cv::Rect &rt)
	{
		std::cout << "x : " << rt.x << " y : " << rt.y << " width : " << rt.width << " height : " << rt.height << std::endl;
	}

	void print_line(const longline &line)
	{
		std::cout << "line : " << line.x1_ << " " << line.x2_ << " "
								<< line.y1_ << " " << line.y2_ << " "
								<< line.angle_ << " " << line.length_ << std::endl;
	}

	void print_matrix(cv::Mat &mat)
	{
		std::cout << "rows : " << mat.rows << ", cols : " << mat.cols << std::endl;
		for(size_t i = 0; i < mat.rows; i++) {
			for(size_t j = 0; j < mat.cols; j++) {
				std::cout << mat.at<double>(i, j) << '\t';
			}
			std::cout << std::endl;
		}
	}

	typedef struct detection_bbox_ {
		cv::Rect 	rt;
		double 		score;
	}detection_bbox;
	
	bool larger_detection_score(const detection_bbox &a, const detection_bbox& b) {
		return (a.score > b.score);
	}

	void nms(std::vector<cv::Rect> &bbs, std::vector<double> &confs)
	{
		std::vector<int> pick;
		std::vector<detection_bbox> buf;
		int i = 0;
		
		for(i = 0; i < confs.size(); i++) {
			detection_bbox temp;

			temp.rt = bbs[i];
			temp.score = confs[i];

			buf.push_back(temp);
			pick.push_back(i);
		}

		// sort the bboxes
		std::sort(buf.begin(), buf.end(), larger_detection_score);

		i = 0;
		int idx, j, iiter;
		while( i < pick.size() ) {
			idx = pick[i];
			std::vector<int> suppress;
			for(j = i + 1; j < pick.size(); j++) {
				if(bb_intunion(buf[idx].rt, buf[j].rt) > 0.3){
					// suppress
					suppress.push_back(j);
				}
			}
			for(j = 0; j < suppress.size(); j++) {
				// std::vector<>::iterator it4 = group_interaction_[i].begin() + idx;
				// group_interaction_[i].erase(it4);
				iiter = suppress[j] - j;
				buf.erase(buf.begin() + iiter);
				pick.erase(pick.begin() + iiter);
			}
			i++;
		}

		bbs.clear(); confs.clear();
		for(i = 0; i < buf.size(); i++) {
			bbs.push_back(buf[i].rt);
			confs.push_back(buf[i].score);
		}
	}
};
