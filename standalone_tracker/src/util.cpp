#include <stdio.h>
#include <stdlib.h>
#include <common/util.h>

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

	double gaussian_prob(double x, double m, double std)
	{
		double var = std * std;
		return 1 / sqrt(2 * M_PI * var) * exp(- pow(x - m, 2) / (2 * var));
	}

	double log_gaussian_prob(double x, double m, double std)
	{
		return -log(sqrt(2 * M_PI) * std) - ( pow((x - m) / std, 2) / 2.0 );
	}
	
	double log_gaussian_prob(cv::Mat &x, cv::Mat& m, cv::Mat &icov, double det)
	{
		my_assert(x.rows == m.rows);
		my_assert(x.rows == icov.cols);
		
		cv::Mat dx = x - m;
		cv::Mat temp = ( (dx.t() * icov) * dx  / 2.0 );
		
		my_assert(temp.rows == 1 && temp.cols == 1);

		double ret =  - x.rows * log(2 * M_PI) / 2 
			   - log(det) / 2  - temp.at<double>(0, 0);

		return ret;
	}

	double state_ground_dist(PeopleStatePtr a, PeopleStatePtr b)
	{
		return sqrt(pow(a->getX() - b->getX(), 2) + pow(a->getZ() - b->getZ(), 2));
	}

	double state_dist(PeopleStatePtr a, PeopleStatePtr b)
	{
		return sqrt(pow(a->getX() - b->getX(), 2) + pow(a->getY() - b->getY(), 2) + pow(a->getZ() - b->getZ(), 2));
	}

	double feat_state_dist(GFeatStatePtr a, GFeatStatePtr b)
	{
		assert(a->getY() == 0.0); assert(b->getY() == 0.0);
		return sqrt(pow(a->getX() - b->getX(), 2) + pow(a->getY() - b->getY(), 2) + pow(a->getZ() - b->getZ(), 2));
	}
#ifdef VEL_STATE
	double state_ground_vel_diff(PeopleStatePtr a, PeopleStatePtr b)
	{
		return sqrt(pow(a->getVX() - b->getVX(), 2) + pow(a->getVZ() - b->getVZ(), 2));
	}
#endif
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
};
