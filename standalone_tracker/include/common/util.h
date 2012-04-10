#ifndef _UTIL_H_
#define _UTIL_H_
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <common/ped_state.h>
#include <common/gfeat_state.h>
#include <common/cam_state.h>
#include <common/lines.h>

namespace people {
	void show_image(cv::Mat &im, const std::string &name, int max_height);

	bool in_any_rect(const std::vector<cv::Rect> &rts, const cv::Point2f &pt);
	bool inrect(const cv::Rect &rt, const cv::Point2f &pt);

	float bb_overlap(const cv::Rect& rt1, const cv::Rect& rt2);
	double gaussian_prob(double x, double m, double std);
	double log_gaussian_prob(double x, double m, double std);

	double log_gaussian_prob(cv::Mat &x, cv::Mat& m, cv::Mat &icov, double det);

	double state_ground_dist(PeopleStatePtr a, PeopleStatePtr b); // x-z distance
	double state_dist(PeopleStatePtr a, PeopleStatePtr b);
#ifdef VEL_STATE
	double state_ground_vel_diff(PeopleStatePtr a, PeopleStatePtr b);
#endif
	double feat_state_dist(GFeatStatePtr a, GFeatStatePtr b);

	void getPairIndex(unsigned int min_idx, unsigned int max_idx, unsigned int &pair_index);
	void getPairFromIndex(unsigned int &min_idx, unsigned int &max_idx, unsigned int num_states, unsigned int pair_index);
	double getMinDist2Dets(const std::vector<cv::Rect> &dets, const cv::Rect &rt, const double &sx, const double &sy, const double &sh);
	double getMinDist2Dets(const std::vector<cv::Rect> &dets, int &idx, const cv::Rect &rt, const double &sx, const double &sy, const double &sh);
	double getDist2AllDets(const std::vector<cv::Rect> &dets, const cv::Rect &rt, const double &sx, const double &sy, const double &sh, const double &th);
	double getOverlap2AllDets(const std::vector<cv::Rect> &dets, const cv::Rect &rt, const double &th);
	double soft_max(double x, double scale);

	std::vector<std::string> read_file_list(const std::string &filename);

	void print_rect(cv::Rect &rt);
	void print_line(const longline &line);
	void print_matrix(cv::Mat &mat);
#ifdef MYDEBUG
#define my_assert(x) assert(x)
	void open_dbg_file(const std::string &filename);
	void print_dbg_file(const char *msg);
	void print_dbg_file(const std::string &msg);
	void close_dbg_file();
#else
#define my_assert(x) 
#define open_dbg_file(x) 
#define print_dbg_file(x) 
#define close_dbg_file() 
#endif
};
#endif // _UTIL_H_
