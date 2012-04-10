#ifndef _TARGET_MANAGER_H_
#define _TARGET_MANAGER_H_

#include <common/lines.h>
#include <common/ped_state.h>
#include <common/cam_state.h>
#include <common/gfeat_state.h>
#include <common/mcmc_sample.h>
#include <common/meanshiftTracker.h>

namespace people {
	class Target;

	typedef boost::shared_ptr<Target> TargetPtr;

	enum TargetState{
		TargetTracking = 0,
		TargetTerminated,
		TargetStateNum,
	};

	class Target
	{
	public:
		Target(int tid);
		Target(const Target &target);
		virtual ~Target();

		void setLocation(PeopleStatePtr loc, cv::Rect rt, cv::Mat image);

		inline void setStatus(TargetState status) {status_ = status;}
		inline TargetState getStatus() {return status_;}
	
		double getDistance(PeopleStatePtr state);
		double getOverlap(const cv::Rect &rt);
		double computeColorSim(const cv::Mat &image, const cv::Rect &rt);

		bool writeToFile(std::string filename);

		inline int getNumStates(){return states_.size();}

		int findIdx(double ts);
		inline PeopleStatePtr getState(int idx) {return states_[idx];}
		inline cv::Rect getRect(int idx) {return rts_[idx];}
	//protected:	
		int 													id_;
		TargetState										status_;
		std::vector<PeopleStatePtr> 	states_;
		std::vector<cv::Rect> 				rts_;
		cv::MatND							hist_;
		bool													init_;
#if 0
		cv::Mat												patch_;
#endif
	};

	class Feature;

	typedef boost::shared_ptr<Feature> FeaturePtr;

	class Feature
	{
	public:
		Feature(int fid);
		Feature(const Feature &feat);
		virtual ~Feature();

		void setState(GFeatStatePtr state, cv::Point2f proj, cv::Point2f obs);

		bool writeToFile(std::string filename);

		inline int getNumStates() {return states_.size();}

		int findIdx(double ts);

		inline GFeatStatePtr getState(int idx) {return states_[idx];};
		inline cv::Point2f getImPoint(int idx) {return projs_[idx];};
		inline cv::Point2f getObs(int idx) {return obs_[idx];};

		inline int getID() {return id_;}
	protected:
		int							id_;

		std::vector<cv::Point2f>		obs_;
		std::vector<cv::Point2f>		projs_;

		std::vector<GFeatStatePtr>	states_;
	};

	class Camera
	{
	public:
		Camera();
		virtual ~Camera();
		
		void setState(CamStatePtr cam);

		bool writeToFile(std::string filename);

		int findIdx(double ts);

		inline CamStatePtr getState(int idx) {return states_[idx];};
	protected:
		std::vector<CamStatePtr> states_;
	};
	
	class Lines
	{
	public:
		Lines();
		Lines(const Lines &src);
		Lines(double timesec);

		virtual ~Lines();
		void setTimesec(double timesec);
		double getTimesec();

		void setLines(const std::vector<longline> &lines);
		void setLanes(const std::vector<longline> &lanes);

		std::vector<longline> getLines() { return lines_; }
		std::vector<longline> getLanes() { return lanes_; }

		bool writeLinesToFile(std::ofstream &out);
		bool writeLanesToFile(std::ofstream &out);
	protected:
		std::vector<longline> 	lines_;
		std::vector<longline> 	lanes_;
		double 					timesec_;
	};

	class TargetManager
	{
	public:
		TargetManager();
		TargetManager(const TargetManager &src);
		virtual ~TargetManager();
		
		/*******************************************
		  update target tracking
		 *******************************************/
		void updateTargets(const std::vector<TargetDistPtr> &targetDists, const std::vector<cv::Rect> &rts, const cv::Mat &image);

		void updateFeatures(const std::vector<int> &feat_idx, const std::vector<FeatureDistPtr> &featDists, const std::vector<cv::Point2f> &proj, const std::vector<cv::Point2f> &obs);

		void updateCamera(CamStatePtr cam_hat);
		/*******************************************
		 find match between targets and detections
		 rearrange detections to get proposals
		********************************************/
		void getProposals(const std::vector<PeopleStatePtr> &dets, const cv::Mat &image, std::vector<PeopleStatePtr> &proposals);
		void getProposals(const std::vector<cv::Rect> &dets, const cv::Mat &image, std::vector<cv::Rect> &proposals);

		void setParameters(const std::string &name, const std::string &value);

		void saveAll(std::string &dirname);

		inline std::vector<TargetPtr> getTargets() {return targets_;}

		std::vector<int> getTerminatingTargets(double threshold, cv::Size im_size = cv::Size(640, 480));

		cv::Mat drawTargets(cv::Mat &image_color, double ts);

		cv::Mat drawFeatures(cv::Mat &image, const std::vector<int> &remove_idx, double ts);
	protected:
		void saveAllTargets(std::string &dirname);
		void saveAllFeatures(std::string &dirname);
		void saveAllCamera(std::string &dirname);

		float computeDistance(const TargetPtr target, const PeopleStatePtr det, const cv::Mat &image);
		float computeDistance(const TargetPtr target, const cv::Rect& det, const cv::Mat &image);
		cv::Scalar get_target_color(int id);
	protected:
		std::vector<TargetPtr> 	targets_;
		std::vector<FeaturePtr>	feats_;
		Camera					cam_;
		// Camera					gt_cam_;
		// parameters
		double									max_dist_;
		double									min_overlap_;
	// MeanShift Trackers per target
	public:
		void 										updateMeanShiftTrackers(cv::Mat &image_color);
		void										runMeanShift(cv::Mat &image_color);

		std::vector<cv::Rect>		getMSRects() { return ms_rts_; };
		std::vector<float>		getMSSims() { return ms_sims_; };
	protected:
		std::vector<MSTarget>		ms_targets_;
		std::vector<cv::Rect>		ms_rts_;
		std::vector<float>			ms_sims_;

	public:
		void updateLines(const std::vector<longline> &lines, const std::vector<longline> &lanes, double timestamp);
	protected:
		int	 findLines(double timesec);
		void saveAllLines(std::string &dirname);
		// added for lane detection
		std::vector<Lines>		lines_;
	};
};
#endif // _TARGET_MANAGER_H_
