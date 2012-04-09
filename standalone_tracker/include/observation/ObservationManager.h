#ifndef _OBSERVATION_MANAGER_H_
#define _OBSERVATION_MANAGER_H_

#include <common/ped_state.h>
#include <common/cam_state.h>
#include <common/gfeat_state.h>
#include <observation/ObservationNode.h>
#include <common/FeatTracker.h>
#include <common/vpestimator.h>

namespace people {
	class ObservationManager
	{
	public:
		ObservationManager();
		virtual ~ObservationManager();

		void 		releaseNodes();
		
		void 		setObjType(ObjectType type);
		void 								insertObservationNode(ObservationNode* node);
		ObservationNode 		*getObservationNode(std::string &type);
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// these two are totally unsafe!!! be cautious!!
		virtual void	setData(const void *data, const std::string &type);
		virtual void	quaryData(const std::string &name, void *data);

		virtual void		setParameters(const std::string &name, const std::string &value);
		virtual void 		preprocess();
		virtual std::vector<int> preprocessFeats(const std::vector<int>& prev_feats_idx, const int max_feats, const std::vector<cv::Rect> &targets = std::vector<cv::Rect>());
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// getConfidence
		virtual double 	getCameraConfidence(CamStatePtr cam_state);
		virtual double	getPeopleConfidence(PeopleStatePtr ped_state, CamStatePtr cam_state, std::string type = std::string("all"));
		virtual double	getGFeatConfidence(GFeatStatePtr feat_state, int feat_idx, CamStatePtr cam_state, std::string type = std::string("all"));
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual std::vector<cv::Rect> getDetections();

		virtual GFeatStatePtr getInitialGFeatState(int idx, CamStatePtr cam_state);
		// virtual double		getDepthFromState(PeopleStatePtr state, CamStatePtr cam_state);
		virtual cv::Mat	getPeopleConfidenceMap(double y, CamStatePtr cam_state, std::string type = std::string("all")); 

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// camera projection functions
		virtual cv::Rect getRectFromState(PeopleStatePtr ped_state, CamStatePtr cam_state) { return cam_state->project(ped_state); }
		virtual cv::Point2f getPointFromState(GFeatStatePtr feat_state, CamStatePtr cam_state) { return cam_state->project(feat_state); }
		virtual PeopleStatePtr getPeopleStateFromRect(cv::Rect &rt, CamStatePtr cam_state) { return cam_state->iproject(rt); }
		virtual GFeatStatePtr getGFeatStateFromPoint(cv::Point2f &pt, CamStatePtr cam_state) { return cam_state->iproject(pt); }
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual std::vector<int> getFeatsIndex() { return gfeats_idx_; };
		virtual std::vector<cv::Point2f> getAllFeats() { return gfeats_; };

		// bool debug_check_projections(PeopleStatePtr state);
		inline double 	getTimeSec() {return time_sec_;};
		inline cv::Mat	getImage() {return img_color_;}
		inline cv::Mat	getImageMono() {return img_mono_;}

		void getHorizonVotes(std::vector<int> &votes, std::vector<double> &stds, double camh);
	protected:
		cv::Mat															img_color_;
		cv::Mat															img_mono_;
		FeatTracker*												feat_tracker_;

		VPEstimator														vp_est_;

		double															time_sec_;
		double															total_weight_;

		double															min_height_;
		double															max_height_;
		double															obs_lkhood_out_of_height_;

		std::vector<ObservationNode*> 			nodes_;
		std::vector<cv::Point2f> 						gfeats_;
		std::vector<int>		 								gfeats_idx_;

		double 															gfeat_sigma_u_; 
		double															gfeat_sigma_v_; 

		ObjectType 														obj_type_;

		double 								mean_horizon_;
		double 								std_horizon_;
	};
}; // Namespace
#endif // _OBSERVATION_MANAGER_H_
