#ifndef _PRIOR_DIST_H_
#define _PRIOR_DIST_H_

#include <common/ped_state.h>
#include <common/mcmc_sample.h>

namespace people {
	class PriorDist
	{
	public:
		PriorDist() {};
		PriorDist(PosteriorDistPtr prior, double timestamp):prev_dist_(prior),timestamp_(timestamp) {};
		virtual ~PriorDist() {};
		virtual void setParameters(const std::string &name, const std::string &value);
		virtual void initCache(MCMCSamplePtr sample);
		virtual double computeLogMotionPrior(const SampleInfo &info);
		virtual void updateCache(const SampleInfo &info, bool accepted);

		virtual void computeNewSampletMotionPrior(const SampleInfo &info);

		// for debugging
		virtual void print_all_cache(const SampleInfo &info, MCMCSamplePtr sample);
#ifdef VEL_STATE
		virtual void initMotionParameters();
#endif
	protected:
		virtual void computeOneTargetMotionPrior(MCMCSamplePtr sample, int tid);
	protected:
		// previous time posterior
		PosteriorDistPtr		prev_dist_;
		//
		double timestamp_;
		// full prior cache 
		cv::Mat motion_cache_;
		// one target prior cache : will be discarded if not accepted
		cv::Mat one_target_motion_cache_;
		// parameters
#ifdef VEL_STATE
		double motion_sigma_x_;
		double motion_sigma_y_;
		double motion_sigma_z_;

		double motion_sigma_vx_;
		double motion_sigma_vy_;
		double motion_sigma_vz_;

		std::vector<cv::Mat> motion_sigma_invs_;
		std::vector<double> motion_sigma_dets_;
#else
		double motion_sigma_x_;
		double motion_sigma_y_;
		double motion_sigma_z_;
#endif
		double prob_stay_;
		double prob_enter_;
	};

	class PriorDistNS : public PriorDist
	{
	public:
		PriorDistNS() { 
			prev_dist_type_ = "dependent"; 
		};

		PriorDistNS(PosteriorDistPtr prior, double timestamp){ 
			prev_dist_type_ = "dependent"; 
			prev_dist_ = prior; 
			timestamp_ = timestamp; 
		};

		virtual ~PriorDistNS() {};
		
		virtual void setPrevDistType(const std::string &type) { prev_dist_type_ = type; }
		virtual double computeLogMotionPrior(const SampleInfo &info);
		virtual void computeNewSampletMotionPrior(const SampleInfo &info);
	protected:
		virtual void computeOneTargetMotionPrior(MCMCSamplePtr sample, int tid);

		std::string prev_dist_type_;
	};

	class PriorDistInteract : public PriorDistNS
	{
	public : 
		PriorDistInteract() { 
			prev_dist_type_ = "independent"; 
			repulsion_const_ = 1; 
			interaction_transition_prob_ = 0.95;
		};

		PriorDistInteract(PosteriorDistPtr prior, double timestamp){ 
			prev_dist_type_ = "independent"; 
			repulsion_const_ = 1; 
			interaction_transition_prob_ = 0.95;
			prev_dist_ = prior; 
			timestamp_ = timestamp; 
		};

		virtual ~PriorDistInteract() {};

		virtual void setParameters(const std::string &name, const std::string &value);

		virtual double computeLogMotionPrior(const SampleInfo &info, MCMCSamplePtr sample);

		virtual void computeNewSampletMotionPrior(const SampleInfo &info);

		virtual void initCache(MCMCSamplePtr sample);

		virtual void updateCache(const SampleInfo &info, bool accepted);
	protected:
		virtual int getInteractionVariableIndex(int min_idx, int max_idx) {
			unsigned int ret = 0;
			getPairIndex((unsigned int)min_idx, (unsigned int)max_idx, ret);
			return ret;
		};

		virtual cv::Mat computeAllTargetMotionPrior(const SampleInfo &info, MCMCSamplePtr sample);
		virtual cv::Mat computeAllInteractionModePrior(const SampleInfo &info, MCMCSamplePtr sample);

		virtual void computeNewInteractionModePrior(const SampleInfo &info);
		virtual double computeInteractionPotential(const SampleInfo &info, MCMCSamplePtr sample);
		virtual double computePairwiseInteractionPotential(PeopleStatePtr state1, PeopleStatePtr state2, bool isgroup);

		double repulsion_const_;
		double group_const_;
		double group_slope_sigmoid_;
		double group_threshold_sigmoid_;

		double interaction_transition_prob_;

		cv::Mat one_interaction_mode_prior_cache_;
		cv::Mat interaction_mode_cache_;
	};

	class PriorDistCameraEstimate : public PriorDistInteract
	{
	public:
		PriorDistCameraEstimate():PriorDistInteract() {};
		PriorDistCameraEstimate(PosteriorDistPtr prior, double timestamp):PriorDistInteract(prior, timestamp) {};
		virtual ~PriorDistCameraEstimate() {};

		virtual void initCache(MCMCSamplePtr sample);
		virtual void updateCache(const SampleInfo &info, bool accepted);
		virtual void setParameters(const std::string &name, const std::string &value);
		virtual double computeLogMotionPrior(const SampleInfo &info, MCMCSamplePtr sample);
		virtual void computeNewSampletMotionPrior(const SampleInfo &info);
	protected:
		virtual cv::Mat computeAllFeatureMotionPrior(const SampleInfo &info, MCMCSamplePtr sample);
		virtual cv::Mat computeCameraMotionPrior(const SampleInfo &info, MCMCSamplePtr sample);

		virtual void computeNewFeaturePrior(const SampleInfo &info);
		virtual void computeNewCameraPrior(const SampleInfo &info);

		// parameters
		double camera_motion_sigma_x_;
		double camera_motion_sigma_z_;
		double camera_motion_sigma_yaw_;
		double camera_motion_sigma_v_;
		double camera_motion_sigma_horizon_;
		
		double prob_feat_enter_;
		double prob_feat_stay_;

		cv::Mat new_camera_prior_cache_;
		cv::Mat camera_prior_cache_;
		cv::Mat one_feature_prior_cache_;
		cv::Mat feature_prior_cache_;
	};
}; // Namespace
#endif // _PRIOR_DIST_H_
