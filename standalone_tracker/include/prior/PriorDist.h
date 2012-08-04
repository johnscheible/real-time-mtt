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

#ifndef _PRIOR_DIST_H_
#define _PRIOR_DIST_H_

#include <common/ped_state.h>
#include <common/mcmc_sample.h>

namespace people {
	class PriorDist
	{
	public:
		PriorDist() {
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
		};

		PriorDist(PosteriorDistPtr prior, double timestamp):prev_dist_(prior),timestamp_(timestamp) {
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
			object_motion_params_.push_back(0.0);
		};

		virtual ~PriorDist() {};
		virtual void setParameters(const std::string &name, const std::string &value);
		virtual void initCache(MCMCSamplePtr sample);
		virtual double computeLogMotionPrior(const SampleInfo &info);
		virtual void updateCache(const SampleInfo &info, bool accepted);

		virtual void computeNewSampletMotionPrior(const SampleInfo &info);

		// for debugging
		virtual void print_all_cache(const SampleInfo &info, MCMCSamplePtr sample);
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
		std::vector<double> object_motion_params_;

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
			interaction_on_ = true;
		};

		PriorDistInteract(PosteriorDistPtr prior, double timestamp){ 
			prev_dist_type_ = "independent"; 
			repulsion_const_ = 1; 
			interaction_transition_prob_ = 0.95;
			prev_dist_ = prior; 
			timestamp_ = timestamp; 
			interaction_on_ = true;
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
		virtual double computePairwiseInteractionPotential(ObjectStatePtr state1, ObjectStatePtr state2, bool isgroup);

		double repulsion_const_;
		double group_const_;
		double group_slope_sigmoid_;
		double group_threshold_sigmoid_;

		double interaction_transition_prob_;

		bool interaction_on_;

		cv::Mat one_interaction_mode_prior_cache_;
		cv::Mat interaction_mode_cache_;
	};

	class PriorDistCameraEstimate : public PriorDistInteract
	{
	public:
		PriorDistCameraEstimate():PriorDistInteract() {
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
		};

		PriorDistCameraEstimate(PosteriorDistPtr prior, double timestamp):PriorDistInteract(prior, timestamp) {
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
			camera_motion_params_.push_back(0.0);
		};

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
		std::vector<double> camera_motion_params_;
		
		double prob_feat_enter_;
		double prob_feat_stay_;

		cv::Mat new_camera_prior_cache_;
		cv::Mat camera_prior_cache_;
		cv::Mat one_feature_prior_cache_;
		cv::Mat feature_prior_cache_;
	};
}; // Namespace
#endif // _PRIOR_DIST_H_
