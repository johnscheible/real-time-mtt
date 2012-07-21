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

#ifndef _RJMCMC_PROPOSAL_H_
#define _RJMCMC_PROPOSAL_H_

#include <opencv/cv.h>
#include <common/ped_state.h>
#include <common/cam_state.h>
#include <common/gfeat_state.h>
#include <common/mcmc_sample.h>

namespace people {
	class RJMCMCProposal
	{
	public:
		RJMCMCProposal(PosteriorDistPtr prior, double tsec):rng_((double)time(NULL)),prev_dist_(prior),timestamp_(tsec)
		{
			// default paramters..
			move_prob_[MoveAdd] = 0.05 * 0.6;
			move_prob_[MoveDelete] = 0.05 * 0.6;
			move_prob_[MoveStay] = 0.1 * 0.6;
			move_prob_[MoveLeave] = 0.1 * 0.6;
			move_prob_[MoveUpdate] = 0.6 * 0.6;
			move_prob_[MoveInteractionFlip] = 0.1 * 0.6;

			move_prob_[MoveCamUpdate] = 0.0;//0.1;
			move_prob_[MoveFeatAdd] = 0.0;//0.1 * 0.3;
			move_prob_[MoveFeatDelete] = 0.0;//0.1 * 0.3;
			move_prob_[MoveFeatStay] = 0.0;//0.1 * 0.3;
			move_prob_[MoveFeatLeave] = 0.0;//0.1 * 0.3;
			move_prob_[MoveFeatUpdate] = 0.0;//0.6 * 0.3;
#ifdef VEL_STATE
			det_sigma_x_ = 0.05;
			det_sigma_y_ = 0.1;
			det_sigma_h_ = 0.1;

			pert_sigma_x_ = 0.1;
			pert_sigma_y_ = 0.1;
			pert_sigma_z_ = 0.1;

			pert_sigma_vx_ = 0.5;
			pert_sigma_vz_ = 0.1;

			motion_sigma_x_ = 0.4;
			motion_sigma_y_ = 0.4;
			motion_sigma_z_ = 0.4;

			motion_sigma_vx_ = 3;
			motion_sigma_vz_ = .6;
#else
			det_sigma_x_ = 0.05;
			det_sigma_y_ = 0.1;
			det_sigma_h_ = 0.1;

			sigma_x_ = 0.03;
			sigma_y_ = 0.03;
			sigma_z_ = 0.02;
#endif
 			pert_cam_sigma_x_ = 0.2;
 			pert_cam_sigma_z_ = 0.05;
 			pert_cam_sigma_yaw_ = 0.03;
 			pert_cam_sigma_v_ = 0.03;
 			pert_cam_sigma_horizon_ = 0.03;

			prob_interaction_flip_ = 0.9;
			min_frames_group_ = 7;
		};

		virtual 	~RJMCMCProposal() {};

		virtual void	setParameters(const std::string &name, const std::string &value);
		
		//
		virtual void initCache(MCMCSamplePtr sample);
		virtual void updateCache(const SampleInfo &info, bool accepted);
		virtual void setDetections(const std::vector<cv::Rect> &dets){dets_ = dets;};
		virtual SampleInfo drawNewSample(MCMCSamplePtr sample);
		virtual double getProposalLogLkhood(const MCMCSamplePtr sample, const SampleInfo &info); // for acceptance ratio computation

		// debug
		virtual void 				dbgCheckCache(MCMCSamplePtr sample);
		virtual void 				printCache();
	protected:
		virtual double 				stateProposalStay(MCMCSamplePtr sample, PeopleStatePtr state, int idx);
		// each proposal moves
		virtual SampleInfo			addTarget(MCMCSamplePtr sample);
		virtual SampleInfo			deleteTarget(MCMCSamplePtr sample);
		virtual SampleInfo 			stayTarget(MCMCSamplePtr sample);
		virtual SampleInfo			leaveTarget(MCMCSamplePtr sample);
		virtual SampleInfo			updateTarget(MCMCSamplePtr sample);
		virtual SampleInfo			flipInteraction(MCMCSamplePtr sample);
		virtual SampleInfo			updateCamera(MCMCSamplePtr sample);
		virtual SampleInfo			stayFeature(MCMCSamplePtr sample);
		virtual SampleInfo			leaveFeature(MCMCSamplePtr sample);
		virtual SampleInfo			updateFeature(MCMCSamplePtr sample);
	protected:
		// random value generator
		cv::RNG					rng_;
		// previous time posterior
		PosteriorDistPtr		prev_dist_;
		// detection proposals
		std::vector<cv::Rect> dets_;
		// 
		double timestamp_;
		// set cache..
		std::vector<int>	set_add_;
		std::vector<int>	set_delete_;
		std::vector<int>	set_stay_;
		std::vector<int>	set_leave_;

		std::vector<int>	set_feat_stay_;
		std::vector<int>	set_feat_leave_;

		// parameters
		double					move_prob_[MoveNums];
#ifdef VEL_STATE
		double					motion_sigma_x_;
		double					motion_sigma_y_;
		double					motion_sigma_z_;
		double					motion_sigma_vx_;
		// double					motion_sigma_vy_;
		double					motion_sigma_vz_;

		double					pert_sigma_x_;
		double					pert_sigma_y_;
		double					pert_sigma_z_;
		double					pert_sigma_vx_;
		// double					pert_sigma_vy_;
		double					pert_sigma_vz_;
#else
		double					sigma_x_;
		double					sigma_y_;
		double					sigma_z_;
#endif
		double					det_sigma_x_;
		double					det_sigma_y_;
		double					det_sigma_h_;

		double					pert_cam_sigma_x_;
		double					pert_cam_sigma_z_;
		double					pert_cam_sigma_yaw_;
		double					pert_cam_sigma_v_;
		double					pert_cam_sigma_horizon_;

		double					pert_feat_sigma_x_;
		double					pert_feat_sigma_z_;

		double					prob_interaction_flip_;
		int							min_frames_group_;
	};
};
#endif // _RJMCMC_PROPOSAL_H_
