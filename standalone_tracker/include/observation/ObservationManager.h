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
		virtual double 	getCameraConfidence(CameraStatePtr cam_state);
		virtual double	getObjectConfidence(ObjectStatePtr ped_state, CameraStatePtr cam_state, std::string type = std::string("all"));
		virtual double	getFeatureConfidence(FeatureStatePtr feat_state, int feat_idx, CameraStatePtr cam_state, std::string type = std::string("all"));
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		virtual std::vector<cv::Rect> getDetections();

		virtual CameraStatePtr	initializeCamera(CameraStatePtr cam_state);
		virtual FeatureStatePtr getInitialFeatureState(int idx, CameraStatePtr cam_state);

		// virtual double		getDepthFromState(PeopleStatePtr state, CamStatePtr cam_state);
		// virtual cv::Mat	getObjectConfidenceMap(double y, CameraStatePtr cam_state, std::string type = std::string("all")); 
		virtual std::vector<int> getFeatsIndex() { return gfeats_idx_; };
		virtual std::vector<cv::Point2f> getAllFeats() { return gfeats_; };

		// bool debug_check_projections(PeopleStatePtr state);
		inline double 	getTimeSec() {return time_sec_;};
		inline cv::Mat	getImage() {return img_color_;}
		inline cv::Mat	getImageMono() {return img_mono_;}
		void getHorizonVotes(std::vector<int> &votes, std::vector<double> &stds, double camh);

		// misc functions
		virtual VPEstimator* getVPEstimator() { return &vp_est_; }
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

		bool									has_depth_;
		cv::Mat								img_depth_;
	};
}; // Namespace
#endif // _OBSERVATION_MANAGER_H_
