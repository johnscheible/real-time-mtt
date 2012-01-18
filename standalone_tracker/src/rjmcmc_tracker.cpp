#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <tracker/rjmcmc_tracker.h>

using namespace people;

void RJMCMCTracker::setParameters(const std::string &name, const std::string &value)
{
	///////////////////////////////////////////////////////
	// sampling
  if (name == "MCMCNumSamples")
		num_samples_ = boost::lexical_cast<int>(value);
  else if (name == "MCMCBurnin")
		burnin_ = boost::lexical_cast<int>(value);
  else if (name == "MCMCThinning")
		thinning_ = boost::lexical_cast<int>(value);
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	// target motion
	else if (name == "MotionSigmaX")
		motion_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaY")
		motion_sigma_y_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaZ")
		motion_sigma_z_ = boost::lexical_cast<double>(value);
#ifdef VEL_STATE
	else if (name == "MotionSigmaVX")
		motion_sigma_vx_ = boost::lexical_cast<double>(value);
	else if (name == "MotionSigmaVZ")
		motion_sigma_vz_ = boost::lexical_cast<double>(value);
#endif
	else if (name == "PerturbSigmaX")
		perturb_x_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaY")
		perturb_y_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaZ")
		perturb_z_ = boost::lexical_cast<double>(value);
#ifdef VEL_STATE
	else if (name == "PerturbSigmaVX")
		perturb_vx_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbSigmaVZ")
		perturb_vz_ = boost::lexical_cast<double>(value);
#endif
	else if (name == "ProbStay")
		prob_stay_ = boost::lexical_cast<double>(value);
	else if (name == "ProbEnter")
		prob_enter_ = boost::lexical_cast<double>(value);
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	// camera motion
	else if (name == "CameraMotionSigmaX")
		motion_cam_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "CameraMotionSigmaZ")
		motion_cam_sigma_z_ = boost::lexical_cast<double>(value);
	else if (name == "CameraMotionSigmaYaw")
		motion_cam_sigma_yaw_ = boost::lexical_cast<double>(value);
	else if (name == "CameraMotionSigmaV")
		motion_cam_sigma_v_ = boost::lexical_cast<double>(value);
	else if (name == "CameraMotionSigmaHorizon")
		motion_cam_sigma_horizon_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaX")
		perturb_cam_x_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaZ")
		perturb_cam_z_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaYaw")
		perturb_cam_yaw_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaV")
		perturb_cam_v_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbCamSigmaHorizon")
		perturb_cam_horizon_ = boost::lexical_cast<double>(value);
	///////////////////////////////////////////////////////

	///////////////////////////////////////////////////////
	// feature
	else if (name == "PerturbFeatSigmaX")
		perturb_feat_x_ = boost::lexical_cast<double>(value);
	else if (name == "PerturbFeatSigmaZ")
		perturb_feat_z_ = boost::lexical_cast<double>(value);
	else if (name == "ProbFeatStay")
		prob_feat_stay_ = boost::lexical_cast<double>(value);
	else if (name == "ProbFeatEnter")
		prob_feat_enter_ = boost::lexical_cast<double>(value);
	///////////////////////////////////////////////////////
	// proposal parameters
	else if (name == "ProbMoveAdd")	
		prob_move_add_ = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveDelete")
		prob_move_delete_ = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveStay")
		prob_move_stay_= boost::lexical_cast<double>(value);
	else if (name == "ProbMoveLeave")
		prob_move_leave_ = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveUpdate")
		prob_move_update_ = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveFeatStay")
		prob_move_feat_stay_ = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveFeatLeave")
		prob_move_feat_leave_ = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveFeatUpdate")
		prob_move_feat_update_ = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveCamUpdate")
		prob_move_cam_update_ = boost::lexical_cast<double>(value);
	else if (name == "ProbMoveInteractionFlip")
		prob_move_interaction_flip_ = boost::lexical_cast<double>(value);
	///////////////////////////////////////////////////////
	///////////////////////////////////////////////////////
	// general
	else if (name == "MaximumGFeats")
		max_gfeats_ = boost::lexical_cast<int>(value);
	else if (name == "EstimateCamera")
		estimate_camera_ = boost::lexical_cast<bool>(value);
	else if (name == "DetectionSigmaX")
		detection_sigma_x_ = boost::lexical_cast<double>(value);
	else if (name == "DetectionSigmaY")
		detection_sigma_y_ = boost::lexical_cast<double>(value);
	else if (name == "DetectionSigmaH")
		detection_sigma_h_ = boost::lexical_cast<double>(value);
	else if (name == "ShowDebugMsg")
		show_dbg_msg_ = boost::lexical_cast<bool>(value);
	///////////////////////////////////////////////////////
	else
		std::cout << "RJMCMCTracker : not defined parameter " << name << ":" << value << std::endl;

	if(show_dbg_msg_)	std::cout << "Set RJMCMCTracker Param " << name << " : " << value << std::endl;
}

MCMCSamplePtr RJMCMCTracker::getInitSample(double timesec)
{
	MCMCSamplePtr ret = boost::make_shared<MCMCSample>(MCMCSample());
	ObservationManager *mgr = obs_wrapper_.getManager();
	
	//////////////////////////////////////////////////////////////////////
	// initialize camera
	CamStatePtr cam_state;
	if(estimate_camera_) {
		cam_state = prev_dist_->getMeanCamera(); // initialize by previous camera
		// first frame :)
		if(cam_state.get() == NULL) {
			assert(init_cam_.get());
			cam_state = init_cam_->clone();
			cam_state->setTS(timesec);
		}
		else {
			cam_state = initializeCamera();
			// cam_state = cam_state->predict(timesec);
		}
	}
	else {
		assert(init_cam_.get());
		cam_state = init_cam_->clone();
		cam_state->setTS(timesec);
	}
	ret->setCamState(cam_state);

	assert(cam_state.get());
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
	// initialize targets
	//////////////////////////////////////////////////////////////////////
	std::vector<PeopleStatePtr> states;
	std::vector<bool> exists;
	std::vector<TargetDistPtr> targets = prev_dist_->getTargetDists();
	for(unsigned int i = 0; i < proposals_.size(); i++) {
		PeopleStatePtr state;
		bool exist = false;

		exist = true;
		if(proposals_[i].width != 0 && proposals_[i].height != 0) { // exist a proposal
			state = cam_state->iproject(proposals_[i]); // mgr->getPeopleStateFromRect(proposals_[i], cam_state);
#ifdef VEL_STATE
			if(i < targets.size()) {
				PeopleStatePtr prev_state = targets[i]->getMean();

				state->setVX(prev_state->getVX() / 2);
				state->setVY(0.0);
				state->setVZ(prev_state->getVZ() / 2);
			}
#endif
		}
		else {
			assert(i < targets.size());
			state = targets[i]->getMean()->clone();
#ifdef VEL_STATE
			state = state->predict(timesec);

			state->setVX(state->getVX() / 2);
			state->setVY(0.0);
			state->setVZ(state->getVZ() / 2);
#endif
		}
		state->setTS(timesec);

		states.push_back(state);
		exists.push_back(exist);
	}
	ret->setStates(states, exists);
	//////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////
	// initialize interaction
	//////////////////////////////////////////////////////////////////////
	std::vector< std::vector<bool> > group_interaction;
	for(unsigned int i = 0; i < proposals_.size(); i++) {
		std::vector<bool> one_col;
		for(unsigned int j = 0; j < i; j++) {
			if(i < (unsigned int)prev_dist_->getNumTargets() && j < (unsigned int)prev_dist_->getNumTargets()) {
				one_col.push_back(prev_dist_->getMeanInteractionMode(j, i) > 0.5);
			}
			else {
				one_col.push_back(false); // by default no group interaction
			}
		}
		group_interaction.push_back(one_col);
	}
	ret->setInteractionMode(group_interaction);
#ifdef CAM_EST
	//////////////////////////////////////////////////////////////////////
	// set initial features
	//////////////////////////////////////////////////////////////////////
	std::vector<GFeatStatePtr> feats;
	std::vector<bool> validities;
	for(size_t i = 0; i < feat_idx_.size(); i++) {
		GFeatStatePtr fstate;
		bool valid = false;
		if(i < (size_t)prev_dist_->getNumFeats()) {
			// from previous distribution
			fstate = prev_dist_->drawFeatSample(i, timesec);

			if(fstate.get())		valid = true;
			else								valid = false;
		}
		else {
			// new feature
			fstate = mgr->getInitialGFeatState(feat_idx_[i], cam_state);
			valid = true;
		}
		fstate->setTS(timesec);
		feats.push_back(fstate);
		validities.push_back(valid);
	}
	ret->setGFeatStates(feats, validities);
	//////////////////////////////////////////////////////////////////////
#endif

	// std::cout << "initial sample information : " << std::endl;
	// ret->print();
	//////////////////////////////////////////////////////////////////////
	// initial lkhood, not being set here...
	//////////////////////////////////////////////////////////////////////
	return ret;
}

void RJMCMCTracker::filterTargets(std::vector<int> &remove_idx)
{
	// remove targets!!!
	for(size_t i = 0; i < remove_idx.size(); i++) {
		prev_dist_->removeTarget(remove_idx[i] - i);
	}
}

void RJMCMCTracker::filterFeatures(std::vector<int> &remove_idx)
{
	// remove targets!!!
	for(size_t i = 0; i < remove_idx.size(); i++) {
		prev_dist_->removeFeature(remove_idx[i] - i);

		std::vector<int>::iterator it = feat_idx_.begin() + remove_idx[i] - i;
		feat_idx_.erase(it);
	}

	my_assert(prev_dist_->getNumFeats() == (int)feat_idx_.size());
}

void RJMCMCTracker::setDefaultParameters()
{
	show_dbg_msg_ = false;
	num_samples_ = 3000; burnin_ = 500; thinning_ = 50;

	prob_enter_ = 0.1;	
	prob_stay_ = 0.9;

	prob_feat_enter_ = 0.1;	
	prob_feat_stay_ = 0.9;
	//////////////////////////////////////////
	prob_move_add_ = 0.05;	
	prob_move_delete_ = 0.05;	
	prob_move_stay_ = 0.1;	
	prob_move_leave_ = 0.1; 
	prob_move_update_ = 0.30;
	prob_move_feat_stay_ = 0.05;
	prob_move_feat_leave_ = 0.05;
	prob_move_feat_update_ = 0.10;
	prob_move_cam_update_ = 0.1;
	prob_move_interaction_flip_ = 0.1;
	/////////////////////////////////////////
#ifdef CAM_EST
	max_gfeats_ = 40;
#endif
	detection_sigma_x_ = 0.05; detection_sigma_y_ = 0.1; detection_sigma_h_ = 0.1;
	/////////////////////////////////////////
	motion_sigma_x_ = 0.9;	// 0.9/30 => 3cm
	motion_sigma_y_ = 0.3;
	motion_sigma_z_ = 0.9;
	perturb_x_ = 0.3; 
	perturb_y_ = 0.1; 	
	perturb_z_ = 0.3;
#ifdef VEL_STATE
	motion_sigma_vx_ = 3.;	
	motion_sigma_vz_ = 3.;
	perturb_vx_ = 1.5; 
	perturb_vz_ = 1.5;
#endif
	/////////////////////////////////////////

	/////////////////////////////////////////
	motion_cam_sigma_x_ = 0.9;
	motion_cam_sigma_z_ = 0.9;
	motion_cam_sigma_yaw_ = 0.1;
	motion_cam_sigma_v_ = 3.0;
	motion_cam_sigma_horizon_ = 3.0; // pixel

	perturb_cam_x_ = 0.3;
	perturb_cam_z_ = 0.3;
	perturb_cam_yaw_ = 0.03;
	perturb_cam_v_ = 1.0;
	perturb_cam_horizon_ = 1.0; // pixel
	/////////////////////////////////////////

	/////////////////////////////////////////
	perturb_feat_x_ = 0.01; // not dependent on time
	perturb_feat_z_ = 0.01;
	/////////////////////////////////////////
	estimate_camera_ = true;
}

// #define SAVE_ALL_SAMPLES
#ifdef SAVE_ALL_SAMPLES
void save_sample_info(FILE *fp, int idx, SampleInfo &info, MCMCSamplePtr sample, double ar, double obs_ar, double prior_ar, double q_ar, bool accepted)
{
	fprintf(fp, "%d\t%d\t%.10f\t%.10f\t%.10f\t%.10f\t%d\t%d\t", idx, accepted, ar, obs_ar, prior_ar, q_ar, info.type_, info.idx_);

	if(info.state_.get() != NULL) {
		fprintf(fp, "%.10f\t%.10f\t%.10f\t", info.state_->x_, info.state_->y_, info.state_->z_);
	}
	else {
		fprintf(fp, "%d\t%d\t%d\t", 0, 0, 0);
	}

	for(size_t i = 0; i < sample->getNumTargets(); i++) {
		PeopleStatePtr temp = sample->getState(i);
		if(temp.get() != NULL)
			fprintf(fp, "%.10f\t%.10f\t%.10f\t", temp->x_, temp->y_, temp->z_);
		else
			fprintf(fp, "%d\t%d\t%d\t", 0, 0, 0);
	}

	fprintf(fp, "\n");
}
#if 0
	// target match visualization routine
	std::vector<TargetDistPtr> targets = prev_dist_->getTargetDists();
	ObservationManager *mgr = obs_wrapper_.getManager();
	cv::Mat image_temp = mgr->getImage().clone();
	
	for(size_t i = 0; i < targets.size(); i++) {
		std::cout << "Target[" << i << "] state" << std::endl;
		PeopleStatePtr mstate = targets[i]->getMean();
		mstate->print();

		cv::Rect rt = mgr->getRectFromState(mstate);
		cv::Scalar color = cv::Scalar((70 * i) % 256, (60 * i) % 256, (50 * i) % 256);

		cv::rectangle(image_temp, rt.tl(), rt.br(), color, 2);
		if(proposals_[i].get() != NULL) {
			PeopleStatePtr temp_proposal = proposals_[i];
			std::cout << "Target[" << i << "] proposal!!!!!!" << std::endl;
			temp_proposal->print();
			rt = mgr->getRectFromState(temp_proposal);
			cv::rectangle(image_temp, rt.tl(), rt.br(), color, 2);
		}
	}
	cv::imshow("dbg_mathc", image_temp);
	cv::waitKey(20);

	for(size_t i = 0; i < targets.size(); i++) {
		std::cout << "Target[" << i << "] state" << std::endl;
		targets[i]->getMean()->print();
		if(proposals_[i].get() != NULL) {
			std::cout << "Target[" << i << "] proposal!!!!!!" << std::endl;
			proposals_[i]->print();

			if(state_dist(proposals_[i], targets[i]->getMean()) > 0.2) {
				std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
				cv::waitKey();
			}
		}
	}
#endif
#endif

void RJMCMCTracker::normalizeMoveProb()
{
	double prob_sum = prob_move_add_ + prob_move_delete_ 
									+ prob_move_stay_ + prob_move_leave_ + prob_move_update_ 
									+	prob_move_interaction_flip_ 
									+ prob_move_feat_stay_ + prob_move_feat_leave_ + prob_move_feat_update_ 
									+ prob_move_cam_update_;

	prob_move_add_ /= prob_sum;
	prob_move_delete_ /= prob_sum;
	prob_move_stay_ /= prob_sum;
	prob_move_leave_ /= prob_sum;
	prob_move_update_ /= prob_sum;
	prob_move_interaction_flip_ /= prob_sum;
	prob_move_feat_stay_ /= prob_sum;
	prob_move_feat_leave_ /= prob_sum;
	prob_move_feat_update_ /= prob_sum;
	prob_move_cam_update_ /= prob_sum;
}

void RJMCMCTracker::prefilter_features(double timestamp)
{
	ObservationManager *mgr = obs_wrapper_.getManager();
	/////////////////////////////////////////////////////
	// get previous targets
	/////////////////////////////////////////////////////
	std::vector<cv::Rect> target_rts;

	if(prev_dist_->getNumTargets() > 0) {
		CamStatePtr cam_state;
		cam_state = prev_dist_->getMeanCamera(); // initialize by previous camera
		cam_state = cam_state->predict(timestamp);

		for(size_t i = 0; i < (size_t)prev_dist_->getNumTargets(); i ++) {
			cv::Rect rt;
			PeopleStatePtr ped_state = prev_dist_->getTarget(i)->getMean();
			rt = cam_state->project(ped_state->predict(timestamp));
			target_rts.push_back(rt);
		}
	}

	std::vector<int> del_idx = mgr->preprocessFeats(feat_idx_, max_gfeats_, target_rts);
	filterFeatures(del_idx);

	std::vector<int> new_feat_idx = mgr->getFeatsIndex();

	for(size_t i = 0; i < new_feat_idx.size(); i++) {
		if(i < feat_idx_.size())
			assert(feat_idx_[i] == new_feat_idx[i]);
		else
			feat_idx_.push_back(new_feat_idx[i]);
	}
	///////////////////////////////////////////////////////////////////////////////////
	// double check
	std::vector<cv::Point2f> feats = mgr->getAllFeats();
#if 0
	static int frame_cnt = 0;
	cv::Mat image = mgr->getImage().clone();
	for(size_t i = 0; i < target_rts.size(); i++) {
		cv::rectangle(image, target_rts[i].tl(), target_rts[i].br(), cv::Scalar(255, 0, 0), 2);
	}
	for(size_t i = 0; i < feats.size(); i++) {
		cv::Rect rt(feats[i].x - 5, feats[i].y - 5, 10, 10);
		cv::rectangle(image, rt.tl(), rt.br(), cv::Scalar(255, 0, 255), 2);
	}

	std::string filename = "/u/wchoi/temp2/frame";
	filename += boost::lexical_cast<std::string>(frame_cnt++);
	filename += ".jpg";
	cv::imwrite(filename, image);
#endif
	for(size_t i = 0; i < feats.size(); i++) {
		my_assert(!in_any_rect(target_rts, cv::Point2f(feats[i].x, feats[i].y)));
	}
}

CamStatePtr RJMCMCTracker::initializeCamera()
{
	// need to initialize camera..
	CamStatePtr prev_cam = prev_dist_->getMeanCamera();
	double 		timesec = obs_wrapper_.getTimeSec();
	CamStatePtr cur_cam = prev_cam->predict(timesec);
	
	// find horizon by detection votes
	std::vector<int> votes;
	std::vector<double> std;
	obs_wrapper_.getManager()->getHorizonVotes(votes, std, prev_cam->getY());
	
	int vote = 0;
	std::vector<int> inliers;
	std::vector<int> max_inliers;

	for(size_t i = 0; i < votes.size(); i++) {
		vote = votes[i];
		inliers.clear();

		for(size_t j = 0; j < votes.size(); j++) {
			if(abs(vote - votes[j]) < 3 * std[j]) {
				inliers.push_back(j);
			}
		}
		if(max_inliers.size() < inliers.size()) {
			max_inliers = inliers;
		}
	}

	double mean_hor = 0;
	for(size_t i = 0; i < max_inliers.size(); i++) {
		mean_hor += (double)votes[max_inliers[i]] / max_inliers.size();
	}

	if(mean_hor != 0) {
		cur_cam->setHorizon(mean_hor);
	}
	// find yaw/x/z/vx by few sampling - hold features, targets fixed
	return cur_cam;
}

// #define TESTING_CONF
void RJMCMCTracker::runMCMCSampling()
{	
	MCMCSamplePtr								one_sample;
	std::vector<MCMCSamplePtr> 	samples;
	SampleInfo 									info;
	double 										ar, rval;
	double 										obs_ar, prior_ar, q_ar, ms_ar;
	int 											counter = 0;
	double 										timesec = obs_wrapper_.getTimeSec();
	RJMCMCProposal 						prop_dist(prev_dist_, timesec);
	PriorDistCameraEstimate		motion_prior(prev_dist_, timesec);
	MeanShiftWrapper					ms_wrapper;

	// verification
	my_assert(ms_rts_.size() == ms_sims_.size());
	my_assert(ms_rts_.size() == (size_t)prev_dist_->getNumTargets());
	ms_wrapper.setData(ms_rts_, ms_sims_);
#ifdef SAVE_ALL_SAMPLES
	FILE *fp = fopen("/home/wgchoi/Codes/ros/ped_tracker/dbg_samples.txt", "w");
#endif
	if(!estimate_camera_) {
		prob_move_cam_update_ = prob_move_feat_update_ = prob_move_feat_leave_ = prob_move_feat_stay_ = 0.0;
	}
#ifdef TESTING_CONF
	if(timesec >= 0.12) {
		prob_move_stay_ = prob_move_leave_ = 0.0;
	}
#endif
	normalizeMoveProb();
	/************************************************
	 ** set paramters
	 ************************************************/
	/////////////////////////////////////////////////////////////////////////////////////////////////
	motion_prior.setParameters("MotionSigmaX", boost::lexical_cast<std::string>(motion_sigma_x_));
	motion_prior.setParameters("MotionSigmaY", boost::lexical_cast<std::string>(motion_sigma_y_));
	motion_prior.setParameters("MotionSigmaZ", boost::lexical_cast<std::string>(motion_sigma_z_));
#ifdef VEL_STATE
	motion_prior.setParameters("MotionSigmaVX", boost::lexical_cast<std::string>(motion_sigma_vx_));
	motion_prior.setParameters("MotionSigmaVZ", boost::lexical_cast<std::string>(motion_sigma_vz_));
#endif
	motion_prior.setParameters("ProbEnter", boost::lexical_cast<std::string>(prob_enter_));
	motion_prior.setParameters("ProbStay", boost::lexical_cast<std::string>(prob_stay_));
	/////////////////////////////////////////////////////////////////////////////////////////////////
	motion_prior.setParameters("CameraMotionSigmaX", boost::lexical_cast<std::string>(motion_cam_sigma_x_));
	motion_prior.setParameters("CameraMotionSigmaZ", boost::lexical_cast<std::string>(motion_cam_sigma_z_));
	motion_prior.setParameters("CameraMotionSigmaYaw", boost::lexical_cast<std::string>(motion_cam_sigma_yaw_));
	motion_prior.setParameters("CameraMotionSigmaV", boost::lexical_cast<std::string>(motion_cam_sigma_v_));
	motion_prior.setParameters("CameraMotionSigmaHorizon", boost::lexical_cast<std::string>(motion_cam_sigma_horizon_));
	/////////////////////////////////////////////////////////////////////////////////////////////////
	motion_prior.setParameters("ProbFeatEnter", boost::lexical_cast<std::string>(prob_feat_enter_));
	motion_prior.setParameters("ProbFeatStay", boost::lexical_cast<std::string>(prob_feat_stay_));
	/////////////////////////////////////////////////////////////////////////////////////////////////

	/////////////////////////////////////////////////////////////////////////////////////////////////
	prop_dist.setParameters("ProbMoveAdd", boost::lexical_cast<std::string>(prob_move_add_));
	prop_dist.setParameters("ProbMoveDelete", boost::lexical_cast<std::string>(prob_move_delete_));
	prop_dist.setParameters("ProbMoveStay", boost::lexical_cast<std::string>(prob_move_stay_));
	prop_dist.setParameters("ProbMoveLeave", boost::lexical_cast<std::string>(prob_move_leave_));
	prop_dist.setParameters("ProbMoveUpdate", boost::lexical_cast<std::string>(prob_move_update_));
	prop_dist.setParameters("ProbMoveInteractionFlip", boost::lexical_cast<std::string>(prob_move_interaction_flip_));
	prop_dist.setParameters("ProbMoveFeatStay", boost::lexical_cast<std::string>(prob_move_feat_stay_));
	prop_dist.setParameters("ProbMoveFeatLeave", boost::lexical_cast<std::string>(prob_move_feat_leave_));
	prop_dist.setParameters("ProbMoveFeatUpdate", boost::lexical_cast<std::string>(prob_move_feat_update_));
	prop_dist.setParameters("ProbMoveCamUpdate", boost::lexical_cast<std::string>(prob_move_cam_update_));
	/////////////////////////////////////////////////////////////////////////////////////////////////
	prop_dist.setParameters("DetectionSigmaX", boost::lexical_cast<std::string>(detection_sigma_x_));
	prop_dist.setParameters("DetectionSigmaY", boost::lexical_cast<std::string>(detection_sigma_y_));
	prop_dist.setParameters("DetectionSigmaH", boost::lexical_cast<std::string>(detection_sigma_h_));
	/////////////////////////////////////////////////////////////////////////////////////////////////
	prop_dist.setParameters("PerturbCamSigmaX", boost::lexical_cast<std::string>(perturb_cam_x_));
	prop_dist.setParameters("PerturbCamSigmaZ", boost::lexical_cast<std::string>(perturb_cam_z_));
	prop_dist.setParameters("PerturbCamSigmaYaw", boost::lexical_cast<std::string>(perturb_cam_yaw_));
	prop_dist.setParameters("PerturbCamSigmaV", boost::lexical_cast<std::string>(perturb_cam_v_));
	prop_dist.setParameters("PerturbCamSigmaHorizon", boost::lexical_cast<std::string>(perturb_cam_horizon_));
	/////////////////////////////////////////////////////////////////////////////////////////////////
	prop_dist.setParameters("PerturbFeatSigmaX", boost::lexical_cast<std::string>(perturb_feat_x_));
	prop_dist.setParameters("PerturbFeatSigmaZ", boost::lexical_cast<std::string>(perturb_feat_z_));
	/////////////////////////////////////////////////////////////////////////////////////////////////
	prop_dist.setParameters("PerturbSigmaX", boost::lexical_cast<std::string>(perturb_x_));
	prop_dist.setParameters("PerturbSigmaY", boost::lexical_cast<std::string>(perturb_y_));
	prop_dist.setParameters("PerturbSigmaZ", boost::lexical_cast<std::string>(perturb_z_));
#ifdef VEL_STATE
	prop_dist.setParameters("PerturbSigmaVX", boost::lexical_cast<std::string>(perturb_vx_));
	prop_dist.setParameters("PerturbSigmaVZ", boost::lexical_cast<std::string>(perturb_vz_));
	/////////////////////////////////////////////////////////////////////////////////////////////////
	prop_dist.setParameters("MotionSigmaX", boost::lexical_cast<std::string>(motion_sigma_x_));
	prop_dist.setParameters("MotionSigmaY", boost::lexical_cast<std::string>(motion_sigma_y_));
	prop_dist.setParameters("MotionSigmaZ", boost::lexical_cast<std::string>(motion_sigma_z_));
	prop_dist.setParameters("MotionSigmaVX", boost::lexical_cast<std::string>(motion_sigma_vx_));
	prop_dist.setParameters("MotionSigmaVZ", boost::lexical_cast<std::string>(motion_sigma_vz_));
	/////////////////////////////////////////////////////////////////////////////////////////////////
#endif
#if 0 // def MYDEBUG
	char msg[1000];
	open_dbg_file("/home/wgchoi/dbg_file.txt");
	// save all prior info
#endif
#ifdef VEL_STATE
	motion_prior.initMotionParameters();
#endif
	prefilter_features(timesec);
	/************************************************/
	prop_dist.setDetections(proposals_);
	// get initial sample..
	one_sample = getInitSample(timesec); // initialization
	samples.push_back(one_sample);
	/************************************************/
	// initialize caches..
	obs_wrapper_.initCache(one_sample);
	prop_dist.initCache(one_sample);
	motion_prior.initCache(one_sample);
	ms_wrapper.initCache(one_sample);
	/************************************************/

	init_sampling_history();
	while(++counter < num_samples_) {
		// generate new sample
		info = prop_dist.drawNewSample(one_sample);
		if(info.type_ == MoveNone) {
			samples.push_back(one_sample);
			// counter--; // not valid sample.. ignore
			continue;
		}

		if((info.type_ == MoveCamUpdate) && (prev_dist_->getMeanCamera().get() == NULL)) {
			samples.push_back(one_sample);
			continue;
		}
		// preprocess caches
		motion_prior.computeNewSampletMotionPrior(info);
		obs_wrapper_.computeNewSampleObservationLkhood(info, one_sample);
		ms_wrapper.computeNewSampleMSLkhood(info, one_sample);
		// compute observation lkhood
		obs_ar = obs_wrapper_.computeLogObservationLkhood(info, one_sample);
		// compute motion prior
#ifdef VEL_STATE
		prior_ar = motion_prior.computeLogMotionPrior(info, one_sample);
#else
		prior_ar = motion_prior.computeLogMotionPrior(info);
#endif
		// compute proposal ratio
		q_ar = prop_dist.getProposalLogLkhood(one_sample, info);
		// compute lkhood from MS
		ms_ar = ms_wrapper.computeLogMSLkhood(info);
		// evaluate acceptance ratio
		ar = obs_ar + prior_ar + q_ar + ms_ar;
#ifdef TESTING_CONF
		if(info.type_ == MoveUpdate && info.idx_ < prev_dist_->getNumTargets() && info.idx_ == 2 && timesec >= 0.12) {
			std::cout << "Move : Update " << ar << " idx " << info.idx_ << std::endl;
			std::cout << "from : ";
			one_sample->getState(info.idx_)->print();
			std::cout << "to : ";
			info.state_->print();
			std::cout << " obs : " << obs_ar 
					<< " pr : " << prior_ar 
					<< " q : " << q_ar 
					<< " ms : " << ms_ar << std::endl;

			// draw sample
			cv::Mat image = obs_wrapper_.getManager()->getImage().clone();
			CamStatePtr cam = one_sample->getCamState();
			PeopleStatePtr old_state = one_sample->getState(info.idx_);
			PeopleStatePtr new_state = info.state_;
			
			cv::Rect new_rt = cam->project(new_state);
			cv::Rect old_rt = cam->project(old_state);
			
			cv::rectangle(image, old_rt.tl(), old_rt.br(), cv::Scalar(0, 0, 0), 2);
			cv::rectangle(image, new_rt.tl(), new_rt.br(), cv::Scalar(255, 255, 255), 2);

			cv::imshow("dbg_tracker", image);
			cv::waitKey();
		}
#endif
#if 0
		if(info.type_ == MoveCamUpdate) {
			std::cout << "Move : update " << ar << std::endl;
			std::cout << "camera from : ";
			one_sample->getCamState()->print();
			std::cout << "camera to : ";
			info.cam_state_->print();
			std::cout << " obs : " << obs_ar 
					<< " pr : " << prior_ar 
					<< " q : " << q_ar 
					<< " ms : " << ms_ar << std::endl;

			cv::Mat image = obs_wrapper_.getManager()->getImage().clone();

			cv::Mat camera_view(image, cv::Rect(20, 20, 40, 40));
			camera_view = cv::Scalar(255, 255, 255);
			cv::rectangle(camera_view, cv::Point(0,0), cv::Point(39, 39), cv::Scalar(0,0,0), 3);
			// draw view point
			double angle = info.cam_state_->getYaw();
			double x = cos(angle), z = sin(angle);
			cv::line(camera_view, cv::Point(19, 19), cv::Point(19 + 20 * x, 19 + 20 * z), cv::Scalar(0,0,255), 1);
			angle = one_sample->getCamState()->getYaw();
			x = cos(angle), z = sin(angle);
			cv::line(camera_view, cv::Point(19, 19), cv::Point(19 + 20 * x, 19 + 20 * z), cv::Scalar(0,0,0), 1);

			cv::line(image, cv::Point(0, one_sample->getCamState()->getHorizon()), cv::Point(1000, one_sample->getCamState()->getHorizon()), cv::Scalar(0, 0, 0));
			cv::line(image, cv::Point(0, info.cam_state_->getHorizon()), cv::Point(1000, info.cam_state_->getHorizon()), cv::Scalar(0, 0, 255));
			
			cv::imshow("dbg_tracker", image);
			cv::waitKey();
		}
#endif
#if 0
		if(info.type_ == MoveStay) {
			std::cout << "Move : Stay " << ar << " idx " << info.idx_ << std::endl;
			info.state_->print();
			std::cout << " obs : " << obs_ar 
					<< " pr : " << prior_ar 
					<< " q : " << q_ar 
					<< " ms : " << ms_ar << std::endl;
			cv::waitKey();
		}
		else if(info.type_ == MoveLeave) {
			std::cout << "Move : Leave " << ar << " idx " << info.idx_ << std::endl;
			std::cout << " obs : " << obs_ar 
					<< " pr : " << prior_ar 
					<< " q : " << q_ar 
					<< " ms : " << ms_ar << std::endl;
			cv::waitKey();
		}
#endif
		rval = rng_.uniform((double)0.0, (double)1.0);
		if(ar > log(rval)) {
			// update sample..
			one_sample = one_sample->setNewSample(info);
			// update caches..
			obs_wrapper_.updateCache(info, true);
			prop_dist.updateCache(info, true);
			motion_prior.updateCache(info, true);
			ms_wrapper.updateCache(info, true);
			// record sampling history
			record_sampling_history(info, true);
		}
		else {
			record_sampling_history(info, false);
		}
#ifdef SAVE_ALL_SAMPLES
		save_sample_info(fp, counter, info, one_sample, ar, obs_ar, prior_ar, q_ar, ar > log(rval));
#endif
		samples.push_back(one_sample);
		/******************
		 **  debug    *****
		 ******************/
		prop_dist.dbgCheckCache(one_sample);
	}
	print_sampling_history();

	std::cout << "mcmc_tracker target cnt [0]: " << samples[0]->getNumTargets();
	std::cout << " [end] : " << samples[samples.size()-1]->getNumTargets() << std::endl;

	/* Target and Sample managements.... */
	// finalize the tracking.. you can use burnin, thining here!!
	// clear previous samples..
	prev_dist_->setSamples(samples, burnin_, thinning_);
	if(show_dbg_msg_) {
		prev_dist_->print_target_summary();
	}
#ifdef SAVE_ALL_SAMPLES
	fclose(fp);
#endif
}
